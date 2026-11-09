// SPDX-FileCopyrightText: Copyright 2026 Eden Emulator Project
// SPDX-License-Identifier: GPL-3.0-or-later

// SPDX-FileCopyrightText: Copyright 2018 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <optional>
#include <span>
#include <string_view>
#include <string>
#include <utility>
#include <vector>

#include <zlib.h>

#include "common/assert.h"
#include "common/common_types.h"
#include "common/hex_util.h"
#include "common/logging.h"
#include "common/settings.h"
#include "common/string_util.h"
#include "core/core.h"
#include "core/file_sys/content_archive.h"
#include "core/file_sys/errors.h"
#include "core/file_sys/fs_directory.h"
#include "core/file_sys/fs_filesystem.h"
#include "core/file_sys/nca_metadata.h"
#include "core/file_sys/patch_manager.h"
#include "core/file_sys/romfs.h"
#include "core/file_sys/romfs_factory.h"
#include "core/file_sys/savedata_factory.h"
#include "core/file_sys/system_archive/system_archive.h"
#include "core/file_sys/vfs/vfs.h"
#include "core/hle/result.h"
#include "core/hle/service/cmif_serialization.h"
#include "core/hle/service/filesystem/filesystem.h"
#include "core/hle/service/filesystem/fsp/fs_i_filesystem.h"
#include "core/hle/service/filesystem/fsp/fs_i_multi_commit_manager.h"
#include "core/hle/service/filesystem/fsp/fs_i_save_data_info_reader.h"
#include "core/hle/service/filesystem/fsp/fs_i_storage.h"
#include "core/hle/service/filesystem/fsp/fsp_srv.h"
#include "core/hle/service/filesystem/fsp/save_data_transfer_prohibiter.h"
#include "core/hle/service/filesystem/romfs_controller.h"
#include "core/hle/service/filesystem/save_data_controller.h"
#include "core/hle/service/hle_ipc.h"
#include "core/hle/service/ipc_helpers.h"
#include "core/loader/loader.h"
#include "core/reporter.h"
#include "common/fs/file.h"
#include "common/fs/fs.h"
#include "core/file_sys/vfs/vfs_offset.h"
#include "core/file_sys/vfs/vfs_vector.h"

namespace Service::FileSystem {

namespace {

// Experiment: re-lays out the textures of a Switch 1 NXTexture file, which are stored
// block-linear with the NVN block height rules. EDEN_NX2_MII_TEX_LAYOUT picks what the
// OunceTexture copies get:
//   "bl1" (default) block-linear level 0 only, mip count set to 1
//   "blkeep"        block-linear, every mip keeps level 0's block height
//   "blone"         block-linear, block height 1 for every mip
//   "bl"            unchanged
//   "linear"        tightly packed rows
//   "linear32"      rows aligned to 32 bytes, each mip aligned to 512 bytes
enum class MiiTexLayout {
    BlockLinear,
    BlockLinearLevel0,
    BlockLinearKeep,
    BlockLinearOne,
    Linear,
    Linear32,
};

MiiTexLayout GetMiiTexLayout() {
    static const MiiTexLayout layout = [] {
        const char* setting = std::getenv("EDEN_NX2_MII_TEX_LAYOUT");
        const std::string_view value = setting != nullptr ? setting : "bl1";
        if (value == "bl") {
            return MiiTexLayout::BlockLinear;
        }
        if (value == "blkeep") {
            return MiiTexLayout::BlockLinearKeep;
        }
        if (value == "blone") {
            return MiiTexLayout::BlockLinearOne;
        }
        if (value == "linear") {
            return MiiTexLayout::Linear;
        }
        if (value == "linear32") {
            return MiiTexLayout::Linear32;
        }
        return MiiTexLayout::BlockLinearLevel0;
    }();
    return layout;
}

constexpr std::size_t DivCeil(std::size_t a, std::size_t b) {
    return (a + b - 1) / b;
}

// Byte offset of (x bytes, y rows) inside a block-linear surface of the given GOB block height.
std::size_t BlockLinearOffset(std::size_t x, std::size_t y, std::size_t gobs_wide,
                              std::size_t block_height) {
    const std::size_t block = (y / (8 * block_height)) * gobs_wide + x / 64;
    const std::size_t gob = (y % (8 * block_height)) / 8;
    const std::size_t in_gob = ((x % 64) / 32) * 256 + ((y % 8) / 2) * 64 + ((x % 32) / 16) * 32 +
                               (y % 2) * 16 + (x % 16);
    return block * 512 * block_height + gob * 512 + in_gob;
}

std::size_t BlockLinearSize(std::size_t row_bytes, std::size_t rows, std::size_t block_height) {
    return DivCeil(row_bytes, 64) * DivCeil(DivCeil(rows, 8), block_height) * block_height * 512;
}

// Converts one texture; `mips` is updated when the layout drops levels.
std::optional<std::vector<u8>> ConvertMiiTexture(std::span<const u8> src, u32 width, u32 height,
                                                 u8 format, u8& mips, MiiTexLayout layout) {
    std::size_t bytes_per_block;
    std::size_t block_dim;
    switch (format) {
    case 3:
        bytes_per_block = 8;
        block_dim = 4;
        break;
    case 4:
        bytes_per_block = 1;
        block_dim = 1;
        break;
    case 6:
        bytes_per_block = 16;
        block_dim = 4;
        break;
    default:
        return std::nullopt;
    }

    const std::size_t rows0 = DivCeil(height, block_dim);
    std::size_t block_height0 = 1;
    while (block_height0 < 16 && block_height0 * 8 < rows0) {
        block_height0 *= 2;
    }
    const u8 out_mips = layout == MiiTexLayout::BlockLinearLevel0 ? u8{1} : mips;

    std::vector<u8> out;
    std::size_t src_offset = 0;
    for (u8 mip = 0; mip < mips; ++mip) {
        const std::size_t row_bytes =
            DivCeil(std::max<u32>(1, width >> mip), block_dim) * bytes_per_block;
        const std::size_t rows = DivCeil(std::max<u32>(1, height >> mip), block_dim);
        const std::size_t gobs_wide = DivCeil(row_bytes, 64);
        std::size_t block_height = block_height0;
        while (block_height > 1 && block_height / 2 >= DivCeil(rows, 8)) {
            block_height /= 2;
        }
        const std::size_t mip_size = BlockLinearSize(row_bytes, rows, block_height);
        if (src_offset + mip_size > src.size()) {
            return std::nullopt;
        }
        if (mip >= out_mips) {
            break;
        }

        const auto source_at = [&](std::size_t x, std::size_t y) {
            return src[src_offset + BlockLinearOffset(x, y, gobs_wide, block_height)];
        };
        if (layout == MiiTexLayout::Linear || layout == MiiTexLayout::Linear32) {
            const std::size_t pitch =
                layout == MiiTexLayout::Linear32 ? DivCeil(row_bytes, 32) * 32 : row_bytes;
            if (layout == MiiTexLayout::Linear32) {
                out.resize(DivCeil(out.size(), 512) * 512);
            }
            const std::size_t base = out.size();
            out.resize(base + pitch * rows);
            for (std::size_t y = 0; y < rows; ++y) {
                for (std::size_t x = 0; x < row_bytes; ++x) {
                    out[base + y * pitch + x] = source_at(x, y);
                }
            }
        } else {
            std::size_t out_block_height = block_height;
            if (layout == MiiTexLayout::BlockLinearKeep) {
                out_block_height = block_height0;
            } else if (layout == MiiTexLayout::BlockLinearOne) {
                out_block_height = 1;
            }
            const std::size_t base = out.size();
            out.resize(base + BlockLinearSize(row_bytes, rows, out_block_height));
            for (std::size_t y = 0; y < rows; ++y) {
                for (std::size_t x = 0; x < row_bytes; ++x) {
                    out[base + BlockLinearOffset(x, y, gobs_wide, out_block_height)] =
                        source_at(x, y);
                }
            }
        }
        src_offset += mip_size;
    }
    mips = out_mips;
    return out;
}

// Rebuilds an NXTexture file with every texture converted to the requested layout.
FileSys::VirtualFile ConvertMiiTextureFile(const FileSys::VirtualFile& file,
                                           const std::string& name) {
    const auto layout = GetMiiTexLayout();
    if (layout == MiiTexLayout::BlockLinear) {
        return std::make_shared<FileSys::OffsetVfsFile>(file, file->GetSize(), 0, name);
    }

    constexpr std::size_t TableOffset = 0x64;
    constexpr std::size_t EntrySize = 0x1C;
    const auto in = file->ReadAllBytes();
    if (in.size() < TableOffset || std::memcmp(in.data(), "NFTR", 4) != 0) {
        return nullptr;
    }
    const auto read32 = [&in](std::size_t pos) {
        u32 value;
        std::memcpy(&value, in.data() + pos, sizeof(value));
        return value;
    };

    std::size_t data_start = in.size();
    for (std::size_t pos = TableOffset; pos + EntrySize <= data_start; pos += EntrySize) {
        if (read32(pos + 8) != 0) {
            data_start = std::min<std::size_t>(data_start, read32(pos));
        }
    }

    std::vector<u8> out(in.begin(), in.begin() + data_start);
    std::size_t converted = 0;
    std::size_t kept = 0;
    for (std::size_t pos = TableOffset; pos + EntrySize <= data_start; pos += EntrySize) {
        const u32 offset = read32(pos);
        const u32 size = read32(pos + 4);
        const u32 compressed_size = read32(pos + 8);
        if (compressed_size == 0) {
            continue;
        }
        if (static_cast<std::size_t>(offset) + compressed_size > in.size()) {
            return nullptr;
        }

        std::vector<u8> texture(size);
        uLongf texture_size = size;
        if (uncompress(texture.data(), &texture_size, in.data() + offset, compressed_size) !=
                Z_OK ||
            texture_size != size) {
            return nullptr;
        }

        u16 width;
        u16 height;
        std::memcpy(&width, in.data() + pos + 0x14, sizeof(width));
        std::memcpy(&height, in.data() + pos + 0x16, sizeof(height));
        const u8 format = in[pos + 0x18];
        u8 mips = in[pos + 0x19];

        auto converted_texture = ConvertMiiTexture(texture, width, height, format, mips, layout);
        if (converted_texture) {
            texture = std::move(*converted_texture);
            out[pos + 0x19] = mips;
            ++converted;
        } else {
            ++kept;
        }

        uLongf packed_size = compressBound(static_cast<uLong>(texture.size()));
        std::vector<u8> packed(packed_size);
        if (compress2(packed.data(), &packed_size, texture.data(),
                      static_cast<uLong>(texture.size()), 6) != Z_OK) {
            return nullptr;
        }

        const u32 new_offset = static_cast<u32>(out.size());
        const u32 new_size = static_cast<u32>(texture.size());
        const u32 new_compressed_size = static_cast<u32>(packed_size);
        std::memcpy(out.data() + pos, &new_offset, sizeof(new_offset));
        std::memcpy(out.data() + pos + 4, &new_size, sizeof(new_size));
        std::memcpy(out.data() + pos + 8, &new_compressed_size, sizeof(new_compressed_size));
        out.insert(out.end(), packed.begin(), packed.begin() + packed_size);
    }

    const u32 total = static_cast<u32>(out.size());
    std::memcpy(out.data() + 8, &total, sizeof(total));
    LOG_WARNING(Service_FS, "Mii texture layout: {} converted, {} kept as they were, for {}",
                converted, kept, name);
    return std::make_shared<FileSys::VectorVfsFile>(std::move(out), name);
}

// Experiment: Switch 2 titles look for OunceTexture*.dat (Low, Mid and High) in the Mii model
// archive, which the Switch 1 MiiModel does not have. Rebuild the archive with the Switch 1
// NXTexture files also served under those names (High uses Mid).
// EDEN_NX2_MII_OUNCE_ALIAS=0 serves the Switch 1 archive unchanged.
FileSys::VirtualFile AddOunceTextureAliases(FileSys::VirtualFile data) {
    static const bool enabled = [] {
        const char* setting = std::getenv("EDEN_NX2_MII_OUNCE_ALIAS");
        return setting == nullptr || setting[0] != '0';
    }();
    if (!enabled) {
        return data;
    }
    const auto root = FileSys::ExtractRomFS(data);
    if (root == nullptr) {
        return data;
    }

    constexpr std::array<std::pair<const char*, const char*>, 6> aliases{{
        {"OunceTextureLowLinear.dat", "NXTextureLowLinear.dat"},
        {"OunceTextureLowSRGB.dat", "NXTextureLowSRGB.dat"},
        {"OunceTextureMidLinear.dat", "NXTextureMidLinear.dat"},
        {"OunceTextureMidSRGB.dat", "NXTextureMidSRGB.dat"},
        {"OunceTextureHighLinear.dat", "NXTextureMidLinear.dat"},
        {"OunceTextureHighSRGB.dat", "NXTextureMidSRGB.dat"},
    }};

    std::vector<FileSys::VirtualFile> files = root->GetFiles();
    std::size_t added = 0;
    for (const auto& [alias, source_name] : aliases) {
        if (root->GetFile(alias) != nullptr) {
            continue;
        }
        const auto source = root->GetFile(source_name);
        if (source == nullptr) {
            continue;
        }
        auto converted = ConvertMiiTextureFile(source, alias);
        if (converted == nullptr) {
            LOG_WARNING(Service_FS, "Could not convert {}, serving it unchanged", source_name);
            converted =
                std::make_shared<FileSys::OffsetVfsFile>(source, source->GetSize(), 0, alias);
        }
        files.push_back(std::move(converted));
        ++added;
    }
    if (added == 0) {
        return data;
    }

    auto rebuilt = FileSys::CreateRomFS(std::make_shared<FileSys::VectorVfsDirectory>(
        std::move(files), root->GetSubdirectories(), "data"));
    if (rebuilt == nullptr) {
        LOG_WARNING(Service_FS, "Could not rebuild MiiModel with OunceTexture aliases");
        return data;
    }
    LOG_WARNING(Service_FS, "Serving {} NXTexture files as OunceTexture for the Mii model",
                added);
    return rebuilt;
}

// Diagnostics for the Switch 2 Mii model fallback, off unless their variable is set.
// EDEN_NX2_MII_TRACE=1 logs the served archive's files and every guest read mapped to a file.
// EDEN_NX2_MII_DUMP=<dir> writes the served archive's files to <dir>.
class MiiTraceFile final : public FileSys::VfsFile {
public:
    struct Entry {
        std::size_t begin;
        std::size_t end;
        std::string path;
    };

    MiiTraceFile(FileSys::VirtualFile backend_, std::vector<Entry> entries_)
        : backend{std::move(backend_)}, entries{std::move(entries_)} {}

    std::string GetName() const override {
        return backend->GetName();
    }
    std::size_t GetSize() const override {
        return backend->GetSize();
    }
    bool Resize(std::size_t) override {
        return false;
    }
    FileSys::VirtualDir GetContainingDirectory() const override {
        return backend->GetContainingDirectory();
    }
    bool IsWritable() const override {
        return false;
    }
    bool IsReadable() const override {
        return true;
    }
    std::size_t Write(const u8*, std::size_t, std::size_t) override {
        return 0;
    }
    bool Rename(std::string_view) override {
        return false;
    }

    std::size_t Read(u8* data, std::size_t length, std::size_t offset) const override {
        if (logged < 4000) {
            ++logged;
            std::string where = "romfs metadata";
            for (const auto& entry : entries) {
                if (offset < entry.end && offset + length > entry.begin) {
                    where = fmt::format("{} +{:#x}", entry.path,
                                        offset > entry.begin ? offset - entry.begin : 0);
                    break;
                }
            }
            LOG_INFO(Service_FS, "Mii trace: read {:#x} bytes at {:#x} ({})", length, offset,
                     where);
        }
        return backend->Read(data, length, offset);
    }

private:
    FileSys::VirtualFile backend;
    std::vector<Entry> entries;
    mutable std::size_t logged = 0;
};

// Reads the file table of a plain RomFS: name, absolute offset and size of every file.
std::vector<MiiTraceFile::Entry> ListRomFSFiles(const FileSys::VirtualFile& romfs) {
    std::vector<MiiTraceFile::Entry> out;
    const auto header = romfs->ReadBytes(0x50);
    if (header.size() != 0x50) {
        return out;
    }
    const auto u64_at = [&header](std::size_t i) {
        u64 value;
        std::memcpy(&value, header.data() + i * 8, sizeof(value));
        return value;
    };
    const u64 file_meta_offset = u64_at(7);
    const u64 file_meta_size = u64_at(8);
    const u64 data_offset = u64_at(9);
    const auto meta = romfs->ReadBytes(file_meta_size, file_meta_offset);

    std::size_t pos = 0;
    while (pos + 0x20 <= meta.size()) {
        u64 offset;
        u64 size;
        u32 name_size;
        std::memcpy(&offset, meta.data() + pos + 0x8, sizeof(offset));
        std::memcpy(&size, meta.data() + pos + 0x10, sizeof(size));
        std::memcpy(&name_size, meta.data() + pos + 0x1C, sizeof(name_size));
        if (pos + 0x20 + name_size > meta.size()) {
            break;
        }
        std::string name(reinterpret_cast<const char*>(meta.data() + pos + 0x20), name_size);
        const std::size_t begin = data_offset + offset;
        out.push_back({begin, begin + size, std::move(name)});
        pos += 0x20 + ((name_size + 3) & ~std::size_t{3});
    }
    return out;
}

FileSys::VirtualFile InstrumentMiiModel(FileSys::VirtualFile data) {
    const char* trace = std::getenv("EDEN_NX2_MII_TRACE");
    const char* dump = std::getenv("EDEN_NX2_MII_DUMP");
    const bool tracing = trace != nullptr && trace[0] != '0';
    if (!tracing && (dump == nullptr || dump[0] == '\0')) {
        return data;
    }

    auto entries = ListRomFSFiles(data);
    for (const auto& entry : entries) {
        const std::size_t size = entry.end - entry.begin;
        const auto bytes = data->ReadBytes(std::min<std::size_t>(size, 32), entry.begin);
        LOG_INFO(Service_FS, "Mii trace: file {} size={:#x} at {:#x} head={}", entry.path, size,
                 entry.begin, Common::HexToString(bytes));

        if (dump != nullptr && dump[0] != '\0') {
            const std::filesystem::path out_path = std::filesystem::path{dump} / entry.path;
            (void)Common::FS::CreateParentDirs(out_path);
            Common::FS::IOFile out{out_path, Common::FS::FileAccessMode::Write,
                                   Common::FS::FileType::BinaryFile};
            const auto all = data->ReadBytes(size, entry.begin);
            if (out.IsOpen() && out.WriteSpan(std::span<const u8>{all}) == all.size()) {
                LOG_INFO(Service_FS, "Mii trace: dumped {}", out_path.string());
            } else {
                LOG_WARNING(Service_FS, "Mii trace: could not write {}", out_path.string());
            }
        }
    }

    if (!tracing) {
        return data;
    }
    return std::make_shared<MiiTraceFile>(std::move(data), std::move(entries));
}

} // namespace

FSP_SRV::FSP_SRV(Core::System& system_)
    : ServiceFramework{system_, "fsp-srv"}, fsc{system.GetFileSystemController()},
      content_provider{system.GetContentProvider()}, reporter{system.GetReporter()} {
    // clang-format off
    static const FunctionInfo functions[] = {
        {0, nullptr, "OpenFileSystem"},
        {1, D<&FSP_SRV::SetCurrentProcess>, "SetCurrentProcess"},
        {2, D<&FSP_SRV::OpenDataFileSystemByCurrentProcess>, "OpenDataFileSystemByCurrentProcess"},
        {7, D<&FSP_SRV::OpenFileSystemWithPatch>, "OpenFileSystemWithPatch"},
        {8, nullptr, "OpenFileSystemWithId"},
        {9, nullptr, "OpenDataFileSystemByApplicationId"},
        {11, nullptr, "OpenBisFileSystem"},
        {12, nullptr, "OpenBisStorage"},
        {13, nullptr, "InvalidateBisCache"},
        {17, nullptr, "OpenHostFileSystem"},
        {18, D<&FSP_SRV::OpenSdCardFileSystem>, "OpenSdCardFileSystem"},
        {19, nullptr, "FormatSdCardFileSystem"},
        {21, nullptr, "DeleteSaveDataFileSystem"},
        {22, D<&FSP_SRV::CreateSaveDataFileSystem>, "CreateSaveDataFileSystem"},
        {23, D<&FSP_SRV::CreateSaveDataFileSystemBySystemSaveDataId>, "CreateSaveDataFileSystemBySystemSaveDataId"},
        {24, nullptr, "RegisterSaveDataFileSystemAtomicDeletion"},
        {25, nullptr, "DeleteSaveDataFileSystemBySaveDataSpaceId"},
        {26, nullptr, "FormatSdCardDryRun"},
        {27, D<&FSP_SRV::IsExFatSupported>, "IsExFatSupported"},
        {28, nullptr, "DeleteSaveDataFileSystemBySaveDataAttribute"},
        {30, nullptr, "OpenGameCardStorage"},
        {31, nullptr, "OpenGameCardFileSystem"},
        {32, D<&FSP_SRV::ExtendSaveDataFileSystem>, "ExtendSaveDataFileSystem"},
        {33, nullptr, "DeleteCacheStorage"},
        {34, D<&FSP_SRV::GetCacheStorageSize>, "GetCacheStorageSize"},
        {35, nullptr, "CreateSaveDataFileSystemByHashSalt"},
        {36, nullptr, "OpenHostFileSystemWithOption"},
        {37, D<&FSP_SRV::CreateSaveDataFileSystemWithCreationInfo2>, "CreateSaveDataFileSystemWithCreationInfo2"},
        {51, D<&FSP_SRV::OpenSaveDataFileSystem>, "OpenSaveDataFileSystem"},
        {52, D<&FSP_SRV::OpenSaveDataFileSystemBySystemSaveDataId>, "OpenSaveDataFileSystemBySystemSaveDataId"},
        {53, D<&FSP_SRV::OpenReadOnlySaveDataFileSystem>, "OpenReadOnlySaveDataFileSystem"},
        {57, D<&FSP_SRV::ReadSaveDataFileSystemExtraDataBySaveDataSpaceId>, "ReadSaveDataFileSystemExtraDataBySaveDataSpaceId"},
        {58, D<&FSP_SRV::ReadSaveDataFileSystemExtraData>, "ReadSaveDataFileSystemExtraData"},
        {59, D<&FSP_SRV::WriteSaveDataFileSystemExtraData>, "WriteSaveDataFileSystemExtraData"},
        {60, nullptr, "OpenSaveDataInfoReader"},
        {61, D<&FSP_SRV::OpenSaveDataInfoReaderBySaveDataSpaceId>, "OpenSaveDataInfoReaderBySaveDataSpaceId"},
        {62, D<&FSP_SRV::OpenSaveDataInfoReaderOnlyCacheStorage>, "OpenSaveDataInfoReaderOnlyCacheStorage"},
        {64, nullptr, "OpenSaveDataInternalStorageFileSystem"},
        {65, nullptr, "UpdateSaveDataMacForDebug"},
        {66, nullptr, "WriteSaveDataFileSystemExtraData2"},
        {67, D<&FSP_SRV::FindSaveDataWithFilter>, "FindSaveDataWithFilter"},
        {68, nullptr, "OpenSaveDataInfoReaderBySaveDataFilter"},
        {69, D<&FSP_SRV::ReadSaveDataFileSystemExtraDataBySaveDataAttribute>, "ReadSaveDataFileSystemExtraDataBySaveDataAttribute"},
        {70, D<&FSP_SRV::WriteSaveDataFileSystemExtraDataWithMaskBySaveDataAttribute>, "WriteSaveDataFileSystemExtraDataWithMaskBySaveDataAttribute"},
        {71, D<&FSP_SRV::ReadSaveDataFileSystemExtraDataWithMaskBySaveDataAttribute>, "ReadSaveDataFileSystemExtraDataWithMaskBySaveDataAttribute"},
        {80, nullptr, "OpenSaveDataMetaFile"},
        {81, nullptr, "OpenSaveDataTransferManager"},
        {82, nullptr, "OpenSaveDataTransferManagerVersion2"},
        {83, D<&FSP_SRV::OpenSaveDataTransferProhibiter>, "OpenSaveDataTransferProhibiter"},
        {84, nullptr, "ListApplicationAccessibleSaveDataOwnerId"},
        {85, nullptr, "OpenSaveDataTransferManagerForSaveDataRepair"},
        {86, nullptr, "OpenSaveDataMover"},
        {87, nullptr, "OpenSaveDataTransferManagerForRepair"},
        {100, nullptr, "OpenImageDirectoryFileSystem"},
        {101, nullptr, "OpenBaseFileSystem"},
        {102, nullptr, "FormatBaseFileSystem"},
        {110, nullptr, "OpenContentStorageFileSystem"},
        {120, nullptr, "OpenCloudBackupWorkStorageFileSystem"},
        {130, nullptr, "OpenCustomStorageFileSystem"},
        {200, D<&FSP_SRV::OpenDataStorageByCurrentProcess>, "OpenDataStorageByCurrentProcess"},
        {201, nullptr, "OpenDataStorageByProgramId"},
        {202, D<&FSP_SRV::OpenDataStorageByDataId>, "OpenDataStorageByDataId"},
        {203, D<&FSP_SRV::OpenPatchDataStorageByCurrentProcess>, "OpenPatchDataStorageByCurrentProcess"},
        {204, nullptr, "OpenDataFileSystemByProgramIndex"},
        {205, D<&FSP_SRV::OpenDataStorageWithProgramIndex>, "OpenDataStorageWithProgramIndex"},
        {206, nullptr, "OpenDataStorageByPath"},
        {210, D<&FSP_SRV::SetCurrentProcess>, "SetCurrentProcess"},
        {400, nullptr, "OpenDeviceOperator"},
        {500, nullptr, "OpenSdCardDetectionEventNotifier"},
        {501, nullptr, "OpenGameCardDetectionEventNotifier"},
        {510, nullptr, "OpenSystemDataUpdateEventNotifier"},
        {511, nullptr, "NotifySystemDataUpdateEvent"},
        {520, nullptr, "SimulateGameCardDetectionEvent"},
        {600, nullptr, "SetCurrentPosixTime"},
        {601, nullptr, "QuerySaveDataTotalSize"},
        {602, nullptr, "VerifySaveDataFileSystem"},
        {603, nullptr, "CorruptSaveDataFileSystem"},
        {604, nullptr, "CreatePaddingFile"},
        {605, nullptr, "DeleteAllPaddingFiles"},
        {606, nullptr, "GetRightsId"},
        {607, nullptr, "RegisterExternalKey"},
        {608, nullptr, "UnregisterAllExternalKey"},
        {609, nullptr, "GetRightsIdByPath"},
        {610, nullptr, "GetRightsIdAndKeyGenerationByPath"},
        {611, nullptr, "SetCurrentPosixTimeWithTimeDifference"},
        {612, nullptr, "GetFreeSpaceSizeForSaveData"},
        {613, nullptr, "VerifySaveDataFileSystemBySaveDataSpaceId"},
        {614, nullptr, "CorruptSaveDataFileSystemBySaveDataSpaceId"},
        {615, nullptr, "QuerySaveDataInternalStorageTotalSize"},
        {616, nullptr, "GetSaveDataCommitId"},
        {617, nullptr, "UnregisterExternalKey"},
        {620, nullptr, "SetSdCardEncryptionSeed"},
        {630, nullptr, "SetSdCardAccessibility"},
        {631, D<&FSP_SRV::IsSdCardAccessible>, "IsSdCardAccessible"},
        {640, nullptr, "IsSignedSystemPartitionOnSdCardValid"},
        {700, nullptr, "OpenAccessFailureResolver"},
        {701, nullptr, "GetAccessFailureDetectionEvent"},
        {702, nullptr, "IsAccessFailureDetected"},
        {710, nullptr, "ResolveAccessFailure"},
        {720, nullptr, "AbandonAccessFailure"},
        {800, nullptr, "GetAndClearFileSystemProxyErrorInfo"},
        {810, nullptr, "RegisterProgramIndexMapInfo"},
        {820, nullptr, "GetContentStorageInfoIndex"},
        {830, nullptr, "EncryptStreamPlaySaveData"},
        {831, nullptr, "DecryptStreamPlaySaveData"},
        {1000, nullptr, "SetBisRootForHost"},
        {1001, nullptr, "SetSaveDataSize"},
        {1002, nullptr, "SetSaveDataRootPath"},
        {1003, D<&FSP_SRV::DisableAutoSaveDataCreation>, "DisableAutoSaveDataCreation"},
        {1004, D<&FSP_SRV::SetGlobalAccessLogMode>, "SetGlobalAccessLogMode"},
        {1005, D<&FSP_SRV::GetGlobalAccessLogMode>, "GetGlobalAccessLogMode"},
        {1006, D<&FSP_SRV::OutputAccessLogToSdCard>, "OutputAccessLogToSdCard"},
        {1007, nullptr, "RegisterUpdatePartition"},
        {1008, nullptr, "OpenRegisteredUpdatePartition"},
        {1009, nullptr, "GetAndClearMemoryReportInfo"},
        {1010, nullptr, "SetDataStorageRedirectTarget"},
        {1011, D<&FSP_SRV::GetProgramIndexForAccessLog>, "GetProgramIndexForAccessLog"},
        {1012, nullptr, "GetFsStackUsage"},
        {1013, nullptr, "UnsetSaveDataRootPath"},
        {1014, nullptr, "OutputMultiProgramTagAccessLog"},
        {1016, D<&FSP_SRV::FlushAccessLogOnSdCard>, "FlushAccessLogOnSdCard"},
        {1017, nullptr, "OutputApplicationInfoAccessLog"},
        {1018, nullptr, "SetDebugOption"},
        {1019, nullptr, "UnsetDebugOption"},
        {1100, nullptr, "OverrideSaveDataTransferTokenSignVerificationKey"},
        {1110, nullptr, "CorruptSaveDataFileSystemBySaveDataSpaceId2"},
        {1200, D<&FSP_SRV::OpenMultiCommitManager>, "OpenMultiCommitManager"},
        {1300, nullptr, "OpenBisWiper"},
    };
    // clang-format on
    RegisterHandlers(functions);

    if (Settings::values.enable_fs_access_log) {
        access_log_mode = AccessLogMode::SdCard;
    }
}

FSP_SRV::~FSP_SRV() = default;

Result FSP_SRV::SetCurrentProcess(ClientProcessId pid) {
    current_process_id = *pid;

    LOG_DEBUG(Service_FS, "called. current_process_id={:#016x}", current_process_id);

    R_RETURN(
        fsc.OpenProcess(&program_id, &save_data_controller, &romfs_controller, current_process_id));
}

Result FSP_SRV::OpenFileSystemWithPatch(OutInterface<IFileSystem> out_interface,
                                        FileSystemProxyType type, u64 open_program_id) {
    LOG_ERROR(Service_FS, "(STUBBED) called with type={}, program_id={:016X}", type,
              open_program_id);

    // FIXME: many issues with this
    ASSERT(type == FileSystemProxyType::Manual);
    const auto manual_romfs = romfs_controller->OpenPatchedRomFS(
        open_program_id, FileSys::ContentRecordType::HtmlDocument);

    ASSERT(manual_romfs != nullptr);

    const auto extracted_romfs = FileSys::ExtractRomFS(manual_romfs);
    ASSERT(extracted_romfs != nullptr);

    *out_interface = std::make_shared<IFileSystem>(
        system, extracted_romfs, SizeGetter::FromStorageId(fsc, FileSys::StorageId::NandUser));

    R_SUCCEED();
}

Result FSP_SRV::OpenSdCardFileSystem(OutInterface<IFileSystem> out_interface) {
    LOG_DEBUG(Service_FS, "called");

    FileSys::VirtualDir sdmc_dir{};
    fsc.OpenSDMC(&sdmc_dir);

    *out_interface = std::make_shared<IFileSystem>(
        system, sdmc_dir, SizeGetter::FromStorageId(fsc, FileSys::StorageId::SdCard));

    R_SUCCEED();
}

Result FSP_SRV::CreateSaveDataFileSystem(FileSys::SaveDataCreationInfo save_create_struct,
                                         FileSys::SaveDataAttribute save_struct, u128 uid) {
    LOG_DEBUG(Service_FS, "called save_struct = {}, uid = {:016X}{:016X}", save_struct.DebugInfo(),
              uid[1], uid[0]);

    FileSys::VirtualDir save_data_dir{};
    R_RETURN(save_data_controller->CreateSaveData(&save_data_dir, FileSys::SaveDataSpaceId::User,
                                                  save_struct));
}

Result FSP_SRV::CreateSaveDataFileSystemBySystemSaveDataId(
    FileSys::SaveDataAttribute save_struct, FileSys::SaveDataCreationInfo save_create_struct) {
    LOG_DEBUG(Service_FS, "called save_struct = {}", save_struct.DebugInfo());

    FileSys::VirtualDir save_data_dir{};
    R_RETURN(save_data_controller->CreateSaveData(&save_data_dir, FileSys::SaveDataSpaceId::System,
                                                  save_struct));
}

Result FSP_SRV::CreateSaveDataFileSystemWithCreationInfo2(
    FileSys::SaveDataCreationInfo2 save_data_creation_info) {
    FileSys::VirtualDir save_data_dir{};
    R_RETURN(save_data_controller->CreateSaveData(&save_data_dir, save_data_creation_info.space_id,
                                                  save_data_creation_info.attribute));
}

Result FSP_SRV::IsExFatSupported(Out<bool> out_is_supported) {
    LOG_WARNING(Service_FS, "(STUBBED) called");

    *out_is_supported = true;

    R_SUCCEED();
}

Result FSP_SRV::OpenSaveDataFileSystem(OutInterface<IFileSystem> out_interface,
                                       FileSys::SaveDataSpaceId space_id,
                                       FileSys::SaveDataAttribute attribute) {
    LOG_INFO(Service_FS, "called.");

    FileSys::VirtualDir dir{};
    R_TRY(save_data_controller->OpenSaveData(&dir, space_id, attribute));

    FileSys::StorageId id{};
    switch (space_id) {
    case FileSys::SaveDataSpaceId::User:
        id = FileSys::StorageId::NandUser;
        break;
    case FileSys::SaveDataSpaceId::SdSystem:
    case FileSys::SaveDataSpaceId::SdUser:
        id = FileSys::StorageId::SdCard;
        break;
    case FileSys::SaveDataSpaceId::System:
        id = FileSys::StorageId::NandSystem;
        break;
    case FileSys::SaveDataSpaceId::Temporary:
        // ok this is definitely wrong. ASSERT(false) here just kills the whole game the first
        // time it opens cache storage, and plenty of games do that (TOTK for one). there is
        // user-space scratch storage so it belongs on user nand. map it, do not crash.
        id = FileSys::StorageId::NandUser;
        break;
    case FileSys::SaveDataSpaceId::ProperSystem:
    case FileSys::SaveDataSpaceId::SafeMode:
        // same deal for these two. they are system-level spaces so they go on system nand.
        // way better than nuking the title over a save-space id we just did not list out.
        id = FileSys::StorageId::NandSystem;
        break;
    }

    *out_interface =
        std::make_shared<IFileSystem>(system, std::move(dir), SizeGetter::FromStorageId(fsc, id));

    R_SUCCEED();
}

Result FSP_SRV::OpenSaveDataFileSystemBySystemSaveDataId(OutInterface<IFileSystem> out_interface,
                                                         FileSys::SaveDataSpaceId space_id,
                                                         FileSys::SaveDataAttribute attribute) {
    LOG_INFO(Service_FS, "called, space_id={}, {}",
             space_id, attribute.DebugInfo());

    R_UNLESS(attribute.system_save_data_id != FileSys::InvalidSystemSaveDataId,
             FileSys::ResultInvalidArgument);

    if (attribute.program_id == 0) {
        attribute.program_id = program_id;
    }

    FileSys::VirtualDir dir{};
    R_TRY(save_data_controller->OpenSaveData(&dir, space_id, attribute));

    FileSys::StorageId id{};
    switch (space_id) {
    case FileSys::SaveDataSpaceId::User:
        id = FileSys::StorageId::NandUser;
        break;
    case FileSys::SaveDataSpaceId::SdSystem:
    case FileSys::SaveDataSpaceId::SdUser:
        id = FileSys::StorageId::SdCard;
        break;
    case FileSys::SaveDataSpaceId::System:
        id = FileSys::StorageId::NandSystem;
        break;
    case FileSys::SaveDataSpaceId::Temporary:
        // same broken switch as OpenSaveDataFileSystem above. do not ASSERT(false) and kill the
        // game over a save-space id, just map Temporary to user nand like it should be.
        id = FileSys::StorageId::NandUser;
        break;
    case FileSys::SaveDataSpaceId::ProperSystem:
    case FileSys::SaveDataSpaceId::SafeMode:
        // system spaces -> system nand. handled, not crashed.
        id = FileSys::StorageId::NandSystem;
        break;
    }

    *out_interface =
        std::make_shared<IFileSystem>(system, std::move(dir), SizeGetter::FromStorageId(fsc, id));

    R_SUCCEED();
}

Result FSP_SRV::OpenReadOnlySaveDataFileSystem(OutInterface<IFileSystem> out_interface,
                                               FileSys::SaveDataSpaceId space_id,
                                               FileSys::SaveDataAttribute attribute) {
    LOG_WARNING(Service_FS, "(STUBBED) called, delegating to 51 OpenSaveDataFilesystem");
    R_RETURN(OpenSaveDataFileSystem(out_interface, space_id, attribute));
}

Result FSP_SRV::OpenSaveDataInfoReaderBySaveDataSpaceId(
    OutInterface<ISaveDataInfoReader> out_interface, FileSys::SaveDataSpaceId space) {
    LOG_INFO(Service_FS, "called, space={}", space);

    *out_interface = std::make_shared<ISaveDataInfoReader>(system, save_data_controller, space);

    R_SUCCEED();
}

Result FSP_SRV::OpenSaveDataInfoReaderOnlyCacheStorage(
    OutInterface<ISaveDataInfoReader> out_interface) {
    LOG_WARNING(Service_FS, "(STUBBED) called");

    *out_interface = std::make_shared<ISaveDataInfoReader>(system, save_data_controller,
                                                           FileSys::SaveDataSpaceId::Temporary);

    R_SUCCEED();
}

Result FSP_SRV::FindSaveDataWithFilter(Out<s64> out_count,
                                       OutBuffer<BufferAttr_HipcMapAlias> out_buffer,
                                       FileSys::SaveDataSpaceId space_id,
                                       FileSys::SaveDataFilter filter) {
    LOG_WARNING(Service_FS, "(STUBBED) called");
    R_THROW(FileSys::ResultTargetNotFound);
}

Result FSP_SRV::WriteSaveDataFileSystemExtraData(InBuffer<BufferAttr_HipcMapAlias> buffer,
                                                 FileSys::SaveDataSpaceId space_id,
                                                 u64 save_data_id) {
    LOG_WARNING(Service_FS, "(STUBBED) called, space_id={}, save_data_id={:016X}", space_id,
                save_data_id);
    R_SUCCEED();
}

Result FSP_SRV::WriteSaveDataFileSystemExtraDataWithMaskBySaveDataAttribute(
    InBuffer<BufferAttr_HipcMapAlias> buffer, InBuffer<BufferAttr_HipcMapAlias> mask_buffer,
    FileSys::SaveDataSpaceId space_id, FileSys::SaveDataAttribute attribute) {
    LOG_WARNING(Service_FS,
                "(STUBBED) called, space_id={}, attribute.program_id={:016X}\n"
                "attribute.user_id={:016X}{:016X}, attribute.save_id={:016X}\n"
                "attribute.type={}, attribute.rank={}, attribute.index={}",
                space_id, attribute.program_id, attribute.user_id[1], attribute.user_id[0],
                attribute.system_save_data_id, attribute.type, attribute.rank, attribute.index);
    R_SUCCEED();
}

Result FSP_SRV::ReadSaveDataFileSystemExtraDataWithMaskBySaveDataAttribute(
    FileSys::SaveDataSpaceId space_id, FileSys::SaveDataAttribute attribute,
    InBuffer<BufferAttr_HipcMapAlias> mask_buffer, OutBuffer<BufferAttr_HipcMapAlias> out_buffer) {
    // Stub this to None for now, backend needs an impl to read/write the SaveDataExtraData
    // In an earlier version of the code, this was returned as an out argument, but this is not
    // correct
    [[maybe_unused]] constexpr auto flags = static_cast<u32>(FileSys::SaveDataFlags::None);

    LOG_WARNING(Service_FS,
                "(STUBBED) called, flags={}, space_id={}, attribute.program_id={:016X}\n"
                "attribute.user_id={:016X}{:016X}, attribute.save_id={:016X}\n"
                "attribute.type={}, attribute.rank={}, attribute.index={}",
                flags, space_id, attribute.program_id, attribute.user_id[1], attribute.user_id[0],
                attribute.system_save_data_id, attribute.type, attribute.rank, attribute.index);

    R_SUCCEED();
}

Result FSP_SRV::ReadSaveDataFileSystemExtraData(OutBuffer<BufferAttr_HipcMapAlias> out_buffer,
                                                u64 save_data_id) {
    // Stub, backend needs an impl to read/write the SaveDataExtraData
    LOG_WARNING(Service_FS, "(STUBBED) called, save_data_id={:016X}", save_data_id);
    std::memset(out_buffer.data(), 0, out_buffer.size());
    R_SUCCEED();
}

Result FSP_SRV::ReadSaveDataFileSystemExtraDataBySaveDataAttribute(
    OutBuffer<BufferAttr_HipcMapAlias> out_buffer, FileSys::SaveDataSpaceId space_id,
    FileSys::SaveDataAttribute attribute) {
    // Stub, backend needs an impl to read/write the SaveDataExtraData
    LOG_WARNING(Service_FS,
                "(STUBBED) called, space_id={}, attribute.program_id={:016X}\n"
                "attribute.user_id={:016X}{:016X}, attribute.save_id={:016X}\n"
                "attribute.type={}, attribute.rank={}, attribute.index={}",
                space_id, attribute.program_id, attribute.user_id[1], attribute.user_id[0],
                attribute.system_save_data_id, attribute.type, attribute.rank, attribute.index);
    std::memset(out_buffer.data(), 0, out_buffer.size());
    R_SUCCEED();
}

Result FSP_SRV::ReadSaveDataFileSystemExtraDataBySaveDataSpaceId(
    OutBuffer<BufferAttr_HipcMapAlias> out_buffer, FileSys::SaveDataSpaceId space_id,
    u64 save_data_id) {
    // Stub, backend needs an impl to read/write the SaveDataExtraData
    LOG_WARNING(Service_FS, "(STUBBED) called, space_id={}, save_data_id={:016X}", space_id,
                save_data_id);
    std::memset(out_buffer.data(), 0, out_buffer.size());
    R_SUCCEED();
}

Result FSP_SRV::OpenSaveDataTransferProhibiter(
    OutInterface<ISaveDataTransferProhibiter> out_prohibiter, u64 id) {
    LOG_WARNING(Service_FS, "(STUBBED) called, id={:016X}", id);
    *out_prohibiter = std::make_shared<ISaveDataTransferProhibiter>(system);
    R_SUCCEED();
}

Result FSP_SRV::OpenDataFileSystemByCurrentProcess(OutInterface<IFileSystem> out_interface) {
    LOG_DEBUG(Service_FS, "called");

    if (!romfs) {
        auto current_romfs = romfs_controller->OpenRomFSCurrentProcess();
        if (!current_romfs) {
            LOG_CRITICAL(Service_FS, "No file system interface available!");
            R_RETURN(ResultUnknown);
        }

        romfs = current_romfs;
    }

    auto extracted_romfs = FileSys::ExtractRomFS(romfs);
    if (!extracted_romfs) {
        LOG_CRITICAL(Service_FS, "Failed to extract RomFS for the current process!");
        R_RETURN(ResultUnknown);
    }

    *out_interface = std::make_shared<IFileSystem>(
        system, extracted_romfs, SizeGetter::FromStorageId(fsc, FileSys::StorageId::NandUser));

    R_SUCCEED();
}

Result FSP_SRV::OpenDataStorageByCurrentProcess(OutInterface<IStorage> out_interface) {
    LOG_DEBUG(Service_FS, "called");

    if (!romfs) {
        auto current_romfs = romfs_controller->OpenRomFSCurrentProcess();
        if (!current_romfs) {
            // TODO (bunnei): Find the right error code to use here
            LOG_CRITICAL(Service_FS, "No file system interface available!");
            R_RETURN(ResultUnknown);
        }

        romfs = current_romfs;
    }

    *out_interface = std::make_shared<IStorage>(system, romfs);

    R_SUCCEED();
}

Result FSP_SRV::OpenDataStorageByDataId(OutInterface<IStorage> out_interface,
                                        FileSys::StorageId storage_id, u32 unknown, u64 title_id) {
    LOG_DEBUG(Service_FS, "called with storage_id={:02X}, unknown={:08X}, title_id={:016X}",
              storage_id, unknown, title_id);

    auto data = romfs_controller->OpenRomFS(title_id, storage_id, FileSys::ContentRecordType::Data);

    // Experiment: Nintendo Switch Sports Resort reads Mii head models from Switch 2 system data
    // 0400000000000849. When it is not installed, serve the Switch 1 MiiModel (0100000000000802)
    // from the installed firmware. EDEN_NX2_MII_MODEL_FALLBACK=0 disables this.
    constexpr u64 Nx2MiiModelId = 0x0400000000000849;
    constexpr u64 MiiModelId = 0x0100000000000802;
    static const bool mii_model_fallback = [] {
        const char* setting = std::getenv("EDEN_NX2_MII_MODEL_FALLBACK");
        return setting == nullptr || setting[0] != '0';
    }();
    if (!data && title_id == Nx2MiiModelId && mii_model_fallback) {
        data = romfs_controller->OpenRomFS(MiiModelId, FileSys::StorageId::NandSystem,
                                           FileSys::ContentRecordType::Data);
        if (data) {
            LOG_WARNING(Service_FS, "Serving MiiModel {:016X} for Switch 2 system data {:016X}",
                        MiiModelId, title_id);
            *out_interface =
                std::make_shared<IStorage>(
                    system, InstrumentMiiModel(AddOunceTextureAliases(std::move(data))));
            R_SUCCEED();
        }
        LOG_WARNING(Service_FS, "MiiModel {:016X} is not installed, no fallback for {:016X}",
                    MiiModelId, title_id);
    }

    if (!data) {
        const auto archive = FileSys::SystemArchive::SynthesizeSystemArchive(title_id);

        if (archive != nullptr) {
            *out_interface = std::make_shared<IStorage>(system, archive);
            R_SUCCEED();
        }

        // TODO(DarkLordZach): Find the right error code to use here
        LOG_ERROR(Service_FS,
                  "Could not open data storage with title_id={:016X}, storage_id={:02X}", title_id,
                  storage_id);
        R_RETURN(ResultUnknown);
    }

    const FileSys::PatchManager pm{title_id, fsc, content_provider};

    auto base =
        romfs_controller->OpenBaseNca(title_id, storage_id, FileSys::ContentRecordType::Data);
    auto storage = std::make_shared<IStorage>(
        system, pm.PatchRomFS(base.get(), std::move(data), FileSys::ContentRecordType::Data));

    *out_interface = std::move(storage);
    R_SUCCEED();
}

Result FSP_SRV::OpenPatchDataStorageByCurrentProcess(OutInterface<IStorage> out_interface,
                                                     FileSys::StorageId storage_id, u64 title_id) {
    LOG_WARNING(Service_FS, "(STUBBED) called with storage_id={:02X}, title_id={:016X}", storage_id,
                title_id);

    R_RETURN(FileSys::ResultTargetNotFound);
}

Result FSP_SRV::OpenDataStorageWithProgramIndex(OutInterface<IStorage> out_interface,
                                                u8 program_index) {
    LOG_DEBUG(Service_FS, "called, program_index={}", program_index);

    auto patched_romfs = romfs_controller->OpenPatchedRomFSWithProgramIndex(
        program_id, program_index, FileSys::ContentRecordType::Program);

    if (!patched_romfs) {
        // TODO: Find the right error code to use here
        LOG_ERROR(Service_FS, "Could not open storage with program_index={}", program_index);
        R_RETURN(ResultUnknown);
    }

    *out_interface = std::make_shared<IStorage>(system, std::move(patched_romfs));

    R_SUCCEED();
}

Result FSP_SRV::IsSdCardAccessible(Out<bool> out_is_accessible) {
    LOG_DEBUG(Service_FS, "(STUBBED) called");

    *out_is_accessible = true;

    R_SUCCEED();
}

Result FSP_SRV::DisableAutoSaveDataCreation() {
    LOG_DEBUG(Service_FS, "called");

    save_data_controller->SetAutoCreate(false);

    R_SUCCEED();
}

Result FSP_SRV::SetGlobalAccessLogMode(AccessLogMode access_log_mode_) {
    LOG_DEBUG(Service_FS, "called, access_log_mode={}", access_log_mode_);

    access_log_mode = access_log_mode_;

    R_SUCCEED();
}

Result FSP_SRV::GetGlobalAccessLogMode(Out<AccessLogMode> out_access_log_mode) {
    LOG_DEBUG(Service_FS, "called");

    *out_access_log_mode = access_log_mode;

    R_SUCCEED();
}

Result FSP_SRV::OutputAccessLogToSdCard(InBuffer<BufferAttr_HipcMapAlias> log_message_buffer) {
    LOG_DEBUG(Service_FS, "called");

    auto log = Common::StringFromFixedZeroTerminatedBuffer(
        reinterpret_cast<const char*>(log_message_buffer.data()), log_message_buffer.size());
    reporter.SaveFSAccessLog(log);

    R_SUCCEED();
}

Result FSP_SRV::GetProgramIndexForAccessLog(Out<AccessLogVersion> out_access_log_version,
                                            Out<u32> out_access_log_program_index) {
    LOG_DEBUG(Service_FS, "(STUBBED) called");

    *out_access_log_version = AccessLogVersion::Latest;
    *out_access_log_program_index = access_log_program_index;

    R_SUCCEED();
}

Result FSP_SRV::FlushAccessLogOnSdCard() {
    LOG_DEBUG(Service_FS, "(STUBBED) called");

    R_SUCCEED();
}

Result FSP_SRV::ExtendSaveDataFileSystem(FileSys::SaveDataSpaceId space_id, u64 save_data_id,
                                         s64 available_size, s64 journal_size) {
    // We don't have an index of save data ids, so we can't implement this.
    LOG_WARNING(Service_FS,
                "(STUBBED) called, space_id={}, save_data_id={:016X}, available_size={:#x}, "
                "journal_size={:#x}",
                space_id, save_data_id, available_size, journal_size);
    R_SUCCEED();
}

Result FSP_SRV::GetCacheStorageSize(s32 index, Out<s64> out_data_size, Out<s64> out_journal_size) {
    LOG_WARNING(Service_FS, "(STUBBED) called with index={}", index);

    *out_data_size = 0;
    *out_journal_size = 0;

    R_SUCCEED();
}

Result FSP_SRV::OpenMultiCommitManager(OutInterface<IMultiCommitManager> out_interface) {
    LOG_DEBUG(Service_FS, "called");

    *out_interface = std::make_shared<IMultiCommitManager>(system);

    R_SUCCEED();
}

} // namespace Service::FileSystem
