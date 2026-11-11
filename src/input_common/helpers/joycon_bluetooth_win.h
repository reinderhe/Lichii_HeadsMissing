// SPDX-FileCopyrightText: Copyright 2026 Eden Emulator Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <span>
#include <string>

#include "common/common_types.h"

namespace InputCommon::Joycon {

/**
 * Looks for Joy-Cons and Pro Controllers in pairing mode and pairs them with Windows, replacing a
 * stale pairing when Windows still remembers the controller but can't reconnect it.
 * Blocks for a few seconds while it scans. Does nothing on other platforms.
 */
void PairDiscoverableControllers();

/// Logs the state Windows reports for paired controllers whenever it changes
void TracePairedControllers();

/**
 * Sends an output report through the HID control channel (HidD_SetOutputReport). Opens the device
 * at path into output_handle on first use. Returns false on failure or on other platforms.
 */
bool SetOutputReport(const std::string& path, void*& output_handle, std::span<const u8> data);

/// Closes a handle opened by SetOutputReport
void CloseOutputHandle(void*& output_handle);

} // namespace InputCommon::Joycon
