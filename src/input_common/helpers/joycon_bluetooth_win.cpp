// SPDX-FileCopyrightText: Copyright 2026 Eden Emulator Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "input_common/helpers/joycon_bluetooth_win.h"

#ifdef _WIN32

#include <cwchar>
#include <set>
#include <string>

// clang-format off
#include <windows.h>
#include <bluetoothapis.h>
// clang-format on

#include <fmt/format.h>

#include "common/logging.h"
#include "common/string_util.h"

namespace InputCommon::Joycon {
namespace {

bool IsSwitchController(const BLUETOOTH_DEVICE_INFO& info) {
    return std::wcsncmp(info.szName, L"Joy-Con", 7) == 0 ||
           std::wcscmp(info.szName, L"Pro Controller") == 0;
}

// Inquiry also returns remembered devices that are out of range, so only trust recent sightings.
bool SeenRecently(const BLUETOOTH_DEVICE_INFO& info) {
    FILETIME seen_ft{};
    FILETIME now_ft{};
    SYSTEMTIME now{};
    GetSystemTime(&now);
    if (!SystemTimeToFileTime(&info.stLastSeen, &seen_ft) || !SystemTimeToFileTime(&now, &now_ft)) {
        return false;
    }
    ULARGE_INTEGER seen{}, current{};
    seen.LowPart = seen_ft.dwLowDateTime;
    seen.HighPart = seen_ft.dwHighDateTime;
    current.LowPart = now_ft.dwLowDateTime;
    current.HighPart = now_ft.dwHighDateTime;
    constexpr ULONGLONG Window = 30ULL * 10'000'000ULL; // 30 s in 100 ns units
    const ULONGLONG diff = current.QuadPart > seen.QuadPart ? current.QuadPart - seen.QuadPart
                                                            : seen.QuadPart - current.QuadPart;
    return diff < Window;
}

BOOL CALLBACK AcceptAuthentication(LPVOID, PBLUETOOTH_AUTHENTICATION_CALLBACK_PARAMS params) {
    BLUETOOTH_AUTHENTICATE_RESPONSE response{};
    response.bthAddressRemote = params->deviceInfo.Address;
    response.authMethod = params->authenticationMethod;
    response.negativeResponse = FALSE;
    switch (params->authenticationMethod) {
    case BLUETOOTH_AUTHENTICATION_METHOD_NUMERIC_COMPARISON:
    case BLUETOOTH_AUTHENTICATION_METHOD_PASSKEY_NOTIFICATION:
        response.numericCompInfo.NumericValue = params->Numeric_Value;
        break;
    case BLUETOOTH_AUTHENTICATION_METHOD_LEGACY:
        response.pinInfo.pinLength = 4;
        response.pinInfo.pin[0] = response.pinInfo.pin[1] = response.pinInfo.pin[2] =
            response.pinInfo.pin[3] = '0';
        break;
    default:
        break;
    }
    const DWORD result = BluetoothSendAuthenticationResponseEx(nullptr, &response);
    LOG_INFO(Input, "Joycon pairing: authentication method {} answered, result={}",
             static_cast<int>(params->authenticationMethod), result);
    return TRUE;
}

void Pair(HANDLE radio, BLUETOOTH_DEVICE_INFO& info) {
    if (info.fRemembered) {
        // Windows keeps a pairing the controller no longer accepts; start over like a new device.
        const DWORD removed = BluetoothRemoveDevice(&info.Address);
        LOG_INFO(Input, "Joycon pairing: removed stale pairing, result={}", removed);
        info.fRemembered = FALSE;
        info.fAuthenticated = FALSE;
    }

    HBLUETOOTH_AUTHENTICATION_REGISTRATION registration = nullptr;
    BluetoothRegisterForAuthenticationEx(&info, &registration, AcceptAuthentication, nullptr);
    const DWORD authenticated = BluetoothAuthenticateDeviceEx(nullptr, radio, &info, nullptr,
                                                              MITMProtectionNotRequiredBonding);
    if (registration != nullptr) {
        BluetoothUnregisterAuthentication(registration);
    }

    // HumanInterfaceDeviceServiceClass_UUID
    GUID hid_service{0x00001124, 0x0000, 0x1000, {0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB}};
    const DWORD enabled =
        BluetoothSetServiceState(radio, &info, &hid_service, BLUETOOTH_SERVICE_ENABLE);
    // Without a stored link key Windows turns the controller away when it reconnects by itself.
    BluetoothGetDeviceInfo(radio, &info);
    LOG_INFO(Input, "Joycon pairing: paired '{}', authenticate={} hid_service={} bonded={}",
             Common::UTF16ToUTF8(info.szName), authenticated, enabled, info.fAuthenticated != 0);
}

std::string DescribeAddress(const BLUETOOTH_ADDRESS& address) {
    return fmt::format("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", address.rgBytes[5],
                       address.rgBytes[4], address.rgBytes[3], address.rgBytes[2],
                       address.rgBytes[1], address.rgBytes[0]);
}

} // namespace

void TracePairedControllers() {
    static std::string last_state;

    BLUETOOTH_FIND_RADIO_PARAMS radio_params{sizeof(radio_params)};
    HANDLE radio = nullptr;
    HBLUETOOTH_RADIO_FIND radio_find = BluetoothFindFirstRadio(&radio_params, &radio);
    if (radio_find == nullptr) {
        return;
    }

    std::set<std::string> controllers;
    do {
        BLUETOOTH_DEVICE_SEARCH_PARAMS search{};
        search.dwSize = sizeof(search);
        search.fReturnAuthenticated = TRUE;
        search.fReturnRemembered = TRUE;
        search.fReturnConnected = TRUE;
        search.hRadio = radio;

        BLUETOOTH_DEVICE_INFO info{};
        info.dwSize = sizeof(info);
        HBLUETOOTH_DEVICE_FIND device_find = BluetoothFindFirstDevice(&search, &info);
        if (device_find != nullptr) {
            do {
                if (IsSwitchController(info)) {
                    controllers.insert(fmt::format(
                        " [{} {} remembered={} authenticated={} connected={}]",
                        Common::UTF16ToUTF8(info.szName), DescribeAddress(info.Address),
                        info.fRemembered != 0, info.fAuthenticated != 0, info.fConnected != 0));
                }
                info = {};
                info.dwSize = sizeof(info);
            } while (BluetoothFindNextDevice(device_find, &info));
            BluetoothFindDeviceClose(device_find);
        }
        CloseHandle(radio);
    } while (BluetoothFindNextRadio(radio_find, &radio));
    BluetoothFindRadioClose(radio_find);

    std::string state;
    for (const auto& controller : controllers) {
        state += controller;
    }
    if (state != last_state) {
        LOG_INFO(Input, "Joycon bluetooth state:{}", state.empty() ? " none" : state);
        last_state = state;
    }
}

void PairDiscoverableControllers() {
    BLUETOOTH_FIND_RADIO_PARAMS radio_params{sizeof(radio_params)};
    HANDLE radio = nullptr;
    HBLUETOOTH_RADIO_FIND radio_find = BluetoothFindFirstRadio(&radio_params, &radio);
    if (radio_find == nullptr) {
        return;
    }

    do {
        BLUETOOTH_DEVICE_SEARCH_PARAMS search{};
        search.dwSize = sizeof(search);
        search.fReturnAuthenticated = TRUE;
        search.fReturnRemembered = TRUE;
        search.fReturnUnknown = TRUE;
        search.fReturnConnected = FALSE;
        search.fIssueInquiry = TRUE;
        search.cTimeoutMultiplier = 2; // 2.56 s
        search.hRadio = radio;

        BLUETOOTH_DEVICE_INFO info{};
        info.dwSize = sizeof(info);
        HBLUETOOTH_DEVICE_FIND device_find = BluetoothFindFirstDevice(&search, &info);
        if (device_find != nullptr) {
            do {
                if (IsSwitchController(info) && !info.fConnected && SeenRecently(info)) {
                    Pair(radio, info);
                }
                info = {};
                info.dwSize = sizeof(info);
            } while (BluetoothFindNextDevice(device_find, &info));
            BluetoothFindDeviceClose(device_find);
        }
        CloseHandle(radio);
    } while (BluetoothFindNextRadio(radio_find, &radio));
    BluetoothFindRadioClose(radio_find);
}

} // namespace InputCommon::Joycon

#else

namespace InputCommon::Joycon {
void PairDiscoverableControllers() {}
void TracePairedControllers() {}
} // namespace InputCommon::Joycon

#endif
