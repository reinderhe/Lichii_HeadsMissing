// SPDX-FileCopyrightText: Copyright 2026 Eden Emulator Project
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace InputCommon::Joycon {

/**
 * Looks for Joy-Cons and Pro Controllers in pairing mode and pairs them with Windows, replacing a
 * stale pairing when Windows still remembers the controller but can't reconnect it.
 * Blocks for a few seconds while it scans. Does nothing on other platforms.
 */
void PairDiscoverableControllers();

/**
 * Checks the controllers Windows has paired without scanning. With trace set, logs their state
 * whenever it changes; with reenable_hid set, reinstalls the HID driver once per session for each
 * paired controller that isn't connected.
 */
void CheckPairedControllers(bool trace, bool reenable_hid);

} // namespace InputCommon::Joycon
