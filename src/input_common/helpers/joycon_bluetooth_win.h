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

/// Logs the state Windows reports for paired controllers whenever it changes
void TracePairedControllers();

} // namespace InputCommon::Joycon
