// SPDX-FileCopyrightText: Copyright 2020 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include <array>
#include <memory>
#include <QWidget>

class QColor;
class QCheckBox;
class QPushButton;

namespace Ui {
class ConfigureInputAdvanced;
}

namespace Core::HID {
class HIDCore;
} // namespace Core::HID

class ConfigureInputAdvanced : public QWidget {
    Q_OBJECT

public:
    explicit ConfigureInputAdvanced(Core::HID::HIDCore& hid_core_, QWidget* parent = nullptr);
    ~ConfigureInputAdvanced() override;

    void ApplyConfiguration();

signals:
    void CallDebugControllerDialog();
    void CallMouseConfigDialog();
    void CallTouchscreenConfigDialog();
    void CallMotionTouchConfigDialog();
    void CallRingControllerDialog();
    void CallCameraDialog();

private:
    void changeEvent(QEvent* event) override;
    void RetranslateUI();
    void UpdateUIEnabled();

    void OnControllerButtonClick(std::size_t player_idx, std::size_t button_idx);

    void LoadConfiguration();

    void AddJoyconOptions();

    std::unique_ptr<Ui::ConfigureInputAdvanced> ui;

    QCheckBox* joycon_auto_pair = nullptr;
    QCheckBox* joycon_power_off_on_exit = nullptr;
    QCheckBox* motion_gravity_correction = nullptr;
    QCheckBox* motion_fast_drift_correction = nullptr;

    std::array<std::array<QColor, 4>, 8> controllers_colors;
    std::array<std::array<QPushButton*, 4>, 8> controllers_color_buttons;

    Core::HID::HIDCore& hid_core;
};
