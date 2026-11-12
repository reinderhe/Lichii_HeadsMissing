// SPDX-FileCopyrightText: Copyright 2026 Eden Emulator Project
// SPDX-License-Identifier: GPL-3.0-or-later

// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "input_common/helpers/joycon_protocol/poller.h"

namespace InputCommon::Joycon {

JoyconPoller::JoyconPoller(ControllerType device_type_, JoyStickCalibration left_stick_calibration_,
                           JoyStickCalibration right_stick_calibration_,
                           MotionCalibration motion_calibration_)
    : device_type{device_type_}, left_stick_calibration{left_stick_calibration_},
      right_stick_calibration{right_stick_calibration_}, motion_calibration{motion_calibration_} {}

void JoyconPoller::SetCallbacks(const JoyconCallbacks& callbacks_) {
    callbacks = std::move(callbacks_);
}

template <typename Rotate>
void JoyconPoller::SendMotion(const InputReportActive& input, const MotionStatus& motion_status,
                              int id, int second_id, Rotate rotate) {
    static const bool split = [] {
        const char* setting = std::getenv("EDEN_NX2_MOTION_SAMPLES");
        return setting != nullptr && setting[0] == '1';
    }();
    const std::size_t samples = std::max<std::size_t>(imu_sample_count, 1);
    if (!split || samples == 1) {
        // Average every sample of the report: one update covers all of them, so using only the
        // first misses part of fast movements.
        auto motion = GetMotionInput(input, motion_status, 0, samples);
        rotate(motion);
        callbacks.on_motion_data(id, motion);
        if (second_id >= 0) {
            callbacks.on_motion_data(second_id, motion);
        }
        return;
    }
    // Oldest sample first, each covering its share of the report interval
    for (std::size_t i = 0; i < samples; ++i) {
        auto motion = GetMotionInput(input, motion_status, i, 1);
        motion.delta_timestamp = motion_status.delta_time / samples;
        rotate(motion);
        callbacks.on_motion_data(id, motion);
        if (second_id >= 0) {
            callbacks.on_motion_data(second_id, motion);
        }
    }
}

void JoyconPoller::ReadActiveMode(std::span<u8> buffer, const MotionStatus& motion_status,
                                  const RingStatus& ring_status) {
    InputReportActive data{};
    memcpy(&data, buffer.data(), sizeof(InputReportActive));

    // A full report carries three IMU samples. The Ring-Con overwrites the third one, and
    // EDEN_JOYCON_IMU_SAMPLES=1 keeps the old behaviour of using only the first.
    static const bool first_sample_only = [] {
        const char* setting = std::getenv("EDEN_JOYCON_IMU_SAMPLES");
        return setting != nullptr && setting[0] == '1';
    }();
    constexpr std::size_t ImuOffset = 13;
    const std::size_t available = buffer.size() >= ImuOffset ? (buffer.size() - ImuOffset) / 12 : 0;
    imu_sample_count =
        first_sample_only ? 1 : std::min<std::size_t>(ring_status.is_enabled ? 2 : 3, available);
    imu_sample_count = std::max<std::size_t>(imu_sample_count, 1);
    if (available >= imu_sample_count) {
        memcpy(imu_samples.data(), buffer.data() + ImuOffset, imu_sample_count * 12);
    } else {
        memcpy(imu_samples.data(), data.motion_input.data(), 12);
        imu_sample_count = 1;
    }

    switch (device_type) {
    case ControllerType::Left:
        UpdateActiveLeftPadInput(data, motion_status);
        break;
    case ControllerType::Right:
        UpdateActiveRightPadInput(data, motion_status);
        break;
    case ControllerType::Pro:
        UpdateActiveProPadInput(data, motion_status);
        break;
    default:
        break;
    }

    if (ring_status.is_enabled) {
        UpdateRing(data.ring_input, ring_status);
    }

    callbacks.on_battery_data(data.battery_status);
}

void JoyconPoller::ReadPassiveMode(std::span<u8> buffer) {
    InputReportPassive data{};
    memcpy(&data, buffer.data(), sizeof(InputReportPassive));

    switch (device_type) {
    case ControllerType::Left:
        UpdatePassiveLeftPadInput(data);
        break;
    case ControllerType::Right:
        UpdatePassiveRightPadInput(data);
        break;
    case ControllerType::Pro:
        UpdatePassiveProPadInput(data);
        break;
    default:
        break;
    }
}

void JoyconPoller::ReadNfcIRMode(std::span<u8> buffer, const MotionStatus& motion_status) {
    // This mode is compatible with the active mode
    ReadActiveMode(buffer, motion_status, {});
}

void JoyconPoller::UpdateColor(const Color& color) {
    callbacks.on_color_data(color);
}

void JoyconPoller::UpdateAmiibo(const Joycon::TagInfo& tag_info) {
    callbacks.on_amiibo_data(tag_info);
}

void JoyconPoller::UpdateCamera(const std::vector<u8>& camera_data, IrsResolution format) {
    callbacks.on_camera_data(camera_data, format);
}

void JoyconPoller::UpdateRing(s16 value, const RingStatus& ring_status) {
    float normalized_value = static_cast<float>(value - ring_status.default_value);
    if (normalized_value > 0) {
        normalized_value = normalized_value /
                           static_cast<float>(ring_status.max_value - ring_status.default_value);
    }
    if (normalized_value < 0) {
        normalized_value = normalized_value /
                           static_cast<float>(ring_status.default_value - ring_status.min_value);
    }
    callbacks.on_ring_data(normalized_value);
}

void JoyconPoller::UpdateActiveLeftPadInput(const InputReportActive& input,
                                            const MotionStatus& motion_status) {
    static constexpr std::array<Joycon::PadButton, 11> left_buttons{
        Joycon::PadButton::Down,    Joycon::PadButton::Up,     Joycon::PadButton::Right,
        Joycon::PadButton::Left,    Joycon::PadButton::LeftSL, Joycon::PadButton::LeftSR,
        Joycon::PadButton::L,       Joycon::PadButton::ZL,     Joycon::PadButton::Minus,
        Joycon::PadButton::Capture, Joycon::PadButton::StickL,
    };

    const u32 raw_button =
        static_cast<u32>(input.button_input[2] | ((input.button_input[1] & 0b00101001) << 16));
    for (std::size_t i = 0; i < left_buttons.size(); ++i) {
        const bool button_status = (raw_button & static_cast<u32>(left_buttons[i])) != 0;
        const int button = static_cast<int>(left_buttons[i]);
        callbacks.on_button_data(button, button_status);
    }

    const u16 raw_left_axis_x =
        static_cast<u16>(input.left_stick_state[0] | ((input.left_stick_state[1] & 0xf) << 8));
    const u16 raw_left_axis_y =
        static_cast<u16>((input.left_stick_state[1] >> 4) | (input.left_stick_state[2] << 4));
    const f32 left_axis_x = GetAxisValue(raw_left_axis_x, left_stick_calibration.x);
    const f32 left_axis_y = GetAxisValue(raw_left_axis_y, left_stick_calibration.y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickX), left_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickY), left_axis_y);

    if (motion_status.is_enabled) {
        // Rotate motion axis to the correct direction
        SendMotion(input, motion_status, static_cast<int>(PadMotion::LeftMotion), -1,
                   [](MotionData& m) {
                       m.accel_y = -m.accel_y;
                       m.accel_z = -m.accel_z;
                       m.gyro_x = -m.gyro_x;
                   });
    }
}

void JoyconPoller::UpdateActiveRightPadInput(const InputReportActive& input,
                                             const MotionStatus& motion_status) {
    static constexpr std::array<Joycon::PadButton, 11> right_buttons{
        Joycon::PadButton::Y,    Joycon::PadButton::X,       Joycon::PadButton::B,
        Joycon::PadButton::A,    Joycon::PadButton::RightSL, Joycon::PadButton::RightSR,
        Joycon::PadButton::R,    Joycon::PadButton::ZR,      Joycon::PadButton::Plus,
        Joycon::PadButton::Home, Joycon::PadButton::StickR,
    };

    const u32 raw_button =
        static_cast<u32>((input.button_input[0] << 8) | (input.button_input[1] << 16));
    for (std::size_t i = 0; i < right_buttons.size(); ++i) {
        const bool button_status = (raw_button & static_cast<u32>(right_buttons[i])) != 0;
        const int button = static_cast<int>(right_buttons[i]);
        callbacks.on_button_data(button, button_status);
    }

    const u16 raw_right_axis_x =
        static_cast<u16>(input.right_stick_state[0] | ((input.right_stick_state[1] & 0xf) << 8));
    const u16 raw_right_axis_y =
        static_cast<u16>((input.right_stick_state[1] >> 4) | (input.right_stick_state[2] << 4));
    const f32 right_axis_x = GetAxisValue(raw_right_axis_x, right_stick_calibration.x);
    const f32 right_axis_y = GetAxisValue(raw_right_axis_y, right_stick_calibration.y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickX), right_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickY), right_axis_y);

    if (motion_status.is_enabled) {
        // Rotate motion axis to the correct direction
        SendMotion(input, motion_status, static_cast<int>(PadMotion::RightMotion), -1,
                   [](MotionData& m) {
                       m.accel_x = -m.accel_x;
                       m.accel_y = -m.accel_y;
                       m.gyro_z = -m.gyro_z;
                   });
    }
}

void JoyconPoller::UpdateActiveProPadInput(const InputReportActive& input,
                                           const MotionStatus& motion_status) {
    static constexpr std::array<Joycon::PadButton, 18> pro_buttons{
        Joycon::PadButton::Down,  Joycon::PadButton::Up,      Joycon::PadButton::Right,
        Joycon::PadButton::Left,  Joycon::PadButton::L,       Joycon::PadButton::ZL,
        Joycon::PadButton::Minus, Joycon::PadButton::Capture, Joycon::PadButton::Y,
        Joycon::PadButton::X,     Joycon::PadButton::B,       Joycon::PadButton::A,
        Joycon::PadButton::R,     Joycon::PadButton::ZR,      Joycon::PadButton::Plus,
        Joycon::PadButton::Home,  Joycon::PadButton::StickL,  Joycon::PadButton::StickR,
    };

    const u32 raw_button = static_cast<u32>(input.button_input[2] | (input.button_input[0] << 8) |
                                            (input.button_input[1] << 16));
    for (std::size_t i = 0; i < pro_buttons.size(); ++i) {
        const bool button_status = (raw_button & static_cast<u32>(pro_buttons[i])) != 0;
        const int button = static_cast<int>(pro_buttons[i]);
        callbacks.on_button_data(button, button_status);
    }

    const u16 raw_left_axis_x =
        static_cast<u16>(input.left_stick_state[0] | ((input.left_stick_state[1] & 0xf) << 8));
    const u16 raw_left_axis_y =
        static_cast<u16>((input.left_stick_state[1] >> 4) | (input.left_stick_state[2] << 4));
    const u16 raw_right_axis_x =
        static_cast<u16>(input.right_stick_state[0] | ((input.right_stick_state[1] & 0xf) << 8));
    const u16 raw_right_axis_y =
        static_cast<u16>((input.right_stick_state[1] >> 4) | (input.right_stick_state[2] << 4));

    const f32 left_axis_x = GetAxisValue(raw_left_axis_x, left_stick_calibration.x);
    const f32 left_axis_y = GetAxisValue(raw_left_axis_y, left_stick_calibration.y);
    const f32 right_axis_x = GetAxisValue(raw_right_axis_x, right_stick_calibration.x);
    const f32 right_axis_y = GetAxisValue(raw_right_axis_y, right_stick_calibration.y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickX), left_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickY), left_axis_y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickX), right_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickY), right_axis_y);

    if (motion_status.is_enabled) {
        SendMotion(input, motion_status, static_cast<int>(PadMotion::LeftMotion),
                   static_cast<int>(PadMotion::RightMotion), [](MotionData& m) {
                       m.gyro_x = -m.gyro_x;
                       m.accel_y = -m.accel_y;
                       m.accel_z = -m.accel_z;
                   });
    }
}

void JoyconPoller::UpdatePassiveLeftPadInput(const InputReportPassive& input) {
    static constexpr std::array<PassivePadButton, 11> left_buttons{
        PassivePadButton::Down_A,  PassivePadButton::Right_X, PassivePadButton::Left_B,
        PassivePadButton::Up_Y,    PassivePadButton::SL,      PassivePadButton::SR,
        PassivePadButton::L_R,     PassivePadButton::ZL_ZR,   PassivePadButton::Minus,
        PassivePadButton::Capture, PassivePadButton::StickL,
    };

    for (auto left_button : left_buttons) {
        const bool button_status = (input.button_input & static_cast<u32>(left_button)) != 0;
        const int button = static_cast<int>(left_button);
        callbacks.on_button_data(button, button_status);
    }

    const auto [left_axis_x, left_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state));
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickX), left_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickY), left_axis_y);
}

void JoyconPoller::UpdatePassiveRightPadInput(const InputReportPassive& input) {
    static constexpr std::array<PassivePadButton, 11> right_buttons{
        PassivePadButton::Down_A, PassivePadButton::Right_X, PassivePadButton::Left_B,
        PassivePadButton::Up_Y,   PassivePadButton::SL,      PassivePadButton::SR,
        PassivePadButton::L_R,    PassivePadButton::ZL_ZR,   PassivePadButton::Plus,
        PassivePadButton::Home,   PassivePadButton::StickR,
    };

    for (auto right_button : right_buttons) {
        const bool button_status = (input.button_input & static_cast<u32>(right_button)) != 0;
        const int button = static_cast<int>(right_button);
        callbacks.on_button_data(button, button_status);
    }

    const auto [right_axis_x, right_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state));
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickX), right_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickY), right_axis_y);
}

void JoyconPoller::UpdatePassiveProPadInput(const InputReportPassive& input) {
    static constexpr std::array<PassivePadButton, 14> pro_buttons{
        PassivePadButton::Down_A, PassivePadButton::Right_X, PassivePadButton::Left_B,
        PassivePadButton::Up_Y,   PassivePadButton::SL,      PassivePadButton::SR,
        PassivePadButton::L_R,    PassivePadButton::ZL_ZR,   PassivePadButton::Minus,
        PassivePadButton::Plus,   PassivePadButton::Capture, PassivePadButton::Home,
        PassivePadButton::StickL, PassivePadButton::StickR,
    };

    for (auto pro_button : pro_buttons) {
        const bool button_status = (input.button_input & static_cast<u32>(pro_button)) != 0;
        const int button = static_cast<int>(pro_button);
        callbacks.on_button_data(button, button_status);
    }

    const auto [left_axis_x, left_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state & 0xf));
    const auto [right_axis_x, right_axis_y] =
        GetPassiveAxisValue(static_cast<PassivePadStick>(input.stick_state >> 4));
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickX), left_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::LeftStickY), left_axis_y);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickX), right_axis_x);
    callbacks.on_stick_data(static_cast<int>(PadAxes::RightStickY), right_axis_y);
}

f32 JoyconPoller::GetAxisValue(u16 raw_value, Joycon::JoyStickAxisCalibration calibration) const {
    const f32 value = static_cast<f32>(raw_value - calibration.center);
    if (value > 0.0f) {
        return value / calibration.max;
    }
    return value / calibration.min;
}

std::pair<f32, f32> JoyconPoller::GetPassiveAxisValue(PassivePadStick raw_value) const {
    switch (raw_value) {
    case PassivePadStick::Right:
        return {1.0f, 0.0f};
    case PassivePadStick::RightDown:
        return {1.0f, -1.0f};
    case PassivePadStick::Down:
        return {0.0f, -1.0f};
    case PassivePadStick::DownLeft:
        return {-1.0f, -1.0f};
    case PassivePadStick::Left:
        return {-1.0f, 0.0f};
    case PassivePadStick::LeftUp:
        return {-1.0f, 1.0f};
    case PassivePadStick::Up:
        return {0.0f, 1.0f};
    case PassivePadStick::UpRight:
        return {1.0f, 1.0f};
    case PassivePadStick::Neutral:
    default:
        return {0.0f, 0.0f};
    }
}

f32 JoyconPoller::GetAccelerometerValue(s16 raw, const MotionSensorCalibration& cal,
                                        AccelerometerSensitivity sensitivity) const {
    // Experiment: EDEN_JOYCON_ACCEL_OFFSET=1 subtracts the factory zero offset like the gyroscope
    // does, in case Joy-Cons lying flat read a few degrees of tilt because of it.
    static const bool subtract_offset = [] {
        const char* setting = std::getenv("EDEN_JOYCON_ACCEL_OFFSET");
        return setting != nullptr && setting[0] == '1';
    }();
    const f32 centered = subtract_offset ? static_cast<f32>(raw - cal.offset) : raw;
    const f32 value = centered * (1.0f / (cal.scale - cal.offset)) * 4;
    switch (sensitivity) {
    case Joycon::AccelerometerSensitivity::G2:
        return value / 4.0f;
    case Joycon::AccelerometerSensitivity::G4:
        return value / 2.0f;
    case Joycon::AccelerometerSensitivity::G8:
        return value;
    case Joycon::AccelerometerSensitivity::G16:
        return value * 2.0f;
    }
    return value;
}

f32 JoyconPoller::GetGyroValue(s16 raw, const MotionSensorCalibration& cal,
                               GyroSensitivity sensitivity) const {
    const f32 value = (raw - cal.offset) * (936.0f / (cal.scale - cal.offset)) / 360.0f;
    switch (sensitivity) {
    case Joycon::GyroSensitivity::DPS250:
        return value / 8.0f;
    case Joycon::GyroSensitivity::DPS500:
        return value / 4.0f;
    case Joycon::GyroSensitivity::DPS1000:
        return value / 2.0f;
    case Joycon::GyroSensitivity::DPS2000:
        return value;
    }
    return value;
}

s16 JoyconPoller::GetRawIMUValues(std::size_t sensor, size_t axis,
                                  const InputReportActive& input) const {
    return input.motion_input[(sensor * 3) + axis];
}

MotionData JoyconPoller::GetMotionInput(const InputReportActive& input,
                                        const MotionStatus& motion_status, std::size_t first,
                                        std::size_t count) const {
    MotionData motion{};
    const auto& accel_cal = motion_calibration.accelerometer;
    const auto& gyro_cal = motion_calibration.gyro;
    motion.delta_timestamp = motion_status.delta_time;

    count = std::max<std::size_t>(count, 1);
    for (std::size_t i = first; i < first + count; ++i) {
        const s16* sample = &imu_samples[i * 6];
        motion.accel_x +=
            GetAccelerometerValue(sample[1], accel_cal[1], motion_status.accelerometer_sensitivity);
        motion.accel_y +=
            GetAccelerometerValue(sample[0], accel_cal[0], motion_status.accelerometer_sensitivity);
        motion.accel_z +=
            GetAccelerometerValue(sample[2], accel_cal[2], motion_status.accelerometer_sensitivity);
        motion.gyro_x += GetGyroValue(sample[4], gyro_cal[1], motion_status.gyro_sensitivity);
        motion.gyro_y += GetGyroValue(sample[3], gyro_cal[0], motion_status.gyro_sensitivity);
        motion.gyro_z += GetGyroValue(sample[5], gyro_cal[2], motion_status.gyro_sensitivity);
    }
    const f32 scale = 1.0f / static_cast<f32>(count);
    motion.accel_x *= scale;
    motion.accel_y *= scale;
    motion.accel_z *= scale;
    motion.gyro_x *= scale;
    motion.gyro_y *= scale;
    motion.gyro_z *= scale;
    return motion;
}

} // namespace InputCommon::Joycon
