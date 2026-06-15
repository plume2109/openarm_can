// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <atomic>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "openarm/can/socket/openarm.hpp"
#include "openarm/controller/dynamics.hpp"

namespace openarm::controller {

// ── Dual-arm joint state snapshot ─────────────────────────────────────────────
// positions/velocities/torques: indices 0-6 = left arm, 7-13 = right arm.

struct DualArmState {
    std::array<double, 14> positions   = {};  // rad
    std::array<double, 14> velocities  = {};  // rad/s
    std::array<double, 14> torques     = {};  // Nm
    double gripper_left_position       = 0.0; // rad
    double gripper_left_torque         = 0.0; // Nm
    double gripper_right_position      = 0.0; // rad
    double gripper_right_torque        = 0.0; // Nm
};

// ── Dual-arm controller ────────────────────────────────────────────────────────
// Drives two OpenArm robots from a single IO thread so both CAN buses are
// commanded in the same loop iteration (~1 kHz).

class DualOpenArmController {
public:
    // left_can / right_can : SocketCAN interface names, e.g. "can0" / "can1"
    // urdf_path            : path to the OpenArm URDF (same model for both arms)
    // root_link / tip_link : kinematic chain endpoints in the URDF
    DualOpenArmController(const std::string& left_can,
                          const std::string& right_can,
                          const std::string& urdf_path,
                          const std::string& root_link,
                          const std::string& tip_link);
    ~DualOpenArmController();

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    void enable();
    void disable();

    // ── Read ──────────────────────────────────────────────────────────────────
    // Returns the latest dual-arm state. Thread-safe, updated at ~1 kHz.
    DualArmState get_joint_state() const;

    // ── Write ─────────────────────────────────────────────────────────────────
    // Send a single joint-space command to both arms simultaneously.
    // positions[0..6] → left arm, positions[7..13] → right arm.
    void send_joint_action(const std::array<double, 14>& positions,
                           double gripper_left_position  = 0.0,
                           double gripper_right_position = 0.0);

    // ── Gravity compensation mode ─────────────────────────────────────────────
    // Both arms float simultaneously; call get_joint_state() to record.
    void enable_gravity_compensation(double gripper_left_position  = 0.0,
                                     double gripper_right_position = 0.0);
    void disable_gravity_compensation();

private:
    // ── Hardware ──────────────────────────────────────────────────────────────
    std::unique_ptr<can::socket::OpenArm> hw_left_;
    std::unique_ptr<can::socket::OpenArm> hw_right_;

    // ── Dynamics ──────────────────────────────────────────────────────────────
    std::unique_ptr<Dynamics> dynamics_left_;
    std::unique_ptr<Dynamics> dynamics_right_;

    // ── IO / control thread ───────────────────────────────────────────────────
    std::thread        io_thread_;
    std::atomic<bool>  running_{false};
    mutable std::mutex state_mutex_;
    DualArmState       latest_state_;
    bool               data_ready_{false};

    // ── Gravity compensation state ────────────────────────────────────────────
    std::atomic<bool>   gravity_comp_active_{false};
    std::atomic<double> gravity_comp_gripper_left_{0.0};
    std::atomic<double> gravity_comp_gripper_right_{0.0};

    // ── MIT gains (position control) ──────────────────────────────────────────
    static constexpr std::array<double, 7> KPS_ = {300.0, 300.0, 150.0, 150.0, 40.0, 40.0, 30.0};
    static constexpr std::array<double, 7> KDS_ = {2.5,   2.5,   2.5,   2.5,   0.8,  0.8,  0.8};

    // ── MIT gains (gravity compensation) ─────────────────────────────────────
    static constexpr std::array<double, 7> GRAV_KD_ = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    // ── Gripper POS_FORCE limits ──────────────────────────────────────────────
    static constexpr double GRIPPER_MAX_SPEED_ = 10.0;  // rad/s
    static constexpr double GRIPPER_TORQUE_PU_ = 0.25;  // per-unit [0–1]

    // ── CAN receive timeout ───────────────────────────────────────────────────
    static constexpr int IO_RECV_TIMEOUT_US_ = 500;

    // ── Helpers ───────────────────────────────────────────────────────────────
    void io_loop();
    void apply_mit(can::socket::OpenArm& hw,
                   const std::array<double, 7>& q_target,
                   const std::array<double, 7>& kp,
                   const std::array<double, 7>& kd,
                   const std::array<double, 7>& tau_ff,
                   double gripper_pos);
};

}  // namespace openarm::controller
