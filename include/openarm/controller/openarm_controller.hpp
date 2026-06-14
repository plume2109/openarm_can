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
#include <vector>

#include "openarm/can/socket/openarm.hpp"
#include "openarm/controller/dynamics.hpp"

namespace openarm::controller {

// ── Joint state snapshot ──────────────────────────────────────────────────────

struct ArmState {
    std::array<double, 7> positions   = {};   // rad
    std::array<double, 7> velocities  = {};   // rad/s
    std::array<double, 7> torques     = {};   // Nm
    double gripper_position           = 0.0;  // rad
    double gripper_torque             = 0.0;  // Nm
};

// ── Controller ────────────────────────────────────────────────────────────────

class OpenArmController {
public:
    // can_interface : SocketCAN interface name, e.g. "can0"
    // urdf_path     : path to the OpenArm URDF file
    // root_link     : root link name in the URDF, e.g. "openarm_body_link0"
    // tip_link      : tip link name in the URDF, e.g. "openarm_right_hand"
    OpenArmController(const std::string& can_interface, const std::string& urdf_path,
                      const std::string& root_link, const std::string& tip_link);
    ~OpenArmController();

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    void enable();
    void disable();

    // ── Read ──────────────────────────────────────────────────────────────────
    // Returns the latest joint state. Thread-safe, updated at ~1 kHz.
    ArmState get_joint_state() const;

    // ── Write ─────────────────────────────────────────────────────────────────
    // Send a single joint-space command. Call in your own loop at your own dt.
    void send_joint_action(const std::array<double, 7>& positions,
                           double gripper_position = 0.0);

    // ── Gravity compensation mode ─────────────────────────────────────────────
    // Runs in the background: kp=0, kd=light damping, tau=gravity feedforward.
    // The arm floats and can be moved by hand.
    // Call get_joint_state() in your own loop to read the demonstrated motion.
    void enable_gravity_compensation(double gripper_position = 0.0);
    void disable_gravity_compensation();

private:
    // ── Hardware ──────────────────────────────────────────────────────────────
    std::unique_ptr<can::socket::OpenArm> hw_;

    // ── Dynamics ──────────────────────────────────────────────────────────────
    std::unique_ptr<Dynamics> dynamics_;

    // ── IO / control thread ───────────────────────────────────────────────────
    std::thread        io_thread_;
    std::atomic<bool>  running_{false};
    mutable std::mutex state_mutex_;
    ArmState           latest_state_;
    bool               data_ready_{false};

    // ── Gravity compensation state ────────────────────────────────────────────
    std::atomic<bool>   gravity_comp_active_{false};
    std::atomic<double> gravity_comp_gripper_pos_{0.0};

    // ── MIT gains (position control) ──────────────────────────────────────────
    static constexpr std::array<double, 7> KPS_ = {300.0, 300.0, 150.0, 150.0, 40.0, 40.0, 30.0};
    static constexpr std::array<double, 7> KDS_ = {2.5,   2.5,   2.5,   2.5,   0.8,  0.8,  0.8};

    // ── MIT gains (gravity compensation) ─────────────────────────────────────
    static constexpr std::array<double, 7> GRAV_KD_ = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    // ── Gripper POS_FORCE limits ──────────────────────────────────────────────
    static constexpr double GRIPPER_MAX_SPEED_ = 10.0;   // rad/s
    static constexpr double GRIPPER_TORQUE_PU_ = 0.25;   // per-unit [0–1]

    // ── CAN receive timeout ───────────────────────────────────────────────────
    static constexpr int IO_RECV_TIMEOUT_US_ = 500;

    // ── Helpers ───────────────────────────────────────────────────────────────
    void io_loop();
    void apply_mit(const std::array<double, 7>& q_target,
                   const std::array<double, 7>& kp,
                   const std::array<double, 7>& kd,
                   const std::array<double, 7>& tau_ff,
                   double gripper_pos);
};

}  // namespace openarm::controller
