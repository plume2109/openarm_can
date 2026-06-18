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
                      const std::string& root_link, const std::string& tip_link,
                      std::array<double, 7> kp            = {300.0, 300.0, 150.0, 150.0, 40.0, 40.0, 30.0},
                      std::array<double, 7> kd            = {2.5,   2.5,   2.5,   2.5,   0.8,  0.8,  0.8},
                      std::array<double, 7> ki            = {0.2,   0.2,   0.2,   0.2,   0.2,  0.2,  0.2},
                      std::array<double, 7> grav_kd       = {0.1,   0.1,   0.1,   0.1,   0.1,  0.1,  0.1},
                      double               grav_tau_scale = 1.0,
                      double               gripper_max_speed  = 10.0,
                      double               gripper_torque_pu  = 0.25);
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

    // Same as send_joint_action, but adds a host-side integral term plus
    // gravity-torque feedforward (computed internally via the same Dynamics
    // model enable_gravity_compensation() uses — not supplied by the
    // caller) on top of the MIT command. See
    // damiao_motor::DMDeviceCollection::pid_control_one. Useful when kp/kd
    // stiffness alone leaves a steady-state tracking error (e.g. gravity
    // sag) that send_joint_action's pure MIT command won't correct. ki is
    // set at construction.
    //   gain_scale : multiplies kp and kd only (ki and the gravity
    //                feedforward are untouched) — e.g. ramp from 0 to 1
    //                over the first few calls after a mode transition to
    //                avoid a large-gain step input exciting oscillation.
    //                1.0 = full gains (default).
    void send_joint_action_pid(const std::array<double, 7>& positions,
                               double gain_scale = 1.0,
                               double gripper_position = 0.0);
    // Clears the accumulated PID integral term — call when re-entering PID
    // control after a period in a different mode (gravity comp, idle, etc.)
    // to avoid carrying over a stale integral.
    void reset_integral();

    // ── Gravity compensation mode ─────────────────────────────────────────────
    // Runs in the background: kp=0, kd=light damping, tau=gravity feedforward.
    // The arm floats and can be moved by hand.
    // Call get_joint_state() in your own loop to read the demonstrated motion.
    void enable_gravity_compensation(double gripper_position = 0.0);
    void disable_gravity_compensation();

private:
    // ── Hardware ──────────────────────────────────────────────────────────────
    std::unique_ptr<can::socket::OpenArm> hw_;
    // Serializes all hw_ CAN access (refresh/recv/mit/disable/enable) between
    // the background io_thread_ and any thread calling the public API —
    // without this, e.g. disable() can race with a gravity-comp frame the
    // io_thread_ is mid-send on, leaving that motor still torqued.
    mutable std::mutex hw_mutex_;

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

    // ── MIT gains + gripper limits (set at construction, tunable from Python) ──
    std::array<double, 7> KPS_;
    std::array<double, 7> KDS_;
    std::array<double, 7> KIS_;
    std::array<double, 7> GRAV_KD_;
    double                GRAV_TAU_SCALE_;    // multiplier on computed gravity torques
    double                GRIPPER_MAX_SPEED_; // rad/s
    double                GRIPPER_TORQUE_PU_; // per-unit [0–1]

    // ── CAN receive timeout ───────────────────────────────────────────────────
    static constexpr int IO_RECV_TIMEOUT_US_ = 500;

    // ── Helpers ───────────────────────────────────────────────────────────────
    void io_loop();
    void apply_mit(const std::array<double, 7>& q_target,
                   const std::array<double, 7>& kp,
                   const std::array<double, 7>& kd,
                   const std::array<double, 7>& tau_ff,
                   double gripper_pos);
    void apply_pid(const std::array<double, 7>& q_target, double gain_scale, double gripper_pos);
};

}  // namespace openarm::controller
