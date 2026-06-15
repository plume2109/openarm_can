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

#include "openarm/controller/openarm_controller.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace openarm::controller {

// ── Hardware configuration ────────────────────────────────────────────────────

static const std::vector<damiao_motor::MotorType> ARM_MOTOR_TYPES = {
    damiao_motor::MotorType::DM8009,  // joint 0 – shoulder pitch
    damiao_motor::MotorType::DM8009,  // joint 1 – shoulder roll
    damiao_motor::MotorType::DM4340,  // joint 2 – shoulder yaw
    damiao_motor::MotorType::DM4340,  // joint 3 – elbow flex
    damiao_motor::MotorType::DM4310,  // joint 4 – wrist 1
    damiao_motor::MotorType::DM4310,  // joint 5 – wrist 2
    damiao_motor::MotorType::DM4310,  // joint 6 – wrist 3
};
static const std::vector<uint32_t> ARM_SEND_IDS = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
static const std::vector<uint32_t> ARM_RECV_IDS = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
static const std::vector<damiao_motor::ControlMode> ARM_CONTROL_MODES = {
    damiao_motor::ControlMode::MIT,  // joint 0 – shoulder pitch
    damiao_motor::ControlMode::MIT,  // joint 1 – shoulder roll
    damiao_motor::ControlMode::MIT,  // joint 2 – shoulder yaw
    damiao_motor::ControlMode::MIT,  // joint 3 – elbow flex
    damiao_motor::ControlMode::MIT,  // joint 4 – wrist 1
    damiao_motor::ControlMode::MIT,  // joint 5 – wrist 2
    damiao_motor::ControlMode::MIT,  // joint 6 – wrist 3
};

static constexpr damiao_motor::MotorType GRIPPER_MOTOR_TYPE = damiao_motor::MotorType::DM4310;
static constexpr uint32_t               GRIPPER_SEND_ID     = 0x08;
static constexpr uint32_t               GRIPPER_RECV_ID     = 0x18;

// ── Constructor / Destructor ──────────────────────────────────────────────────

OpenArmController::OpenArmController(const std::string& can_interface,
                                     const std::string& urdf_path,
                                     const std::string& root_link,
                                     const std::string& tip_link,
                                     std::array<double, 7> kp,
                                     std::array<double, 7> kd,
                                     std::array<double, 7> grav_kd,
                                     double               grav_tau_scale,
                                     double               gripper_max_speed,
                                     double               gripper_torque_pu)
    : KPS_(kp), KDS_(kd), GRAV_KD_(grav_kd),
      GRAV_TAU_SCALE_(grav_tau_scale),
      GRIPPER_MAX_SPEED_(gripper_max_speed),
      GRIPPER_TORQUE_PU_(gripper_torque_pu) {
    // Hardware (CAN-FD enabled)
    hw_ = std::make_unique<can::socket::OpenArm>(can_interface, true);
    hw_->init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS, ARM_CONTROL_MODES);
    hw_->init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID,
                            damiao_motor::ControlMode::POS_FORCE);
    hw_->set_callback_mode_all(damiao_motor::CallbackMode::STATE);

    // Flush stale frames
    for (int i = 0; i < 50; ++i) hw_->recv_all(0);

    // Dynamics
    dynamics_ = std::make_unique<Dynamics>(urdf_path, root_link, tip_link);
    if (!dynamics_->Init()) {
        throw std::runtime_error("[OpenArmController] Failed to load URDF: " + urdf_path);
    }

    // Start IO thread and wait for first valid state
    running_ = true;
    io_thread_ = std::thread(&OpenArmController::io_loop, this);

    std::cout << "[OpenArmController] Waiting for hardware on " << can_interface << " ...\n";
    while (true) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (data_ready_) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[OpenArmController] Ready.\n";
}

OpenArmController::~OpenArmController() {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void OpenArmController::enable() {
    hw_->set_callback_mode_all(damiao_motor::CallbackMode::STATE);
    hw_->enable_all();
    hw_->recv_all(2000);
}

void OpenArmController::disable() {
    gravity_comp_active_ = false;
    hw_->disable_all();
    hw_->recv_all(1000);
}

// ── Read ──────────────────────────────────────────────────────────────────────

ArmState OpenArmController::get_joint_state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

// ── Write ─────────────────────────────────────────────────────────────────────

void OpenArmController::send_joint_action(const std::array<double, 7>& positions,
                                          double gripper_position) {
    constexpr std::array<double, 7> zero = {};
    apply_mit(positions, KPS_, KDS_, zero, gripper_position);
}

// ── Gravity compensation mode ─────────────────────────────────────────────────

void OpenArmController::enable_gravity_compensation(double gripper_position) {
    gravity_comp_gripper_pos_ = gripper_position;
    gravity_comp_active_      = true;
}

void OpenArmController::disable_gravity_compensation() {
    gravity_comp_active_ = false;
}

// ── IO / control thread ───────────────────────────────────────────────────────

void OpenArmController::io_loop() {
    std::array<double, 7> gravity = {};

    while (running_) {
        // Read state from hardware
        hw_->refresh_all();
        hw_->recv_all(IO_RECV_TIMEOUT_US_);

        const auto arm_motors    = hw_->get_arm().get_motors();
        const auto gripper_motor = hw_->get_gripper().get_motors()[0];

        ArmState state;
        for (int i = 0; i < 7; ++i) {
            state.positions[i]  = arm_motors[i].get_position();
            state.velocities[i] = arm_motors[i].get_velocity();
            state.torques[i]    = arm_motors[i].get_torque();
        }
        state.gripper_position = gripper_motor.get_position();
        state.gripper_torque   = gripper_motor.get_torque();

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = state;
            data_ready_   = true;
        }

        // Gravity compensation: runs entirely in this thread when active
        if (gravity_comp_active_) {
            dynamics_->GetGravity(state.positions.data(), gravity.data());
            for (auto& t : gravity) t *= GRAV_TAU_SCALE_;
            apply_mit(state.positions, {}, GRAV_KD_, gravity,
                      gravity_comp_gripper_pos_.load());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// ── MIT command ───────────────────────────────────────────────────────────────

void OpenArmController::apply_mit(const std::array<double, 7>& q_target,
                                  const std::array<double, 7>& kp,
                                  const std::array<double, 7>& kd,
                                  const std::array<double, 7>& tau_ff,
                                  double gripper_pos) {
    std::vector<damiao_motor::MITParam> params(7);
    for (int i = 0; i < 7; ++i) {
        params[i] = {kp[i], kd[i], q_target[i], 0.0, tau_ff[i]};
    }
    hw_->get_arm().mit_control_all(params);
    hw_->get_gripper().posforce_control_one(
        0, damiao_motor::PosForceParam{gripper_pos, GRIPPER_MAX_SPEED_, GRIPPER_TORQUE_PU_});
}

}  // namespace openarm::controller
