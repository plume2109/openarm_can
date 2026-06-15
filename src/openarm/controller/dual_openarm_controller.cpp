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

#include "openarm/controller/dual_openarm_controller.hpp"

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

DualOpenArmController::DualOpenArmController(const std::string& left_can,
                                             const std::string& right_can,
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
    auto init_hw = [&](const std::string& iface) {
        auto hw = std::make_unique<can::socket::OpenArm>(iface, true);
        hw->init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS, ARM_CONTROL_MODES);
        hw->init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID,
                               damiao_motor::ControlMode::POS_FORCE);
        hw->set_callback_mode_all(damiao_motor::CallbackMode::STATE);
        for (int i = 0; i < 50; ++i) hw->recv_all(0);
        return hw;
    };

    hw_left_  = init_hw(left_can);
    hw_right_ = init_hw(right_can);

    auto init_dyn = [&]() {
        auto dyn = std::make_unique<Dynamics>(urdf_path, root_link, tip_link);
        if (!dyn->Init()) {
            throw std::runtime_error("[DualOpenArmController] Failed to load URDF: " + urdf_path);
        }
        return dyn;
    };

    dynamics_left_  = init_dyn();
    dynamics_right_ = init_dyn();

    running_   = true;
    io_thread_ = std::thread(&DualOpenArmController::io_loop, this);

    std::cout << "[DualOpenArmController] Waiting for hardware on "
              << left_can << " and " << right_can << " ...\n";
    while (true) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (data_ready_) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[DualOpenArmController] Ready.\n";
}

DualOpenArmController::~DualOpenArmController() {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void DualOpenArmController::enable() {
    for (auto* hw : {hw_left_.get(), hw_right_.get()}) {
        hw->set_callback_mode_all(damiao_motor::CallbackMode::STATE);
        hw->enable_all();
        hw->recv_all(2000);
    }
}

void DualOpenArmController::disable() {
    gravity_comp_active_ = false;
    for (auto* hw : {hw_left_.get(), hw_right_.get()}) {
        hw->disable_all();
        hw->recv_all(1000);
    }
}

// ── Read ──────────────────────────────────────────────────────────────────────

DualArmState DualOpenArmController::get_joint_state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

// ── Write ─────────────────────────────────────────────────────────────────────

void DualOpenArmController::send_joint_action(const std::array<double, 14>& positions,
                                              double gripper_left_position,
                                              double gripper_right_position) {
    constexpr std::array<double, 7> zero = {};
    std::array<double, 7> q_left, q_right;
    for (int i = 0; i < 7; ++i) {
        q_left[i]  = positions[i];
        q_right[i] = positions[i + 7];
    }
    apply_mit(*hw_left_,  q_left,  KPS_, KDS_, zero, gripper_left_position);
    apply_mit(*hw_right_, q_right, KPS_, KDS_, zero, gripper_right_position);
}

// ── Gravity compensation mode ─────────────────────────────────────────────────

void DualOpenArmController::enable_gravity_compensation(double gripper_left_position,
                                                        double gripper_right_position) {
    gravity_comp_gripper_left_  = gripper_left_position;
    gravity_comp_gripper_right_ = gripper_right_position;
    gravity_comp_active_        = true;
}

void DualOpenArmController::disable_gravity_compensation() {
    gravity_comp_active_ = false;
}

// ── IO / control thread ───────────────────────────────────────────────────────

void DualOpenArmController::io_loop() {
    std::array<double, 7> grav_left  = {};
    std::array<double, 7> grav_right = {};

    while (running_) {
        // Refresh both buses
        hw_left_->refresh_all();
        hw_right_->refresh_all();
        hw_left_->recv_all(IO_RECV_TIMEOUT_US_);
        hw_right_->recv_all(IO_RECV_TIMEOUT_US_);

        const auto left_motors    = hw_left_->get_arm().get_motors();
        const auto left_gripper   = hw_left_->get_gripper().get_motors()[0];
        const auto right_motors   = hw_right_->get_arm().get_motors();
        const auto right_gripper  = hw_right_->get_gripper().get_motors()[0];

        DualArmState state;
        for (int i = 0; i < 7; ++i) {
            state.positions[i]      = left_motors[i].get_position();
            state.velocities[i]     = left_motors[i].get_velocity();
            state.torques[i]        = left_motors[i].get_torque();
            state.positions[i + 7]  = right_motors[i].get_position();
            state.velocities[i + 7] = right_motors[i].get_velocity();
            state.torques[i + 7]    = right_motors[i].get_torque();
        }
        state.gripper_left_position  = left_gripper.get_position();
        state.gripper_left_torque    = left_gripper.get_torque();
        state.gripper_right_position = right_gripper.get_position();
        state.gripper_right_torque   = right_gripper.get_torque();

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = state;
            data_ready_   = true;
        }

        if (gravity_comp_active_) {
            std::array<double, 7> q_left, q_right;
            for (int i = 0; i < 7; ++i) {
                q_left[i]  = state.positions[i];
                q_right[i] = state.positions[i + 7];
            }
            dynamics_left_->GetGravity(q_left.data(),  grav_left.data());
            dynamics_right_->GetGravity(q_right.data(), grav_right.data());
            for (auto& t : grav_left)  t *= GRAV_TAU_SCALE_;
            for (auto& t : grav_right) t *= GRAV_TAU_SCALE_;
            apply_mit(*hw_left_,  q_left,  {}, GRAV_KD_, grav_left,  gravity_comp_gripper_left_.load());
            apply_mit(*hw_right_, q_right, {}, GRAV_KD_, grav_right, gravity_comp_gripper_right_.load());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// ── MIT command ───────────────────────────────────────────────────────────────

void DualOpenArmController::apply_mit(can::socket::OpenArm& hw,
                                      const std::array<double, 7>& q_target,
                                      const std::array<double, 7>& kp,
                                      const std::array<double, 7>& kd,
                                      const std::array<double, 7>& tau_ff,
                                      double gripper_pos) {
    std::vector<damiao_motor::MITParam> params(7);
    for (int i = 0; i < 7; ++i) {
        params[i] = {kp[i], kd[i], q_target[i], 0.0, tau_ff[i]};
    }
    hw.get_arm().mit_control_all(params);
    hw.get_gripper().posforce_control_one(
        0, damiao_motor::PosForceParam{gripper_pos, GRIPPER_MAX_SPEED_, GRIPPER_TORQUE_PU_});
}

}  // namespace openarm::controller
