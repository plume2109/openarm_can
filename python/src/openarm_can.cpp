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

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <openarm/controller/openarm_controller.hpp>
#include <openarm/controller/dual_openarm_controller.hpp>

using namespace openarm::controller;

namespace nb = nanobind;

NB_MODULE(openarm_can, m) {
    m.doc() = "OpenArm CAN — low-level controller for lili-o move V2";

    // ── ArmState ──────────────────────────────────────────────────────────────
    nb::class_<ArmState>(m, "ArmState")
        .def(nb::init<>())
        .def_rw("positions",        &ArmState::positions)
        .def_rw("velocities",       &ArmState::velocities)
        .def_rw("torques",          &ArmState::torques)
        .def_rw("gripper_position", &ArmState::gripper_position)
        .def_rw("gripper_torque",   &ArmState::gripper_torque);

    // ── OpenArmController ─────────────────────────────────────────────────────
    nb::class_<OpenArmController>(m, "OpenArmController")
        .def(nb::init<const std::string&, const std::string&,
                      const std::string&, const std::string&,
                      std::array<double, 7>, std::array<double, 7>, std::array<double, 7>,
                      std::array<double, 7>, double, double, double>(),
             nb::arg("can_interface"),
             nb::arg("urdf_path"),
             nb::arg("root_link"),
             nb::arg("tip_link"),
             nb::arg("kp")               = std::array<double, 7>{300.0, 300.0, 150.0, 150.0, 40.0, 40.0, 30.0},
             nb::arg("kd")               = std::array<double, 7>{2.5,   2.5,   2.5,   2.5,   0.8,  0.8,  0.8},
             nb::arg("ki")               = std::array<double, 7>{0.2,   0.2,   0.2,   0.2,   0.2,  0.2,  0.2},
             nb::arg("grav_kd")          = std::array<double, 7>{0.1,   0.1,   0.1,   0.1,   0.1,  0.1,  0.1},
             nb::arg("grav_tau_scale")   = 1.0,
             nb::arg("gripper_max_speed") = 10.0,
             nb::arg("gripper_torque_pu") = 0.25)
        // Lifecycle
        .def("enable",  &OpenArmController::enable)
        .def("disable", &OpenArmController::disable)
        // Read
        .def("get_joint_state", &OpenArmController::get_joint_state)
        // Write
        .def("send_joint_action",
             &OpenArmController::send_joint_action,
             nb::arg("positions"),
             nb::arg("gripper_position") = 0.0)
        // Write (PID: MIT + host-side integral + gravity feedforward,
        // ki set at construction)
        .def("send_joint_action_pid",
             &OpenArmController::send_joint_action_pid,
             nb::arg("positions"),
             nb::arg("gain_scale") = 1.0,
             nb::arg("gripper_position") = 0.0)
        .def("reset_integral", &OpenArmController::reset_integral)
        // Gravity compensation mode
        .def("enable_gravity_compensation",
             &OpenArmController::enable_gravity_compensation,
             nb::arg("gripper_position") = 0.0)
        .def("disable_gravity_compensation",
             &OpenArmController::disable_gravity_compensation);

    // ── DualArmState ──────────────────────────────────────────────────────────
    nb::class_<DualArmState>(m, "DualArmState")
        .def(nb::init<>())
        .def_rw("positions",             &DualArmState::positions)
        .def_rw("velocities",            &DualArmState::velocities)
        .def_rw("torques",               &DualArmState::torques)
        .def_rw("gripper_left_position", &DualArmState::gripper_left_position)
        .def_rw("gripper_left_torque",   &DualArmState::gripper_left_torque)
        .def_rw("gripper_right_position",&DualArmState::gripper_right_position)
        .def_rw("gripper_right_torque",  &DualArmState::gripper_right_torque);

    // ── DualOpenArmController ─────────────────────────────────────────────────
    nb::class_<DualOpenArmController>(m, "DualOpenArmController")
        .def(nb::init<const std::string&, const std::string&,
                      const std::string&, const std::string&, const std::string&,
                      std::array<double, 7>, std::array<double, 7>, std::array<double, 7>,
                      std::array<double, 7>, double, double, double>(),
             nb::arg("left_can"),
             nb::arg("right_can"),
             nb::arg("urdf_path"),
             nb::arg("root_link"),
             nb::arg("tip_link"),
             nb::arg("kp")                = std::array<double, 7>{300.0, 300.0, 150.0, 150.0, 40.0, 40.0, 30.0},
             nb::arg("kd")                = std::array<double, 7>{2.5,   2.5,   2.5,   2.5,   0.8,  0.8,  0.8},
             nb::arg("ki")                = std::array<double, 7>{0.2,   0.2,   0.2,   0.2,   0.2,  0.2,  0.2},
             nb::arg("grav_kd")           = std::array<double, 7>{0.1,   0.1,   0.1,   0.1,   0.1,  0.1,  0.1},
             nb::arg("grav_tau_scale")    = 1.0,
             nb::arg("gripper_max_speed") = 10.0,
             nb::arg("gripper_torque_pu") = 0.25)
        // Lifecycle
        .def("enable",  &DualOpenArmController::enable)
        .def("disable", &DualOpenArmController::disable)
        // Read
        .def("get_joint_state", &DualOpenArmController::get_joint_state)
        // Write — positions[0..6] = left, positions[7..13] = right
        .def("send_joint_action",
             &DualOpenArmController::send_joint_action,
             nb::arg("positions"),
             nb::arg("gripper_left_position")  = 0.0,
             nb::arg("gripper_right_position") = 0.0)
        // Write (PID: MIT + host-side integral + gravity feedforward,
        // ki set at construction)
        .def("send_joint_action_pid",
             &DualOpenArmController::send_joint_action_pid,
             nb::arg("positions"),
             nb::arg("gain_scale") = 1.0,
             nb::arg("gripper_left_position")  = 0.0,
             nb::arg("gripper_right_position") = 0.0)
        .def("reset_integral", &DualOpenArmController::reset_integral)
        // Gravity compensation mode
        .def("enable_gravity_compensation",
             &DualOpenArmController::enable_gravity_compensation,
             nb::arg("gripper_left_position")  = 0.0,
             nb::arg("gripper_right_position") = 0.0)
        .def("disable_gravity_compensation",
             &DualOpenArmController::disable_gravity_compensation);
}
