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
                      const std::string&, const std::string&>(),
             nb::arg("can_interface"),
             nb::arg("urdf_path"),
             nb::arg("root_link"),
             nb::arg("tip_link"))
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
        // Gravity compensation mode
        .def("enable_gravity_compensation",
             &OpenArmController::enable_gravity_compensation,
             nb::arg("gripper_position") = 0.0)
        .def("disable_gravity_compensation",
             &OpenArmController::disable_gravity_compensation);
}
