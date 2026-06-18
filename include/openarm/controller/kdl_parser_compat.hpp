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

#include <kdl/tree.hpp>
#include <urdf_model/model.h>

// Vendored from the `kdl_parser` ROS package (BSD-3-Clause, Copyright (c)
// 2008, Willow Garage, Inc.) to avoid pulling in ROS build tooling
// (ament_cmake, rcutils) for the single function openarm_can needs.
namespace openarm::controller::kdl_parser_compat {

bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, KDL::Tree& tree);

}  // namespace openarm::controller::kdl_parser_compat
