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
//
// Vendored from the `kdl_parser` ROS package (BSD-3-Clause):
//
// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Only `treeFromUrdfModel` (and the helpers it needs) were kept; the
// file-/string-loading entry points that depended on ROS's `urdf::Model`
// and `rcutils` logging were dropped since openarm_can never calls them.

#include "openarm/controller/kdl_parser_compat.hpp"

#include <kdl/frames_io.hpp>

#include <iostream>
#include <vector>

namespace openarm::controller::kdl_parser_compat {

namespace {

KDL::Vector toKdl(urdf::Vector3 v) { return KDL::Vector(v.x, v.y, v.z); }

KDL::Rotation toKdl(urdf::Rotation r) { return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w); }

KDL::Frame toKdl(urdf::Pose p) { return KDL::Frame(toKdl(p.rotation), toKdl(p.position)); }

KDL::Joint toKdl(urdf::JointSharedPtr jnt) {
    KDL::Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);

    switch (jnt->type) {
        case urdf::Joint::FIXED: {
            return KDL::Joint(jnt->name, KDL::Joint::None);
        }
        case urdf::Joint::REVOLUTE: {
            KDL::Vector axis = toKdl(jnt->axis);
            return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::RotAxis);
        }
        case urdf::Joint::CONTINUOUS: {
            KDL::Vector axis = toKdl(jnt->axis);
            return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::RotAxis);
        }
        case urdf::Joint::PRISMATIC: {
            KDL::Vector axis = toKdl(jnt->axis);
            return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * axis, KDL::Joint::TransAxis);
        }
        default: {
            std::cerr << "[kdl_parser_compat] Converting unknown joint type of joint '"
                      << jnt->name << "' into a fixed joint\n";
            return KDL::Joint(jnt->name, KDL::Joint::None);
        }
    }
}

KDL::RigidBodyInertia toKdl(urdf::InertialSharedPtr i) {
    KDL::Frame origin = toKdl(i->origin);

    double kdl_mass = i->mass;
    KDL::Vector kdl_com = origin.p;

    KDL::RotationalInertia urdf_inertia =
        KDL::RotationalInertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz);

    KDL::RigidBodyInertia kdl_inertia_wrt_com_workaround =
        origin.M * KDL::RigidBodyInertia(0, KDL::Vector::Zero(), urdf_inertia);

    KDL::RotationalInertia kdl_inertia_wrt_com = kdl_inertia_wrt_com_workaround.getRotationalInertia();

    return KDL::RigidBodyInertia(kdl_mass, kdl_com, kdl_inertia_wrt_com);
}

bool addChildrenToTree(urdf::LinkConstSharedPtr root, KDL::Tree& tree) {
    std::vector<urdf::LinkSharedPtr> children = root->child_links;

    KDL::RigidBodyInertia inert(0);
    if (root->inertial) {
        inert = toKdl(root->inertial);
    }

    KDL::Joint jnt = toKdl(root->parent_joint);

    KDL::Segment sgm(root->name, jnt, toKdl(root->parent_joint->parent_to_joint_origin_transform),
                      inert);

    tree.addSegment(sgm, root->parent_joint->parent_link_name);

    for (size_t i = 0; i < children.size(); i++) {
        if (!addChildrenToTree(children[i], tree)) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool treeFromUrdfModel(const urdf::ModelInterface& robot_model, KDL::Tree& tree) {
    if (!robot_model.getRoot()) {
        return false;
    }

    tree = KDL::Tree(robot_model.getRoot()->name);

    if (robot_model.getRoot()->inertial) {
        std::cerr << "[kdl_parser_compat] The root link " << robot_model.getRoot()->name
                  << " has an inertia specified in the URDF, but KDL does not support a root "
                     "link with an inertia. As a workaround, you can add an extra dummy link to "
                     "your URDF.\n";
    }

    for (size_t i = 0; i < robot_model.getRoot()->child_links.size(); i++) {
        if (!addChildrenToTree(robot_model.getRoot()->child_links[i], tree)) {
            return false;
        }
    }

    return true;
}

}  // namespace openarm::controller::kdl_parser_compat
