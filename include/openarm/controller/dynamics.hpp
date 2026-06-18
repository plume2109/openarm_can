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

#include <string.h>
#include <unistd.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <memory>
#include <openarm/controller/kdl_parser_compat.hpp>
#include <sstream>
#include <vector>

namespace openarm::controller {

class Dynamics {
private:
    std::shared_ptr<urdf::ModelInterface> urdf_model_interface_;
    std::string urdf_path_;
    std::string start_link_;
    std::string end_link_;
    KDL::JntSpaceInertiaMatrix inertia_matrix_;
    KDL::JntArray coriolis_forces_;
    KDL::JntArray gravity_forces_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainDynParam> solver_;

public:
    Dynamics(const std::string& urdf_path, const std::string& start_link,
             const std::string& end_link);
    ~Dynamics() = default;

    bool Init();

    void GetGravity(const double* motor_position, double* gravity);
    void GetCoriolis(const double* motor_position, const double* motor_velocity, double* coriolis);
    void GetMassMatrixDiagonal(const double* motor_position, double* inertia_diag);
    void GetJacobian(const double* motor_position, Eigen::MatrixXd& jacobian);
    void GetEECoordinate(const double* motor_position, Eigen::Matrix3d& R, Eigen::Vector3d& p);
};

}  // namespace openarm::controller
