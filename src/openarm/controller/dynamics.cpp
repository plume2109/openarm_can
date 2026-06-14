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

#include "openarm/controller/dynamics.hpp"

namespace openarm::controller {

Dynamics::Dynamics(const std::string& urdf_path, const std::string& start_link,
                   const std::string& end_link)
    : urdf_path_(urdf_path), start_link_(start_link), end_link_(end_link) {}

bool Dynamics::Init() {
    std::ifstream file(urdf_path_);
    if (!file.is_open()) {
        fprintf(stderr, "[Dynamics] Failed to open URDF file: %s\n", urdf_path_.c_str());
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    urdf_model_interface_ = urdf::parseURDF(buffer.str());
    if (!urdf_model_interface_) {
        fprintf(stderr, "[Dynamics] Failed to parse URDF: %s\n", urdf_path_.c_str());
        return false;
    }

    if (!kdl_parser::treeFromUrdfModel(*urdf_model_interface_, kdl_tree_)) {
        fprintf(stderr, "[Dynamics] Failed to extract KDL tree from URDF\n");
        return false;
    }

    if (!kdl_tree_.getChain(start_link_, end_link_, kdl_chain_)) {
        fprintf(stderr, "[Dynamics] Failed to get KDL chain from %s to %s\n",
                start_link_.c_str(), end_link_.c_str());
        return false;
    }

    const unsigned int njoints = kdl_chain_.getNrOfJoints();
    coriolis_forces_.resize(njoints);
    gravity_forces_.resize(njoints);
    inertia_matrix_.resize(njoints);
    coriolis_forces_.data.setZero();
    gravity_forces_.data.setZero();
    inertia_matrix_.data.setZero();

    solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0, 0.0, -9.81));
    return true;
}

void Dynamics::GetGravity(const double* motor_position, double* gravity) {
    const unsigned int njoints = kdl_chain_.getNrOfJoints();
    KDL::JntArray q(njoints);
    for (unsigned int i = 0; i < njoints; ++i) q(i) = motor_position[i];

    solver_->JntToGravity(q, gravity_forces_);
    for (unsigned int i = 0; i < njoints; ++i) gravity[i] = gravity_forces_(i);
}

void Dynamics::GetCoriolis(const double* motor_position, const double* motor_velocity,
                           double* coriolis) {
    const unsigned int njoints = kdl_chain_.getNrOfJoints();
    KDL::JntArray q(njoints), q_dot(njoints);
    for (unsigned int i = 0; i < njoints; ++i) {
        q(i)     = motor_position[i];
        q_dot(i) = motor_velocity[i];
    }
    solver_->JntToCoriolis(q, q_dot, coriolis_forces_);
    for (unsigned int i = 0; i < njoints; ++i) coriolis[i] = coriolis_forces_(i);
}

void Dynamics::GetMassMatrixDiagonal(const double* motor_position, double* inertia_diag) {
    const unsigned int njoints = kdl_chain_.getNrOfJoints();
    KDL::JntArray q(njoints);
    KDL::JntSpaceInertiaMatrix M(njoints);
    for (unsigned int i = 0; i < njoints; ++i) q(i) = motor_position[i];
    solver_->JntToMass(q, M);
    for (unsigned int i = 0; i < njoints; ++i) inertia_diag[i] = M(i, i);
}

void Dynamics::GetJacobian(const double* motor_position, Eigen::MatrixXd& jacobian) {
    const unsigned int njoints = kdl_chain_.getNrOfJoints();
    KDL::JntArray q(njoints);
    for (unsigned int i = 0; i < njoints; ++i) q(i) = motor_position[i];

    KDL::Jacobian kdl_jac(njoints);
    KDL::ChainJntToJacSolver jac_solver(kdl_chain_);
    jac_solver.JntToJac(q, kdl_jac);

    jacobian.resize(6, njoints);
    for (unsigned int i = 0; i < 6; ++i)
        for (unsigned int j = 0; j < njoints; ++j)
            jacobian(i, j) = kdl_jac(i, j);
}

void Dynamics::GetEECoordinate(const double* motor_position, Eigen::Matrix3d& R,
                               Eigen::Vector3d& p) {
    const unsigned int njoints = kdl_chain_.getNrOfJoints();
    KDL::JntArray q(njoints);
    for (unsigned int i = 0; i < njoints; ++i) q(i) = motor_position[i];

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
    KDL::Frame frame;
    if (fk_solver.JntToCart(q, frame) < 0) return;

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R(i, j) = frame.M(i, j);
    p << frame.p[0], frame.p[1], frame.p[2];
}

}  // namespace openarm::controller
