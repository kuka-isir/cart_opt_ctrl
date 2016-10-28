// cart_opt_ctrl - ISIR ven. 24 juil. 2015 13:20:21 CEST
// Copyright (c) Antoine Hoarau, All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3.0 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library.

#ifndef __CART_OPT_CTRL_HPP__
#define __CART_OPT_CTRL_HPP__

#include <rtt_ros_kdl_tools/chain_utils.hpp>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <kdl/frameacc.hpp>
#include <qpOASES.hpp>
#include <memory>

#include <geometry_msgs/Pose.h>


template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}


class CartOptCtrl : public RTT::TaskContext
{
public:
    CartOptCtrl(const std::string& name);
    virtual ~CartOptCtrl(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
protected:
    // Output ports
    RTT::OutputPort<Eigen::VectorXd> port_joint_torque_out;
    RTT::OutputPort<geometry_msgs::Pose> port_x_des;
    // Input ports
    RTT::InputPort<KDL::FrameAcc> port_traj_in;

    RTT::InputPort<Eigen::VectorXd> port_joint_position_in;
    RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in;

    // CHain chain_utils
    rtt_ros_kdl_tools::ChainUtils arm;
    Eigen::VectorXd joint_torque_out,
                    joint_position_in,
                    joint_velocity_in;
    KDL::FrameAcc traj_pt_in;
    std::string ee_frame;
    bool has_first_command = false;

    KDL::Frame X_traj,X_curr;
    KDL::Twist X_err,Xd_err,Xdd_err;
    KDL::Twist Xd_curr,Xdd_curr,Xd_traj,Xdd_traj;
    KDL::Twist Xdd_des;

    Eigen::VectorXd P_gain,D_gain;

    std::unique_ptr<qpOASES::SQProblem> qpoases_solver;
};


ORO_CREATE_COMPONENT(CartOptCtrl)
#endif
