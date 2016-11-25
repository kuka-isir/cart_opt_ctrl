#ifndef __CART_TRAJ_INTERP_HPP__
#define __CART_TRAJ_INTERP_HPP__

#include <rtt_ros_kdl_tools/chain_utils.hpp>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <memory>
#include <iostream>

class CartTrajInterp : public RTT::TaskContext
{
public:
  CartTrajInterp(const std::string& name);
  virtual ~CartTrajInterp(){}

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  
  // Interpolation came from industrial_trajectory_filters package
  /*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Southwest Research Institute, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
  bool interpolate(trajectory_msgs::JointTrajectory& trajectory_in, trajectory_msgs::JointTrajectory& trajectory_out);
  bool interpolatePt(trajectory_msgs::JointTrajectoryPoint & p1,
                      trajectory_msgs::JointTrajectoryPoint & p2, double time_from_start,
                      trajectory_msgs::JointTrajectoryPoint & interp_pt);
protected:
  // Input ports
  RTT::InputPort<moveit_msgs::MoveGroupActionResult> port_traj_in_;
  RTT::InputPort<Eigen::VectorXd> port_joint_position_in_;
  RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in_;
  
  // Current joint position and velocity
  Eigen::VectorXd joint_position_in_, joint_velocity_in_;
  
  // Output ports
  RTT::OutputPort<KDL::FrameAcc> port_traj_pt_out_;
  RTT::OutputPort<KDL::JntArrayAcc> port_traj_joint_out_;
  
  // Chain utils
  rtt_ros_kdl_tools::ChainUtils arm_;
  
  // Trajectory in
  moveit_msgs::MoveGroupActionResult traj_in_;
  trajectory_msgs::JointTrajectory traj_curr_;
  KDL::JntArrayAcc pt_in_;
  
  // Cartesian trajectory point out
  KDL::FrameAcc traj_pt_out_;
  
  // FK solver
  boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_solver_vel_;
  
  // Jdot solver
  boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jntToJacDotSolver_;
  
  // Number of joints
  int dof_;
  
  // Specify which command point needs to be sent
  int traj_pt_nb_;
  
  // The time between each point to send to the controller
  double sample_duration_;
  
};
ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( CartTrajInterp )
#endif