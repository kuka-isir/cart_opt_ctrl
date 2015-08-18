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

#include "rtt_lwr_abstract/rtt_lwr_abstract.hpp"
#include "cart_opt_ctrl/cart_model_solver.hpp"

#include <eigen_conversions/eigen_msg.h>
#include <kdl/frames.hpp>

#include <kdl/frames_io.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/frameacc_io.hpp>

#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/trajectory_composite.hpp>

#include <nav_msgs/Path.h>
#include <rtt_ros_kdl_tools/chainjnttojacdotsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

#include <unsupported/Eigen/MatrixFunctions>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

namespace lwr{
  static const int PINV_SOLVER  =0;
  static const int WDL_SOLVER   =1;
  
  class CartOptCtrl : public RTTLWRAbstract{
    public:
      CartOptCtrl(const std::string& name);
      virtual ~CartOptCtrl(){delete ctraject;};
      bool computeTrajectory(const double radius,const double eqradius,const double vmax=0.02, const double accmax=0.1);
      void updateHook();
      bool configureHook();
      void setSolverVerbose(bool v);
      void setSolverTimeLimit(double t);
      void setSolverBarrierConvergeanceTolerance(double t);
      void setSolverMethod(int i);
      RTT::OutputPort<geometry_msgs::PoseStamped> port_X_curr;
      RTT::OutputPort<geometry_msgs::PoseStamped> port_X_des;
      RTT::OutputPort<geometry_msgs::PoseStamped> port_X_tmp;
      RTT::OutputPort<geometry_msgs::PoseArray> port_pose_array;
      RTT::InputPort<geometry_msgs::WrenchStamped> port_ftdata;
      RTT::OutputPort<std_msgs::Float32> port_solver_duration;
      RTT::OutputPort<nav_msgs::Path> port_path_ros;
      RTT::OutputPort<std_msgs::Float32MultiArray> port_qdd_min;
      RTT::OutputPort<std_msgs::Float32MultiArray> port_qdd_max;
      RTT::OutputPort<std_msgs::Float32MultiArray> port_qdd_des;
        
    protected:
      std_msgs::Float32MultiArray qdd_min_ros,qdd_max_ros,qdd_des_ros;
      geometry_msgs::PoseStamped X_curr_msg,X_des_msg,X_tmp_msg;
      bool ready_to_start_;
      double gain_;
      KDL::Frame frame_des_kdl;
      Eigen::Matrix<double,6,1> X_err,Xd_err;
      KDL::Frame frame_kdl;
      KDL::FrameVel frame_vel_des_kdl;
      tf::Pose cart_pos_tf,cart_pos_tf_des;
      KDL::Wrench wrench_kdl;
      KDL::JntArray jnt_acc_kdl;
      Eigen::Matrix<double,6,1> F_ext;
      KDL::Twist cart_twist_des_kdl;

      KDL::Path_RoundedComposite* path;
      KDL::VelocityProfile* velpref;
      KDL::Trajectory* traject;
      KDL::Trajectory_Composite* ctraject;
      void publishTrajectory();
      boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jdot_solver;
      boost::scoped_ptr<KDL::ChainIkSolverVel_wdls> wdls_solver;
      KDL::Jacobian jdot,J_ati_base,J_ee_base;
      KDL::Twist jdot_qdot;
      double t_traj_curr;

      Eigen::Matrix<double,7,1> jnt_pos_eigen;
      Eigen::Matrix<double,7,1> corr_cart;

      boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> pinv_solver;

      double kp_lin,kd_lin,kp_ang,kd_ang;
      KDL::JntArray qdd_des_kdl;
      Eigen::VectorXd qdd_des,coriolis;
      KDL::JntSpaceInertiaMatrix mass_kdl;
      bool traj_computed;
      double d_ang_max_;
      bool debug_mode_;
      const bool isReadyToStart()const{return ready_to_start_;};
      KDL::Twist d_err_last;
      bool use_jdot_qdot_,use_coriolis_,use_f_ext_,use_xdd_des_,use_ft_sensor_;
      int jacobian_solver_type_;
      Eigen::MatrixXd mass_inv;
      double elapsed,dw_max_;
      bool use_mass_sqrt_;
      bool use_xd_des_;
      geometry_msgs::WrenchStamped ft_data;
      Eigen::Matrix<double,6,1> ft_wrench;
      KDL::Wrench ft_wrench_kdl;
      lwr::LWRCartOptSolver cart_model_solver_;
private:
      bool model_verbose_;
      double solver_duration;
  };
}
ORO_CREATE_COMPONENT(lwr::CartOptCtrl)
#endif
