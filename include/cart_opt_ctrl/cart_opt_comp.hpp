#ifndef CARTOPTCTRL_CARTOPTCOMP_HPP_
#define CARTOPTCTRL_CARTOPTCOMP_HPP_

#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <kdl/frameacc.hpp>
#include <qpOASES.hpp>
#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl_conversions/kdl_msg.h>


class CartOptCtrl : public RTT::TaskContext{
  public:
    CartOptCtrl(const std::string& name);
    virtual ~CartOptCtrl(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    
  protected:
    // Output ports
    RTT::OutputPort<Eigen::VectorXd> port_joint_torque_out_;
    RTT::OutputPort<geometry_msgs::PoseStamped> port_x_des_;
    RTT::OutputPort<trajectory_msgs::JointTrajectoryPoint> port_joint_pos_vel_in_; 
    RTT::OutputPort<geometry_msgs::Twist> port_error_out_; 
    
    // Input ports
    RTT::InputPort<KDL::Frame> port_pnt_pos_in_;
    RTT::InputPort<KDL::Twist> port_pnt_vel_in_;
    RTT::InputPort<KDL::Twist> port_pnt_acc_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in_;
    RTT::InputPort<std_msgs::Bool> port_button_pressed_in_;
    
    std_msgs::Bool button_pressed_msg_;
    bool button_pressed_;

    // Chain chain_utils
    rtt_ros_kdl_tools::ChainUtils arm_;
    Eigen::VectorXd joint_torque_out_,
                    joint_position_in_,
                    joint_velocity_in_;
    

    KDL::Frame pt_pos_in_;
    KDL::Twist pt_vel_in_, pt_acc_in_;
    
    std::string ee_frame_;
    bool has_first_command_ = false;

    KDL::Frame X_traj_,X_curr_;
    KDL::Twist X_err_,Xd_err_,Xdd_err_;
    KDL::Twist Xd_curr_,Xdd_curr_,Xd_traj_,Xdd_traj_;
    KDL::Twist Xdd_des;
    
    Eigen::VectorXd regularisation_weight_, damping_weight_;
    double transition_gain_;
    double position_saturation_, orientation_saturation_;
    bool compensate_gravity_;
    Eigen::VectorXd p_gains_, d_gains_, torque_max_, jnt_vel_max_;
    std::vector<Eigen::VectorXd> select_components_, select_axes_;

    std::unique_ptr<qpOASES::SQProblem> qpoases_solver_;
};

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( CartOptCtrl )
#endif // CARTOPTCTRL_CARTOPTCOMP_HPP_
