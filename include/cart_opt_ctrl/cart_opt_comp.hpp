#ifndef __CART_OPT_COMP_HPP__
#define __CART_OPT_COMP_HPP__

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
    RTT::OutputPort<geometry_msgs::PoseStamped> port_x_des;
    RTT::OutputPort<trajectory_msgs::JointTrajectoryPoint> port_joint_pos_vel_in; 
    RTT::OutputPort<geometry_msgs::Twist> port_error_out; 
    // Input ports
    RTT::InputPort<KDL::Frame> port_pnt_pos_in;
    RTT::InputPort<KDL::Twist> port_pnt_vel_in;
    RTT::InputPort<KDL::Twist> port_pnt_acc_in;
    RTT::InputPort<Eigen::VectorXd> port_joint_position_in;
    RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in;
    RTT::InputPort<std_msgs::Bool> port_button_pressed_in;
    
    std_msgs::Bool button_pressed_msg;
    bool button_pressed;

    // CHain chain_utils
    rtt_ros_kdl_tools::ChainUtils arm;
    Eigen::VectorXd joint_torque_out,
                    joint_position_in,
                    joint_velocity_in;
    

    KDL::Frame pt_pos_in;
    KDL::Twist pt_vel_in, pt_acc_in;
    
    std::string ee_frame;
    bool has_first_command = false;

    KDL::Frame X_traj,X_curr;
    KDL::Twist X_err,Xd_err,Xdd_err;
    KDL::Twist Xd_curr,Xdd_curr,Xd_traj,Xdd_traj;
    KDL::Twist Xdd_des;
    
    double Regularisation_Weight;
    double transition_gain;
    double Position_Saturation, Orientation_Saturation;
    double Damping_Weight;
    bool Compensate_Gravity;
    Eigen::VectorXd P_gain, D_gain, Torque_Max, Joint_Velocity_Max;

    std::unique_ptr<qpOASES::SQProblem> qpoases_solver;
};

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( CartOptCtrl )
#endif
