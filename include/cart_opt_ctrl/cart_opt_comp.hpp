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
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <cart_opt_ctrl/GetCurrentPose.h>


class CartOptCtrl : public RTT::TaskContext{
  public:
    CartOptCtrl(const std::string& name);
    virtual ~CartOptCtrl(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    
    bool getCurrentPose(cart_opt_ctrl::GetCurrentPose::Request& req, cart_opt_ctrl::GetCurrentPose::Response& resp);
    
  protected:
    // Output ports
    RTT::OutputPort<Eigen::VectorXd> port_joint_torque_out_;
    RTT::OutputPort<geometry_msgs::PoseStamped> port_x_des_;
    RTT::OutputPort<trajectory_msgs::JointTrajectoryPoint> port_joint_pos_vel_in_; 
    RTT::OutputPort<geometry_msgs::Twist> port_error_out_; 
    RTT::OutputPort<std_msgs::Float32> port_ec_lim_out_;
    
    // Input ports
    RTT::InputPort<KDL::Frame> port_pnt_pos_in_;
    RTT::InputPort<KDL::Twist> port_pnt_vel_in_;
    RTT::InputPort<KDL::Twist> port_pnt_acc_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in_;
    RTT::InputPort<bool> port_button_pressed_in_;
    RTT::InputPort<geometry_msgs::PointStamped> port_human_pos_in_;
    
    KDL::Jacobian J_;
    KDL::JntSpaceInertiaMatrix M_inv_;
    KDL::JntArray coriolis_;
    KDL::JntArray gravity_;
    KDL::Twist Jdotqdot_;
    Eigen::Matrix<double,6,1> jdot_qdot_;
    Eigen::Matrix<double,6,1> xdd_des_;

    // Matrices for qpOASES
    // NOTE: We need RowMajor (see qpoases doc)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
    Eigen::VectorXd g_;
    Eigen::Matrix<double,6,Eigen::Dynamic> a_;
    Eigen::Matrix<double,6,1> b_;
    Eigen::VectorXd lb_, ub_;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
    Eigen::VectorXd lbA_, ubA_, qd_min_, qd_max_;
    Eigen::VectorXd nonLinearTerms_;
    Eigen::VectorXd x_max_, x_min_;
    Eigen::Matrix<double,6,1> xd_curr_, x_curr_;
    Eigen::Matrix<double,3,1> x_curr_lin_;
    Eigen::Matrix<double,6,6> Lambda_;
    Eigen::Matrix<double,6,1> delta_x_;
    double ec_next_filtered_;
    
    // For ROS debug
    geometry_msgs::Pose x_des_pos_out_;
    geometry_msgs::PoseStamped x_des_pos_stamped_out_;
    trajectory_msgs::JointTrajectoryPoint joint_pos_vel_;
    geometry_msgs::Twist error_twist_ros_;
    
    bool button_pressed_;

    // Chain chain_utils
    rtt_ros_kdl_tools::ChainUtils arm_;
    Eigen::VectorXd joint_torque_out_,
                    joint_position_in_,
                    joint_velocity_in_;
    
    double distance_to_contact_;

    KDL::Frame pt_pos_in_;
    KDL::Twist pt_vel_in_, pt_acc_in_;
    
    std::string ee_frame_, base_frame_;
    bool has_first_command_ = false;

    KDL::Frame X_traj_,X_curr_;
    KDL::Twist X_err_,Xd_err_,Xdd_err_;
    KDL::Twist Xd_curr_,Xdd_curr_,Xd_traj_,Xdd_traj_;
    KDL::Twist Xdd_des_;
    KDL::Twist integral_error_;
    
    Eigen::VectorXd damping_weight_, cart_min_constraints_, cart_max_constraints_;
    Eigen::VectorXd viscous_coeffs_;
    double transition_gain_, regularisation_weight_, horizon_steps_;
    double position_saturation_, orientation_saturation_, integral_pos_saturation_, integral_rot_saturation_;
    double max_viscous_coeff_, viscous_walls_thickness_;
    bool compensate_gravity_, viscous_walls_;
    Eigen::VectorXd p_gains_, i_gains_, d_gains_, torque_max_, jnt_vel_max_;
    std::vector<Eigen::VectorXd> select_components_, select_axes_;
    double ec_lim_, ec_max_, ec_safe_, human_min_dist_, human_max_dist_;

    std::unique_ptr<qpOASES::SQProblem> qpoases_solver_;
    int number_of_constraints_;
};

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( CartOptCtrl )
#endif // CARTOPTCTRL_CARTOPTCOMP_HPP_
