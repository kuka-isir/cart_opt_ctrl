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

// Actions Clients !!
#include <krl_msgs/LINAction.h>
#include <krl_msgs/PTPAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <kdl/frames_io.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <cart_opt_ctrl/GetCurrentPose.h>

#include <tf/tf.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class CartOptCtrl : public RTT::TaskContext{
  public:
    CartOptCtrl(const std::string& name);
    virtual ~CartOptCtrl(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();

    double FrictionCompensation(int JointNumber,double TorqueFriction);
    bool getCurrentPose(cart_opt_ctrl::GetCurrentPose::Request& req, cart_opt_ctrl::GetCurrentPose::Response& resp);
    bool getPointingPose(cart_opt_ctrl::GetCurrentPose::Request& req, cart_opt_ctrl::GetCurrentPose::Response& resp);
    KDL::Rotation PointTarget(KDL::Frame frame_target, KDL::Frame frame_des);
    double KineticEnergy();
    Eigen::Matrix<double,2,1> GetPointingTarget(KDL::Frame frame,KDL::Frame frame_target);

  protected:
    // Output ports
    RTT::OutputPort<Eigen::VectorXd> port_joint_torque_out_;
    RTT::OutputPort<geometry_msgs::PoseStamped> port_x_ee_des_,port_x_target_,port_x_O0OM_,port_x_ee_;
    RTT::OutputPort<geometry_msgs::TwistStamped> port_Xm_err,port_X_err;
    RTT::OutputPort<trajectory_msgs::JointTrajectoryPoint> port_joint_pos_vel_in_;
    RTT::OutputPort<geometry_msgs::Twist> port_error_out_,port_Xd_out_,port_xdd_des_,port_mes_err;
    RTT::OutputPort<std_msgs::Float64MultiArray> port_Xd_out_const_,port_xdd_filtered,port_joint_position_out_,port_joint_velocity_out_,inertia_msg,port_Jm,port_J;

    // Input ports
    RTT::InputPort<KDL::Frame> port_pnt_pos_in_;
    RTT::InputPort<KDL::Twist> port_pnt_vel_in_;
    RTT::InputPort<KDL::Twist> port_pnt_acc_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in_;
    RTT::InputPort<std_msgs::Float64> port_encoder_in_;
//     RTT::InputPort<geometry_msgs::PoseStamped> port_x_mes_;
    RTT::InputPort<std_msgs::Bool> port_button_pressed_in_;
    RTT::InputPort<geometry_msgs::PoseArray> port_polaris_in;
    RTT::InputPort<geometry_msgs::WrenchStamped> port_ft_sensor_in_;
    std_msgs::Bool button_pressed_msg_;
    bool button_pressed_;

    std_msgs::Float64 encoder_data_msg_;
    double encoder_data_;
    double ray_length,ray_length2;

    geometry_msgs::PoseArray polaris_array_;
    geometry_msgs::WrenchStamped ft_data_msg_;
    geometry_msgs::Pose base,end_effector;
    // Chain chain_utils
    rtt_ros_kdl_tools::ChainUtils arm_;
    Eigen::VectorXd joint_torque_out_,
                    joint_position_in_,
                    joint_velocity_in_;


    KDL::Frame pt_pos_in_;
    KDL::Twist pt_vel_in_, pt_acc_in_;

    std::string ee_frame_,link_6;
    bool has_first_command_ = false;
    bool start_pointing,point_to_target,moving_target;
    KDL::Rotation rotz ;
    KDL::Frame X_traj_,X_curr_,frame_target;
    KDL::Twist X_err_,Xd_err_,Xdd_err_;
    KDL::Twist Xd_curr_,Old_xd_curr_,Xdd_curr_,Xd_traj_,Xdd_traj_;
    KDL::Twist Old_xdd_des,Xdd_des,Xddm_des;
    KDL::Vector Xl;
    KDL::Frame X_laser;
    KDL::Twist integral_term,integral_term_sat,integral_m_term,integral_m_term_sat;
    
    KDL::Frame O0OM,O6OM;
    
    double integral_pos_saturation,integral_ori_saturation;

    bool rotation,activate_pointing;
    double regularisation_weight_;
    Eigen::VectorXd damping_weight_;
//     Eigen::Matrix<double,6,6> button_selection;
    double transition_gain_;
    double Ec_lim , F_lim;
    double horizon;
    double position_saturation_, orientation_saturation_;
    bool compensate_gravity_;
    bool perturbation,inc,record;
    int number_of_constraints;
    std::string link_6_frame,link_laser,ati_link;
    Eigen::VectorXd p_gains_, d_gains_, i_gains_, torque_max_, jnt_vel_max_;
    double targetx,targety,targetz;
    Eigen::Matrix<double,3,1> target;
    Eigen::Matrix<double,2,1> Xx,Xx_polaris;
    Eigen::Matrix<double,6,1> delta_x;
    Eigen::VectorXd a_butter,b_butter;
    Eigen::Matrix<double,3,6> filtered_vel,filtered_acc,old_vel,old_acc;
    Eigen::Matrix<double,6,1> Xdd_filtered, xd_curr_filtered_, xd_curr_;
    Eigen::Matrix<double,6,2> xd_curr_old_,xd_curr_filtered_old_,xdd_old_,xdd_filtered_old_;
    std::vector<Eigen::VectorXd> select_components_, select_axes_;
    KDL::Frame base_polaris,ee_polaris;
    std::unique_ptr<qpOASES::SQProblem> qpoases_solver_;
    double friction_jnt_4,friction_jnt_5,friction_jnt_6,torque_thresh;
    Eigen::Matrix<double,3,1> ff_xd,X_err_old,ff_xd_old,ff_xdd;

    Eigen::Matrix<double,6,1> delta_x_filtered;
    double time_record;
    geometry_msgs::Twist xdd_des_msg,xdd_des_const_msg,integral_term_msg;
    std_msgs::Float64MultiArray delta_x_msg,
				xd_curr_filtered_msg_,
				xdd_filtered_msg_,
				joint_pos_msg_,
				joint_vel_msg_,
				force_info_msg;

    double X_err_norm_vel,X_err_norm_rot;

    RTT::OutputPort<std_msgs::Float64MultiArray> port_delta_x_info,
						 port_force_info;

    std_msgs::Float64MultiArray Ec_constraints_msg,Data_thomas_msg,norm_error_msg,kp_x_err_msg,Xdd_out_msg,xdd_des_const_float_msg;
    RTT::OutputPort<std_msgs::Float64MultiArray> port_Ec_constraints,port_Data_Thomas,port_norm_error,port_kp_x_err,port_Xdd_out,port_joint_torque_,port_xdd_des_const_;

    std_msgs::Float64 positioning_error,positioning_error_polaris,pointing_error,Ec_constraint2,pointing_error_polaris;
    RTT::OutputPort<std_msgs::Float64> port_positioning_error,port_positioning_error_polaris,port_pointing_error,port_pointing_error_polaris,port_Ec_constraint2;

    RTT::OutputPort< geometry_msgs::Twist> port_integral_term;

};

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( CartOptCtrl )
#endif // CARTOPTCTRL_CARTOPTCOMP_HPP_
