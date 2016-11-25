#ifndef CART_OPT_JOINT_TRAJECTORY_CONTROLLER
#define CART_OPT_JOINT_TRAJECTORY_CONTROLLER

// ROS
#include <ros/node_handle.h>
// ROS messages
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>
// actionlib
#include <actionlib/server/action_server.h>
// realtime_tools
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>
// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
// chain_utils
#include <rtt_ros_kdl_tools/chain_utils.hpp>
// solvers
#include <kdl/chainfksolvervel_recursive.hpp>
#include <qpOASES.hpp>
#include <kdl/velocityprofile_spline.hpp>
// conversions
#include <eigen_conversions/eigen_kdl.h>

// typedefs
typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
typedef ActionServer::GoalHandle                                                            GoalHandle;
typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;

namespace cart_opt_controllers
{
  class JointTrajectoryController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {
  // Funtions
  public:
    JointTrajectoryController();
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    void starting(const ros::Time& time);
    void stopping(const ros::Time& time) { };
    void update(const ros::Time& time, const ros::Duration& period);

  private:
    void goalCB(ActionServer::GoalHandle gh);
    void cancelCB(ActionServer::GoalHandle gh);
    bool queryStateService(control_msgs::QueryTrajectoryState::Request& req, control_msgs::QueryTrajectoryState::Response& resp);
    bool interpolate(trajectory_msgs::JointTrajectory & trajectory_in, trajectory_msgs::JointTrajectory & trajectory_out);
    bool interpolatePt(trajectory_msgs::JointTrajectoryPoint & p1,
                                              trajectory_msgs::JointTrajectoryPoint & p2, double time_from_start,
                                              trajectory_msgs::JointTrajectoryPoint & interp_pt);
  // Variables
  public:
    size_t n_joints_;
    std::vector<hardware_interface::JointHandle> joints_;

    size_t point_index_;
    ros::Time commanded_start_time_;
    realtime_tools::RealtimeBuffer<trajectory_msgs::JointTrajectory> trajectory_command_buffer_;
    
  private:
    std::string name_;
    ros::NodeHandle controller_nh_;
    RealtimeGoalHandlePtr rt_active_goal_;
    ros::Duration action_monitor_period_;
    ActionServerPtr action_server_;
    ros::ServiceServer query_state_service_;
    ros::Timer goal_handle_timer_;

    rtt_ros_kdl_tools::ChainUtils arm_;
    Eigen::VectorXd joint_position_in, joint_velocity_in, joint_torque_out;
                    
    bool has_first_command;
    bool new_reference_;
    std::string ee_frame;

    KDL::Frame X_traj,X_curr;
    KDL::Twist X_err,Xd_err,Xdd_err,Xd_curr,Xdd_curr,Xd_traj,Xdd_traj,Xdd_des;
    KDL::JntArray q_curr, qd_curr, qdd_curr, q_traj, qd_traj, qdd_traj;
    Eigen::VectorXd qdd_des,P_gain,D_gain,P_joint_gain,D_joint_gain;
    double Alpha, Regularisation;
    
    // Solvers
    boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_solver_vel_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jntToJacDotSolver_;
    std::unique_ptr<qpOASES::SQProblem> qpoases_solver;
  };
} // namespace

#endif