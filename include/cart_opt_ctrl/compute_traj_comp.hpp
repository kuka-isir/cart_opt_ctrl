#ifndef CARTOPTCTRL_COMPUTETRAJCOMP_HPP_
#define CARTOPTCTRL_COMPUTETRAJCOMP_HPP_

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <memory>
#include <rtt_ros_kdl_tools/tools.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/framevel.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/utilities/error.h>

#include <tf_conversions/tf_kdl.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <cart_opt_ctrl/UpdateWaypoints.h>
#include <std_msgs/Bool.h>

class KDLTrajCompute : public RTT::TaskContext{
  public:
    KDLTrajCompute(const std::string& name);
    virtual ~KDLTrajCompute(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    
    void publishTrajectory();
    bool computeTrajectory();
    bool updateWaypoints(cart_opt_ctrl::UpdateWaypoints::Request& req, cart_opt_ctrl::UpdateWaypoints::Response& resp);
    
  protected:
    // Output ports
    RTT::OutputPort<KDL::Frame> port_pnt_pos_out_;
    RTT::OutputPort<KDL::Twist> port_pnt_vel_out_, port_pnt_acc_out_;
    RTT::OutputPort<nav_msgs::Path> port_path_out_;
    RTT::OutputPort<geometry_msgs::PoseArray> port_pose_array_out_;
    
    // Input ports
    RTT::InputPort<std_msgs::Bool> port_button_pressed_in_;
    
    std_msgs::Bool button_pressed_msg_;
    geometry_msgs::PoseArray waypoints_in_;
    KDL::Frame current_pos_;
    KDL::Twist current_vel_, current_acc_;
    
    double current_traj_time_, vel_max_, acc_max_, radius_, eqradius_;
    bool traj_computed_;
      
    KDL::Path_RoundedComposite* path_;
    KDL::Trajectory* traject_;
    KDL::Trajectory_Composite* ctraject_;
    KDL::VelocityProfile* vel_profile_;
    KDL::RotationalInterpolation_SingleAxis* interpolator_;
};

ORO_LIST_COMPONENT_TYPE( KDLTrajCompute )
#endif // CARTOPTCTRL_COMPUTETRAJCOMP_HPP_
