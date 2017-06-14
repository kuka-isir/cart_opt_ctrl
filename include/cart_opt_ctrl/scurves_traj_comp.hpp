#ifndef CARTOPTCTRL_SCURVESTRAJCOMP_HPP_
#define CARTOPTCTRL_SCURVESTRAJCOMP_HPP_

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <memory>
#include <rtt_ros_kdl_tools/tools.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_point.hpp>
#include <kdl/framevel.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/utilities/error.h>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <cart_opt_ctrl/UpdateWaypoints.h>
#include <std_msgs/Bool.h>

#include <cart_opt_ctrl/SCurveProfile.hpp>

class SCurvesTrajComp : public RTT::TaskContext{
  public:
    SCurvesTrajComp(const std::string& name);
    virtual ~SCurvesTrajComp(){}

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
    
    RTT::OutputPort<geometry_msgs::Pose> port_debug_pos_out_;
    RTT::OutputPort<geometry_msgs::Twist> port_debug_vel_out_, port_debug_acc_out_;
    
    // Input ports
    RTT::InputPort<bool> port_button_pressed_in_;
    RTT::InputPort<geometry_msgs::PoseStamped> port_x_curr_;
    RTT::InputPort<geometry_msgs::Twist> port_xd_curr_;
    RTT::InputPort<geometry_msgs::Twist> port_xdd_curr_;
    
    double save_acc_, save_vel_, save_pose_;
    KDL::Frame start_pos_kdl_;
    
    
    bool button_pressed_;
    geometry_msgs::PoseStamped goal_pose_, start_pose_, curr_pose_;
    geometry_msgs::Twist curr_vel_, curr_acc_;
    KDL::Frame current_pos_;
    KDL::Twist current_vel_, current_acc_;
    
    double vel_max_, acc_max_, j_max_;
    bool traj_computed_;
    std::string base_frame_;
    
    tf::TransformListener* tf_;
    
    SCurveProfile* scurve_profiler_; 
};

ORO_LIST_COMPONENT_TYPE( SCurvesTrajComp )
#endif // CARTOPTCTRL_SCURVESTRAJCOMP_HPP_
