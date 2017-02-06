#ifndef __KDL_TRAJ_COMPUTE_HPP__
#define __KDL_TRAJ_COMPUTE_HPP__

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/framevel.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_segment.hpp>

#include <tf_conversions/tf_kdl.h>
#include <nav_msgs/Path.h>

class KDLTrajCompute : public RTT::TaskContext
{
public:
  KDLTrajCompute(const std::string& name);
  virtual ~KDLTrajCompute(){}

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void publishTrajectory();
  void computeTrajectory();
protected:
  // Input ports
  RTT::InputPort<geometry_msgs::Pose> port_start_pose_in_;
  // Output ports
  RTT::OutputPort<KDL::Frame> port_pnt_pos_out_;
  RTT::OutputPort<KDL::Twist> port_pnt_vel_out_, port_pnt_acc_out_;
  RTT::OutputPort<nav_msgs::Path> port_path_out_;
  RTT::OutputPort<geometry_msgs::PoseArray> port_pose_array_out_;
  
  geometry_msgs::Pose start_pose_in_;
  KDL::Frame current_pos_;
  KDL::Twist current_vel_, current_acc_;
  
  double current_traj_time_;
  bool new_trajectory_call_, traj_computed_, traj_finished_;
    
  KDL::Frame frame_des_kdl;
  KDL::FrameVel frame_vel_des_kdl;
  KDL::Path_RoundedComposite* path_;
  KDL::Trajectory* traject_;
  KDL::Trajectory_Composite* ctraject_;
  KDL::VelocityProfile* vel_profile_;
  KDL::RotationalInterpolation_SingleAxis* interpolator_;
};


ORO_LIST_COMPONENT_TYPE( KDLTrajCompute )
#endif
