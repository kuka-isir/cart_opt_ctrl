#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <cart_opt_ctrl/UpdateWaypoints.h>
#include <cart_opt_ctrl/GetCurrentPose.h>
#define DEG2RAD 3.141592653589793/180

int main(int argc, char** argv){
  ros::init(argc, argv, "send_simple_traj");
  ros::NodeHandle nh;

  geometry_msgs::PoseArray waypoints;

  std::string root_link;
  if(!nh.getParam("root_link", root_link))
  {
      std::cerr << "No root_link param" << '\n';
      return 0;
  }

  waypoints.header.frame_id = root_link;
  waypoints.header.stamp = ros::Time::now();

  //Get current pose
  ros::ServiceClient pose_client = nh.serviceClient<cart_opt_ctrl::GetCurrentPose>("/CartOptCtrl/getCurrentPose");
  cart_opt_ctrl::GetCurrentPose curr_pos;
  pose_client.call(curr_pos);

  geometry_msgs::Pose curr_pos_msg = curr_pos.response.current_pose;
  waypoints.poses.push_back(curr_pos_msg);

  geometry_msgs::Pose pose;
  pose.position.x = 0.4;
  pose.position.y = 0.0;
  pose.position.z = 0.4;
  pose.orientation.x = -0.70711;
  pose.orientation.y = 0.70711;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  waypoints.poses.push_back(pose);

  pose.position.y += 0.3;
  waypoints.poses.push_back(pose);

  pose.position.y -= 0.5;
  pose.position.z -= 0.1;

  KDL::Frame pos_kdl ;
  tf::poseMsgToKDL(pose,pos_kdl );
  //Do a rotation along a XYZ axis
  pos_kdl.M.DoRotZ(-60.0*DEG2RAD);
  tf::poseKDLToMsg(pos_kdl ,pose);

  waypoints.poses.push_back(pose);

  ros::ServiceClient client = nh.serviceClient<cart_opt_ctrl::UpdateWaypoints>("/KDLTrajCompute/updateWaypoints");
  cart_opt_ctrl::UpdateWaypoints srv;
  srv.request.waypoints = waypoints;
  if (client.call(srv))
  {
    ROS_INFO("Service call succeedeed");
  }
  else
  {
    ROS_INFO("Service call failed");
  }

  ros::shutdown();
  return 1;
}
