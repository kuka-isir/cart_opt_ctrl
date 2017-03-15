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
  waypoints.header.frame_id = "link_0";
  waypoints.header.stamp = ros::Time::now();

  //Get current pose
  ros::ServiceClient pose_client = nh.serviceClient<cart_opt_ctrl::GetCurrentPose>("/CartOptCtrl/getCurrentPose");
  cart_opt_ctrl::GetCurrentPose curr_pos;
  pose_client.call(curr_pos);

  geometry_msgs::Pose curr_pos_msg = curr_pos.response.current_pose;

  waypoints.poses.push_back(curr_pos_msg);

  KDL::Frame curr_pos_kdl ;
  tf::poseMsgToKDL(curr_pos_msg,curr_pos_kdl );
  //Do a rotation along a XYZ axis
  curr_pos_kdl.M.DoRotZ(-5.0*3.141519/180.0);
  tf::poseKDLToMsg(curr_pos_kdl ,curr_pos_msg);
  
  geometry_msgs::Pose pose;
  pose.position.x = -0.052;
  pose.position.y = -0.561;
  pose.position.z = 0.420;
  pose.orientation.x = -0.70711;
  pose.orientation.y = 0.70711;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  waypoints.poses.push_back(pose);
  
  pose.position.x += 0.3;
  pose.position.y += 0.3;
  waypoints.poses.push_back(pose);
  
  pose.position.x -= 0.4;
  pose.position.y -= 0.5;
  pose.position.z -= 0.1;
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
