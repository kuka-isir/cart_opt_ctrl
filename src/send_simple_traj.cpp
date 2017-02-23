#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <cart_opt_ctrl/UpdateWaypoints.h>
#define DEG2RAD 3.141592653589793/180

int main(int argc, char** argv){
  ros::init(argc, argv, "send_simple_traj");
  ros::NodeHandle nh;
  
  geometry_msgs::PoseArray waypoints;
  waypoints.header.frame_id = "link_0";
  waypoints.header.stamp = ros::Time::now();
  
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
