#!/usr/bin/env python
# Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>

import sys
import copy
import rospy
from geometry_msgs.msg import PoseArray, Pose
from cart_opt_ctrl.srv import UpdateWaypoints, UpdateWaypointsRequest, GetCurrentPose, GetCurrentPoseRequest, GetCurrentPoseResponse

def main(argv):
  rospy.init_node('simple_traj_script')

  #Get current pose
  client_pose_current = rospy.ServiceProxy('/CartOptCtrl/getCurrentPose', GetCurrentPose)
  request_pose = GetCurrentPoseRequest()
  resp = client_pose_current(request_pose);

  if not resp.success:
    rospy.logerror("Service call to /CartOptCtrl/getCurrentPose failed ... EXIT ")
    exit(0)

  waypoints = PoseArray()
  waypoints.header.frame_id = "base_link"
  waypoints.header.stamp = rospy.get_rostime()
  
  waypoints.poses.append(resp.current_pose)
  
  pose = Pose()
  pose.position.x = resp.current_pose.position.x +0.2
  pose.position.y = resp.current_pose.position.y 
  pose.position.z = resp.current_pose.position.z
  pose.orientation.x = 1.0
  pose.orientation.y = 0.0
  pose.orientation.z = 0.0
  pose.orientation.w = 0.0
  waypoints.poses.append(pose)

  client = rospy.ServiceProxy('/SCurvesTrajComp/updateWaypoints', UpdateWaypoints)


  req = UpdateWaypointsRequest();
  req.waypoints = waypoints;

  client.call(req)


if __name__ == "__main__":
  main(sys.argv)
  exit(0)
