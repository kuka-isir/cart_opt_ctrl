#!/usr/bin/env python
# Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>

import sys
import copy
import rospy
from time import sleep
from geometry_msgs.msg import PoseArray, Pose
from cart_opt_ctrl.srv import UpdateWaypoints, UpdateWaypointsRequest, GetCurrentPose, GetCurrentPoseRequest, GetCurrentPoseResponse

def main(argv):
  rospy.init_node('go_to_point')

  #Get current pose
  client_pose_current = rospy.ServiceProxy('/CartOptCtrl/getCurrentPose', GetCurrentPose)
  request_pose = GetCurrentPoseRequest()
  resp = client_pose_current(request_pose);

  if not resp.success:
    rospy.logerror("Service call to /CartOptCtrl/getCurrentPose failed ... EXIT ")
    exit(0)

  waypoints = PoseArray()
  waypoints.header.frame_id = "table_link"
  
  waypoints.poses.append(resp.current_pose)

  
  pose = Pose()
  pose.position.x = 0.55656 
  pose.position.y = 0.0
  pose.position.z = 0.5
  pose.orientation.x = 1
  pose.orientation.y = 0
  pose.orientation.z = 0.0
  pose.orientation.w = 0

  waypoints.poses.append(pose)

  client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)
  req = UpdateWaypointsRequest();
  
  #while not rospy.is_shutdown():
  resp = client_pose_current(request_pose);
  waypoints.poses[:] = []
  waypoints.poses.append(resp.current_pose)
  waypoints.poses.append(pose)
  req.waypoints = waypoints;
  client.call(req)
    

if __name__ == "__main__":
  main(sys.argv)
  exit(0)
