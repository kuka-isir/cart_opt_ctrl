#!/usr/bin/env python
# Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>

import sys
import copy
import rospy
from time import sleep
from geometry_msgs.msg import PoseArray, Pose
from cart_opt_ctrl.srv import UpdateWaypoints, UpdateWaypointsRequest, GetCurrentPose, GetCurrentPoseRequest, GetCurrentPoseResponse

def main(argv):
  rospy.init_node('square')

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

  pose1 = Pose()
  pose1.position.x = 0.4
  pose1.position.y = 0.15
  pose1.position.z = 0.400
  pose1.orientation.x = 1
  pose1.orientation.y = 0
  pose1.orientation.z = 0.0
  pose1.orientation.w = 0
  waypoints.poses.append(pose1)

  pose2 = Pose()
  pose2.position.x = 0.6
  pose2.position.y = 0.15
  pose2.position.z = 0.400
  pose2.orientation.x = 1
  pose2.orientation.y = 0
  pose2.orientation.z = 0.0
  pose2.orientation.w = 0
  waypoints.poses.append(pose2)

  pose3 = Pose()
  pose3.position.x = 0.6
  pose3.position.y = -0.2
  pose3.position.z = 0.400
  pose3.orientation.x = 1
  pose3.orientation.y = 0
  pose3.orientation.z = 0.0
  pose3.orientation.w = 0
  waypoints.poses.append(pose3)

  pose4 = Pose()
  pose4.position.x = 0.40
  pose4.position.y = -0.2
  pose4.position.z = 0.400
  pose4.orientation.x = 1
  pose4.orientation.y = 0
  pose4.orientation.z = 0.0
  pose4.orientation.w = 0
  waypoints.poses.append(pose4)

  client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)
  req = UpdateWaypointsRequest();

  resp = client_pose_current(request_pose);
  waypoints.poses[:] = []
  waypoints.poses.append(resp.current_pose)
  while True:
    waypoints.poses.append(pose1)
    waypoints.poses.append(pose2)
    waypoints.poses.append(pose3)
    waypoints.poses.append(pose4)
    waypoints.poses.append(pose1)
    req.waypoints = waypoints;
    client.call(req)
    waypoints.poses[:] = []

    #sleep(0.1);

    #resp = client_pose_current(request_pose);
    #waypoints.poses[:] = []
    #waypoints.poses.append(resp.current_pose)
    #waypoints.poses.append(pose2)
    #req.waypoints = waypoints;
    #client.call(req)

    ##sleep(0.1);

    #resp = client_pose_current(request_pose);
    #waypoints.poses[:] = []
    #waypoints.poses.append(resp.current_pose)
    #waypoints.poses.append(pose3)
    #req.waypoints = waypoints;
    #client.call(req)

    ##sleep(0.1);

    #resp = client_pose_current(request_pose);
    #waypoints.poses[:] = []
    #waypoints.poses.append(resp.current_pose)
    #waypoints.poses.append(pose4)
    #req.waypoints = waypoints;
    #client.call(req)

    #sleep(0.1);

if __name__ == "__main__":
  main(sys.argv)
  exit(0)
