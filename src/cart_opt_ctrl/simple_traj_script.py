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
  waypoints.header.frame_id = "table_link"

  pose = Pose()
  pose = resp.current_pose
  waypoints.poses.append(pose)

  pose2 = copy.deepcopy(pose)
  pose2.position.x = 0.435
  pose2.position.y = 0.24285015
  pose2.position.z = 0.55
  pose2.orientation.x = 0.70711
  pose2.orientation.y = 0.70711
  pose2.orientation.z = 0.0
  pose2.orientation.w = 0.0

  waypoints.poses.append(pose2)
  client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


  req = UpdateWaypointsRequest();
  req.waypoints = waypoints;

  rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
  if client.call(req):
    rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
    #Repeat 3 times the same motion
    for loop in range(0,3):
      rospy.loginfo("loop :"+str(loop))
      waypoints = PoseArray()
      waypoints.header.frame_id = "table_link"

      pose = Pose()
      pose.position.x = 0.435
      pose.position.y = 0.24285015
      pose.position.z = 0.55
      pose.orientation.x = 0.70711
      pose.orientation.y = 0.70711
      pose.orientation.z = 0.0
      pose.orientation.w = 0.0
      waypoints.poses.append(pose)

      pose2 = copy.deepcopy(pose)
      pose2.position.x = 0.435
      pose2.position.y = -0.44285015
      waypoints.poses.append(pose2)

      client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)

      req = UpdateWaypointsRequest();
      req.waypoints = waypoints;

      rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))

      if client.call(req):
	rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
	waypoints = PoseArray()
	waypoints.header.frame_id = "table_link"

	pose = Pose()
	pose.position.x = 0.435
	pose.position.y = -0.44285015
	pose.position.z = 0.55
	pose.orientation.x = 0.70711
	pose.orientation.y = 0.70711
	pose.orientation.z = 0.0
	pose.orientation.w = 0.0
	waypoints.poses.append(pose)

	pose2 = copy.deepcopy(pose)
	pose2.position.x = 0.435
	pose2.position.y = 0.24285015
	waypoints.poses.append(pose2)

	client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)

	req = UpdateWaypointsRequest();
	req.waypoints = waypoints;

	rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
	client.call(req)
      else:
	rospy.logerror("Service call to /KDLTrajCompute/updateWaypoints failed !")


if __name__ == "__main__":
  main(sys.argv)
  exit(0)
