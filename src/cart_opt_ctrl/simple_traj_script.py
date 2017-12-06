#!/usr/bin/env python
# Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>

import sys
import copy
import rospy
import tf
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
  waypoints.header.frame_id = rospy.get_param("root_link")

  waypoints.poses.append(resp.current_pose)

  pose = Pose()
  pose.position.x = 0.4
  pose.position.y = 0.0
  pose.position.z = 0.4

  roll = -3.1416
  pitch = 0.0
  yaw = 0.0
  quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
  pose.orientation.x = quaternion[0]
  pose.orientation.y = quaternion[1]
  pose.orientation.z = quaternion[2]
  pose.orientation.w = quaternion[3]

  waypoints.poses.append(pose)

  client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


  req = UpdateWaypointsRequest();
  req.waypoints = waypoints;

  client.call(req)


if __name__ == "__main__":
  main(sys.argv)
  exit(0)
