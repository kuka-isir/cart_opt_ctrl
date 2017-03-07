#!/usr/bin/env python
# Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>

import sys
import copy
import rospy
from geometry_msgs.msg import PoseArray, Pose
from cart_opt_ctrl.srv import UpdateWaypoints, UpdateWaypointsRequest

def main(argv):
  rospy.init_node('simple_traj_script')
  
  waypoints = PoseArray()
  waypoints.header.frame_id = "base_link"
  
  pose = Pose()
  pose.position.x = 0.4
  pose.position.y = 0.0
  pose.position.z = 0.4
  pose.orientation.x = 0.70711
  pose.orientation.y = 0.70711
  pose.orientation.z = 0.0
  pose.orientation.w = 0.0
  waypoints.poses.append(pose)
    
  pose2 = copy.deepcopy(pose)
  pose2.position.x += 0.3
  pose2.position.y += 0.3
  waypoints.poses.append(pose2)
  
  pose3 = copy.deepcopy(pose2)
  pose3.position.x -= 0.4
  pose3.position.y -= 0.5
  pose3.position.z -= 0.1
  waypoints.poses.append(pose3)
  
  client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)
  
  req = UpdateWaypointsRequest();
  req.waypoints = waypoints;
  
  rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
  
  if client.call(req):
    rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
  else:
    rospy.logerror("Service call to /KDLTrajCompute/updateWaypoints failed !")  
  

if __name__ == "__main__":
  main(sys.argv)
  exit(0)
