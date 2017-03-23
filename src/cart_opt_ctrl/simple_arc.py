#!/usr/bin/env python
# Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>

import sys
import copy
import time
import rospy
from geometry_msgs.msg import PoseArray, Pose
from cart_opt_ctrl.srv import UpdateWaypoints, UpdateWaypointsRequest, GetCurrentPose, GetCurrentPoseRequest, GetCurrentPoseResponse

def send_pose(pose_start,pose_end):
  waypoints = PoseArray()
  waypoints.header.frame_id = "table_link"
  pose1 = Pose()
  pose1.position.x = pose_start.position.x
  pose1.position.y = pose_start.position.y
  pose1.position.z = pose_start.position.z
  pose1.orientation.x = pose_start.orientation.x
  pose1.orientation.y = pose_start.orientation.y
  pose1.orientation.z = pose_start.orientation.z
  pose1.orientation.w = pose_start.orientation.w
  waypoints.poses.append(pose1)

  pose2 = copy.deepcopy(pose1)
  pose2.position.x = pose_end.position.x
  pose2.position.y = pose_end.position.y
  pose2.position.z = pose_end.position.z
  pose2.orientation.x = pose_end.orientation.x
  pose2.orientation.y = pose_end.orientation.y
  pose2.orientation.z = pose_end.orientation.z
  pose2.orientation.w = pose_end.orientation.w

  waypoints.poses.append(pose2)
  client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


  req = UpdateWaypointsRequest();
  req.waypoints = waypoints;

  rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
  if client.call(req):
    return True

def main(argv):
  rospy.init_node('simple_traj_script')

  #Get current pose
  client_pose_current = rospy.ServiceProxy('/CartOptCtrl/getCurrentPose', GetCurrentPose)
  request_pose = GetCurrentPoseRequest()
  resp = client_pose_current(request_pose);

  if not resp.success:
    rospy.logerror("Service call to /CartOptCtrl/getCurrentPose failed ... EXIT ")
    exit(0)

  pose = Pose()
  pose = resp.current_pose
  
  pose2 = copy.deepcopy(pose)
  pose2.position.x = 0.435
  pose2.position.y = -0.14285015
  pose2.position.z = 0.55435536

  pose3 = copy.deepcopy(pose2)
  pose3.position.x = 0.435
  pose3.position.y = -0.10751101
  pose3.position.z = 0.5511846
  
  pose4 = copy.deepcopy(pose3)
  pose4.position.x = 0.435
  pose4.position.y = -0.07185214
  pose4.position.z = 0.55607718
        
  pose5 = copy.deepcopy(pose4)
  pose5.position.x = 0.435
  pose5.position.y = -0.03597957
  pose5.position.z = 0.55901857       
	  
  pose6 = copy.deepcopy(pose5)
  pose6.position.x = 0.435
  pose6.position.y = 0.0
  pose6.position.z = 0.56
  
  pose7 = copy.deepcopy(pose6)
  pose7.position.x = 0.435
  pose7.position.y = 0.03597957
  pose7.position.z = 0.55901857
		  
  pose8 = copy.deepcopy(pose7)
  pose8.position.x = 0.435
  pose8.position.y = 0.07185214
  pose8.position.z = 0.55607718
		      
  pose9 = copy.deepcopy(pose8)
  pose9.position.x = 0.435
  pose9.position.y = 0.10751101
  pose9.position.z = 0.5511846
		      
  pose10 = copy.deepcopy(pose9)
  pose10.position.x = 0.435
  pose10.position.y = 0.14285015
  pose10.position.z = 0.55435536
			
  pose11 = copy.deepcopy(pose10)
  pose11.position.x = 0.435
  pose11.position.y = -0.14285015
  pose11.position.z = 0.55435536


  if send_pose(pose,pose11):
    rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
    
    for i in range(1,10):
      if send_pose(pose11,pose10):
	  rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
	  time.sleep(2)
	  if send_pose(pose10,pose9):
	      rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
	      
	      if send_pose(pose9,pose8):
		  rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")

		  if send_pose(pose8,pose7):
		      rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
		    
		      if send_pose(pose7,pose6):
			rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
			
			if send_pose(pose6,pose5):
			  rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")

			  if send_pose(pose5,pose4):
			    rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")

			    if send_pose(pose4,pose3):
			      rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")

			      if send_pose(pose3,pose2):
				rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
				send_pose(pose2,pose11)
				time.sleep(2)
				


  else:
    rospy.logerror("Service call to /KDLTrajCompute/updateWaypoints failed !")


if __name__ == "__main__":
  main(sys.argv)
  exit(0)