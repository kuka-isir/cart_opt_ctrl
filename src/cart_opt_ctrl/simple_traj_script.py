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
  pose2.position.y = -0.14285015
  pose2.position.z = 0.64435536

  waypoints.poses.append(pose2)

  client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


  req = UpdateWaypointsRequest();
  req.waypoints = waypoints;

  rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
  if client.call(req):
    rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
    waypoints = PoseArray()
    waypoints.header.frame_id = "table_link"

    waypoints.poses.append(pose2)

    pose3 = copy.deepcopy(pose2)
    pose3.position.x = 0.435
    pose3.position.y = -0.10751101
    pose3.position.z = 0.6511846

    waypoints.poses.append(pose3)
    
    client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


    req = UpdateWaypointsRequest();
    req.waypoints = waypoints;

    rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
    if client.call(req):
        rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
        waypoints = PoseArray()
        waypoints.header.frame_id = "table_link"
        waypoints.poses.append(pose3)

        pose4 = copy.deepcopy(pose3)
        pose4.position.x = 0.435
        pose4.position.y = -0.07185214
        pose4.position.z = 0.65607718

        waypoints.poses.append(pose4)
        client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


        req = UpdateWaypointsRequest();
        req.waypoints = waypoints;

        rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
        if client.call(req):
            rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
            waypoints = PoseArray()
            waypoints.header.frame_id = "table_link"

            waypoints.poses.append(pose4)

            pose5 = copy.deepcopy(pose4)
            pose5.position.x = 0.435
            pose5.position.y = -0.03597957
            pose5.position.z = 0.65901857

            waypoints.poses.append(pose5)
            client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


            req = UpdateWaypointsRequest();
            req.waypoints = waypoints;

            rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
            if client.call(req):
                rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
                waypoints = PoseArray()
                waypoints.header.frame_id = "table_link"
                
                waypoints.poses.append(pose5)

                pose6 = copy.deepcopy(pose5)
                pose6.position.x = 0.435
                pose6.position.y = 0.0
                pose6.position.z = 0.66

                waypoints.poses.append(pose6)
                client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


                req = UpdateWaypointsRequest();
                req.waypoints = waypoints;

                rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
                if client.call(req):
                    rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
                    waypoints = PoseArray()
                    waypoints.header.frame_id = "table_link"
                    
                    waypoints.poses.append(pose6)

                    pose7 = copy.deepcopy(pose6)
                    pose7.position.x = 0.435
                    pose7.position.y = 0.03597957
                    pose7.position.z = 0.65901857

                    waypoints.poses.append(pose7)
                    client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


                    req = UpdateWaypointsRequest();
                    req.waypoints = waypoints;

                    rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
                    if client.call(req):
		      rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
		      waypoints = PoseArray()
		      waypoints.header.frame_id = "table_link"
		      
		      waypoints.poses.append(pose7)

		      pose8 = copy.deepcopy(pose7)
		      pose8.position.x = 0.435
		      pose8.position.y = 0.07185214
		      pose8.position.z = 0.65607718

		      waypoints.poses.append(pose8)
		      client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


		      req = UpdateWaypointsRequest();
		      req.waypoints = waypoints;

		      rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
		      if client.call(req):
			rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
			waypoints = PoseArray()
			waypoints.header.frame_id = "table_link"
			
			waypoints.poses.append(pose8)

			pose9 = copy.deepcopy(pose8)
			pose9.position.x = 0.435
			pose9.position.y = 0.10751101
			pose9.position.z = 0.6511846

			waypoints.poses.append(pose9)
			client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


			req = UpdateWaypointsRequest();
			req.waypoints = waypoints;

			rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
			if client.call(req):
			  rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
			  waypoints = PoseArray()
			  waypoints.header.frame_id = "table_link"
			  
			  waypoints.poses.append(pose9)

			  pose10 = copy.deepcopy(pose9)
			  pose10.position.x = 0.435
			  pose10.position.y = 0.14285015
			  pose10.position.z = 0.64435536

			  waypoints.poses.append(pose10)
			  client = rospy.ServiceProxy('/KDLTrajCompute/updateWaypoints', UpdateWaypoints)


			  req = UpdateWaypointsRequest();
			  req.waypoints = waypoints;

			  rospy.loginfo("Calling /KDLTrajCompute/updateWaypoints service with the following waypoints :\n" + str(waypoints))
			  if client.call(req):
			    rospy.loginfo("Service call to /KDLTrajCompute/updateWaypoints succeedeed !")
			    waypoints = PoseArray()
			    waypoints.header.frame_id = "table_link"
			    
			    waypoints.poses.append(pose10)

			    pose11 = copy.deepcopy(pose10)
			    pose11.position.x = 0.435
			    pose11.position.y = -0.14285015
			    pose11.position.z = 0.64435536

			    waypoints.poses.append(pose11)
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