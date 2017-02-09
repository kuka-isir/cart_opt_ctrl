#include "cart_opt_ctrl/compute_traj_comp.hpp"


using namespace RTT;

KDLTrajCompute::KDLTrajCompute(const std::string& name):RTT::TaskContext(name)
{ 
  this->addPort("TrajectoryPointPosOut",this->port_pnt_pos_out_);
  this->addPort("TrajectoryPointVelOut",this->port_pnt_vel_out_);
  this->addPort("TrajectoryPointAccOut",this->port_pnt_acc_out_);
  this->addPort("PathROSOut",this->port_path_out_);
  this->addPort("PathPosesROSOut",this->port_pose_array_out_);
  this->addOperation("updateWaypoints",&KDLTrajCompute::updateWaypoints,this,RTT::OwnThread);
  
  // TODO param this
  double vmax = 0.1, accmax = 2.0;
  vel_profile_ = new KDL::VelocityProfile_Trap(vmax,accmax);
  interpolator_ = new KDL::RotationalInterpolation_SingleAxis();
}

bool KDLTrajCompute::updateWaypoints(cart_opt_ctrl::UpdateWaypoints::Request& req, cart_opt_ctrl::UpdateWaypoints::Response& resp){  
  waypoints_in_ = req.waypoints;
  
  bool success = computeTrajectory();
  current_traj_time_ = 0.0;
  traj_computed_ = success;
  resp.success = success;
  
  return success;
}


bool KDLTrajCompute::configureHook()
{ 
  current_traj_time_ = 0.0;
  traj_computed_ = false;
  return true;
}

bool KDLTrajCompute::startHook()
{ 
  return true;
}

void KDLTrajCompute::updateHook()
{ 
  if (traj_computed_){
    if (current_traj_time_ < ctraject_->Duration()){
      // Get trajectory point
      current_pos_ = ctraject_->Pos(current_traj_time_);
      current_vel_ = ctraject_->Vel(current_traj_time_);
      current_acc_ = ctraject_->Acc(current_traj_time_);
      
      // Send point via ports
      port_pnt_pos_out_.write(current_pos_);
      port_pnt_vel_out_.write(current_vel_);
      port_pnt_acc_out_.write(current_acc_);
      
      // Increase timer
      current_traj_time_ += this->getPeriod();
    }
    else{
      traj_computed_ = false;
      current_traj_time_ = 0.0;
    }
  }
}

bool KDLTrajCompute::computeTrajectory(){
  // TODO param this
  double radius = 0.01, eqradius = 0.05;
  
  try {
    // Initialize path with roundness between waypoints
    path_ = new KDL::Path_RoundedComposite(radius,eqradius,interpolator_);

    // Add all the waypoints to the path
    KDL::Frame frame;
    for(int i=0; i<waypoints_in_.poses.size(); i++){
      tf::poseMsgToKDL(waypoints_in_.poses[i], frame);
      path_->Add(frame);
    }
    path_->Finish();
    
    // Set velocity profile of the trajectory
    vel_profile_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, vel_profile_);

    // Wait at the end of the trajectory
    ctraject_ = new KDL::Trajectory_Composite();
    ctraject_->Add(traject_);
    ctraject_->Add(new KDL::Trajectory_Stationary(0.5,frame));
    
    // Publish a displayable path to ROS
    publishTrajectory();
  
    } catch(KDL::Error& error) {
      std::cout <<"I encountered this error : " << error.Description() << endlog();
      std::cout << "with the following type " << error.GetType() << endlog();
      return false;
    }
  return true;
}

void KDLTrajCompute::publishTrajectory(){
  nav_msgs::Path path_ros;
  path_ros.header.frame_id = waypoints_in_.header.frame_id;
  path_ros.header.stamp = ros::Time::now();

  geometry_msgs::PoseArray pose_array;
  pose_array.header = path_ros.header;
  
  KDL::Frame current_pose;
  KDL::Twist current_vel,current_acc;
  geometry_msgs::Pose pose;
  geometry_msgs::PoseStamped pose_st;
  pose_st.header = path_ros.header;
  for (double t=0.0; t <= ctraject_->Duration(); t+= 0.1) {    
    current_pose = ctraject_->Pos(t);
    current_vel = ctraject_->Vel(t);
    current_acc = ctraject_->Acc(t);
                
    tf::poseKDLToMsg(current_pose,pose);
    pose_array.poses.push_back(pose);
    pose_st.pose = pose;
    path_ros.poses.push_back(pose_st);

  }
  port_path_out_.write(path_ros);
  port_pose_array_out_.write(pose_array);
}

void KDLTrajCompute::stopHook()
{
}
