#include "cart_opt_ctrl/scurves_traj_comp.hpp"

using namespace RTT;

SCurvesTrajComp::SCurvesTrajComp(const std::string& name) : RTT::TaskContext(name)
{ 
  this->addPort("TrajectoryPointPosOut",port_pnt_pos_out_);
  this->addPort("TrajectoryPointVelOut",port_pnt_vel_out_);
  this->addPort("TrajectoryPointAccOut",port_pnt_acc_out_);
  this->addPort("PathROSOut",port_path_out_);
  this->addPort("PathPosesROSOut",port_pose_array_out_);
  this->addPort("ButtonPressed",port_button_pressed_in_);
  this->addOperation("updateWaypoints",&SCurvesTrajComp::updateWaypoints,this,RTT::ClientThread);
  
  this->addProperty("base_frame",base_frame_).doc("Max cartesian velocity");
  this->addProperty("vel_max",vel_max_).doc("Max cartesian velocity");
  this->addProperty("acc_max",acc_max_).doc("Max cartesian acceleration");
  
  // Default params
  base_frame_ = "base_link";
  vel_max_ = 0.1;
  acc_max_ = 2.0;
  
  // Match all properties (defined in the constructor) 
  // with the rosparams in the namespace : 
  // nameOfThisComponent/nameOftheProperty
  // Equivalent to ros::param::get("CartOptCtrl/p_gains_");
  rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);
  
  tf_ = new tf::TransformListener();
  
  button_pressed_ = false;
}

bool SCurvesTrajComp::updateWaypoints(cart_opt_ctrl::UpdateWaypoints::Request& req, cart_opt_ctrl::UpdateWaypoints::Response& resp){  
  
  // Transform the waypoints to the base_frame
  geometry_msgs::PoseStamped tmp_pose_stmp;
  tmp_pose_stmp.header = req.waypoints.header;
  waypoints_in_.header = req.waypoints.header;
  waypoints_in_.poses.clear();
  waypoints_in_.poses.resize(req.waypoints.poses.size());
  try{
    for(int i=0; i<req.waypoints.poses.size(); i++){
      tmp_pose_stmp.pose = req.waypoints.poses[i];
      tf_->transformPose(base_frame_, tmp_pose_stmp, tmp_pose_stmp);
      waypoints_in_.poses[i] = tmp_pose_stmp.pose;
    }
  }catch(tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  waypoints_in_.header.frame_id = base_frame_;
  
  bool success = computeTrajectory();
  current_traj_time_ = 0.0;
  traj_computed_ = success;
  resp.success = success;
  
  while(traj_computed_){
    // Read button press port
    if(this->port_button_pressed_in_.read(button_pressed_) != RTT::NoData){
      // If gravity compensation activated, return failure on service
      if (button_pressed_){
        current_traj_time_ = 0.0;
        traj_computed_ = false;
        resp.success = false;
        return false;
      }
    }
    usleep(1e05);
  }

  return true;
}


bool SCurvesTrajComp::configureHook(){ 
  current_traj_time_ = 0.0;
  traj_computed_ = false;
  return true;
}

bool SCurvesTrajComp::startHook(){ 
  return true;
}

void SCurvesTrajComp::updateHook(){ 
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

bool SCurvesTrajComp::computeTrajectory(){  
  
  // TODO
  
    
  // Publish a displayable path to ROS
  publishTrajectory();

  return true;
}

void SCurvesTrajComp::publishTrajectory(){
  nav_msgs::Path path_ros;
  path_ros.header.frame_id = waypoints_in_.header.frame_id;
  path_ros.header.stamp = ros::Time::now();

  geometry_msgs::PoseArray pose_array;
  pose_array.header = path_ros.header;
  
  //TODO

  port_path_out_.write(path_ros);
  port_pose_array_out_.write(pose_array);
}

void SCurvesTrajComp::stopHook(){}
