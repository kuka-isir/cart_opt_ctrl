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
  this->addPort("CurrentPose",port_x_curr_);
  this->addPort("CurrentVel",port_xd_curr_);
  this->addPort("CurrentAcc",port_xdd_curr_);
  this->addPort("DebugPose",port_debug_pos_out_);
  this->addPort("DebugVel",port_debug_vel_out_);
  this->addPort("DebugAcc",port_debug_acc_out_);
  this->addOperation("updateWaypoints",&SCurvesTrajComp::updateWaypoints,this,RTT::ClientThread);
  
  this->addProperty("base_frame",base_frame_).doc("Max cartesian velocity");
  this->addProperty("vel_max",vel_max_).doc("Max cartesian velocity");
  this->addProperty("acc_max",acc_max_).doc("Max cartesian acceleration");
  
  // Default params
  base_frame_ = "base_link";
  vel_max_ = 0.5;
  acc_max_ = 2.0;
  
  // Match all properties (defined in the constructor) 
  // with the rosparams in the namespace : 
  // nameOfThisComponent/nameOftheProperty
  // Equivalent to ros::param::get("CartOptCtrl/p_gains_");
  rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);
  
  tf_ = new tf::TransformListener();
  
  button_pressed_ = false;
  
  scurve_profiler_ = new SCurveProfile();
  scurve_profiler_->set_period(0.001);
}

bool SCurvesTrajComp::updateWaypoints(cart_opt_ctrl::UpdateWaypoints::Request& req, cart_opt_ctrl::UpdateWaypoints::Response& resp){
  
  save_acc_ = 0;
  save_vel_ = 0;
  
  
  // Get start pose from the controller
  port_x_curr_.read(start_pose_);
  
  
  
  save_pose_ = 0;
  
  
  
  // Transform the goal to the base_frame
  geometry_msgs::PoseStamped tmp_pose_stmp;
  tmp_pose_stmp.header = req.waypoints.header;
  try{
    if(req.waypoints.poses.size()>0){
      tmp_pose_stmp.pose = req.waypoints.poses[req.waypoints.poses.size()-1];
      tf_->transformPose(base_frame_, tmp_pose_stmp, tmp_pose_stmp);
      goal_pose_ = tmp_pose_stmp;
    }
    else{
      ROS_ERROR("Waypoints is an empty list");
      return false;
    }
  }catch(tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  bool success = computeTrajectory();
  traj_computed_ = success;
  resp.success = success;
  
  while(traj_computed_){
    // Read button press port
    if(this->port_button_pressed_in_.read(button_pressed_) != RTT::NoData){
      // If gravity compensation activated, return failure on service
      if (button_pressed_){
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
  traj_computed_ = false;
  return true;
}

bool SCurvesTrajComp::startHook(){ 
  return true;
}

void SCurvesTrajComp::updateHook(){ 
  if (traj_computed_){
    
    port_x_curr_.read(curr_pose_);
    port_xd_curr_.read(curr_vel_);
    port_xdd_curr_.read(curr_acc_);
    
    tf::twistMsgToKDL(curr_vel_, current_vel_);
    tf::twistMsgToKDL(curr_acc_, current_acc_);
    
//     double distance = std::sqrt(
//       std::pow(goal_pose_.pose.position.x-curr_pose_.pose.position.x,2)
//       + std::pow(goal_pose_.pose.position.y-curr_pose_.pose.position.y,2)
//       + std::pow(goal_pose_.pose.position.z-curr_pose_.pose.position.z,2));
    
    // TODO   
//     double distance = std::sqrt(std::pow(goal_pose_.pose.position.x-curr_pose_.pose.position.x,2));
    double distance = 0.1;
//     double current_speed = current_vel_.vel.x();
    double current_speed = save_vel_;
//     double current_acc = current_acc_.vel.x();
    double current_acc = save_acc_;
//     log(RTT::Error) << "Distance to goal : " << distance << endlog();
//     log(RTT::Error) << "Speed : " << current_speed << endlog();
//     log(RTT::Error) << "Acc : " << current_acc << endlog();
    
    if (save_pose_ <= distance){  
      // Update profile
      scurve_profiler_->config(save_pose_,current_speed,current_acc,distance,0,0,0.5, 1, 20);
      scurve_profiler_->compute_curves();
      
      // Compute next pose
      //TODO
      tf::poseMsgToKDL(curr_pose_.pose, current_pos_);
      current_pos_.p.data[0] = start_pose_.pose.position.x + save_pose_;
      log(RTT::Error) << "Pose : " << save_pose_ << endlog();
      save_pose_ = save_pose_ * 0.95 + scurve_profiler_->s_vect_[1]*0.05;
      
      // Compute next speed
      //TODO
      current_vel_.vel.data[0] = scurve_profiler_->v_vect_[1];
      log(RTT::Error) << "Vel : " << current_vel_.vel.data[0] << endlog();
      save_vel_ = save_vel_*0.95 + scurve_profiler_->v_vect_[1]*0.05;
      
      // Compute next acceleration
      //TODO
      current_acc_.vel.data[0] = scurve_profiler_->a_vect_[1];
      save_acc_ = save_acc_ *0.95+ scurve_profiler_->a_vect_[1]*0.05;
      
      log(RTT::Error) << "Acc : " << current_acc_.vel.data[0] << endlog();
      log(RTT::Error) << "**************" << endlog();
      
      geometry_msgs::Twist debug_vel, debug_acc;
      geometry_msgs::Pose debug_pose;
      tf::poseKDLToMsg(current_pos_, debug_pose);
      port_debug_pos_out_.write(debug_pose);
      tf::twistKDLToMsg(current_vel_, debug_vel);
      port_debug_vel_out_.write(debug_vel);
      tf::twistKDLToMsg(current_acc_, debug_acc);
      port_debug_acc_out_.write(debug_acc);
      
      // Send point via ports
      port_pnt_pos_out_.write(current_pos_);
      port_pnt_vel_out_.write(current_vel_);
      port_pnt_acc_out_.write(current_acc_);
    }
    else{
      traj_computed_ = false;
    }
  }
}

bool SCurvesTrajComp::computeTrajectory(){  
    
  // Publish a displayable path to ROS
  publishTrajectory();

  return true;
}

void SCurvesTrajComp::publishTrajectory(){
  // Publish Path
  nav_msgs::Path path_ros;
  path_ros.header.frame_id = goal_pose_.header.frame_id;
  path_ros.header.stamp = ros::Time::now();
  path_ros.poses.push_back(start_pose_);
  path_ros.poses.push_back(goal_pose_);
  port_path_out_.write(path_ros);
  
  // Publish PoseArray
  geometry_msgs::PoseArray pose_array;
  pose_array.header = path_ros.header;
  pose_array.poses.push_back(start_pose_.pose);
  pose_array.poses.push_back(goal_pose_.pose);
  port_pose_array_out_.write(pose_array);
}

void SCurvesTrajComp::stopHook(){}
