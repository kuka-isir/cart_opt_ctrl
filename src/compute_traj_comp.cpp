#include "cart_opt_ctrl/compute_traj_comp.hpp"


using namespace RTT;

KDLTrajCompute::KDLTrajCompute(const std::string& name):RTT::TaskContext(name)
{ 
  this->addPort("StartPoseIn",this->port_start_pose_in_);
  this->addPort("TrajectoryPointPosOut",this->port_pnt_pos_out_);
  this->addPort("TrajectoryPointVelOut",this->port_pnt_vel_out_);
  this->addPort("TrajectoryPointAccOut",this->port_pnt_acc_out_);
  this->addPort("PathROSOut",this->port_path_out_);
  this->addPort("PathPosesROSOut",this->port_pose_array_out_);
  this->addOperation("publishTrajectory",&KDLTrajCompute::publishTrajectory,this,RTT::OwnThread);
  this->addOperation("computeTrajectory",&KDLTrajCompute::computeTrajectory,this,RTT::OwnThread);
    
  double vmax = 0.1, accmax = 2.0;
  vel_profile_ = new KDL::VelocityProfile_Trap(vmax,accmax);
  
  interpolator_ = new KDL::RotationalInterpolation_SingleAxis();
}

bool KDLTrajCompute::configureHook()
{ 
  current_traj_time_ = 0.0;
  new_trajectory_call_ = true;
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
    if (new_trajectory_call_){
      current_traj_time_ = 0.0;
      new_trajectory_call_ = false;
    }
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
      
      // Set booleans
      new_trajectory_call_ = false;
      traj_finished_ = false;
    }
    else{
      traj_computed_ = false;
    }
  }
  else{
    traj_computed_ = false;
  }
}

void KDLTrajCompute::computeTrajectory(){
  start_pose_in_.position.x = -0.052;
  start_pose_in_.position.y = -0.561;
  start_pose_in_.position.z = 0.420;
  start_pose_in_.orientation.x = -0.70711;
  start_pose_in_.orientation.y = 0.70711;
  start_pose_in_.orientation.z = 0.0;
  start_pose_in_.orientation.w = 0.0;

  KDL::Frame first_frame, second_frame, third_frame;
  tf::poseMsgToKDL(start_pose_in_, first_frame);
  second_frame = KDL::Frame(first_frame.M, KDL::Vector(first_frame.p.x(), first_frame.p.y() + 0.1, first_frame.p.z() + 0.1));
  third_frame = KDL::Frame(first_frame.M, KDL::Vector(first_frame.p.x(), first_frame.p.y() + 0.2, first_frame.p.z()));
  
  double radius = 0.01, eqradius = 0.05;
  path_ = new KDL::Path_RoundedComposite(radius,eqradius,interpolator_);
  path_->Add(first_frame);
  path_->Add(second_frame);
  path_->Add(third_frame);
  path_->Add(first_frame);
  path_->Finish();
  
  vel_profile_->SetProfile(0,path_->PathLength());
  traject_ = new KDL::Trajectory_Segment(path_, vel_profile_);

  ctraject_ = new KDL::Trajectory_Composite();
  ctraject_->Add(traject_);
  ctraject_->Add(new KDL::Trajectory_Stationary(1.0,first_frame));

  current_traj_time_ = 0.0;
  new_trajectory_call_ = true;
  traj_computed_ = true;
  log(RTT::Info) << "Trajectory computed!" << endlog();
  publishTrajectory();
}

void KDLTrajCompute::publishTrajectory(){
  nav_msgs::Path path_ros;
  path_ros.header.frame_id = "link_0";
  path_ros.header.stamp = ros::Time::now();

  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = path_ros.header.frame_id;
  
  for (double t=0.0; t <= ctraject_->Duration(); t+= 0.1) {
    KDL::Frame current_pose;
    KDL::Twist current_vel,current_acc;
    
    current_pose = ctraject_->Pos(t);
    current_vel = ctraject_->Vel(t);
    current_acc = ctraject_->Acc(t);
                
    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped pose_st;
    tf::poseKDLToMsg(current_pose,pose);
    pose_array.poses.push_back(pose);
    pose_st.header.frame_id = path_ros.header.frame_id;
    pose_st.pose = pose;
    path_ros.poses.push_back(pose_st);

  }
  port_path_out_.write(path_ros);
  port_pose_array_out_.write(pose_array);
}

void KDLTrajCompute::stopHook()
{
}
