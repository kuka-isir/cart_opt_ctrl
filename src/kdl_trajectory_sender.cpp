#include <ros/ros.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf_conversions/tf_kdl.h>
#include <nav_msgs/Path.h>
#include <kdl/chainiksolvervel_pinv.hpp>

boost::shared_ptr<KDL::ChainJntToJacSolver> jntToJacDotSolver_;
KDL::Frame frame_des_kdl;
KDL::FrameVel frame_vel_des_kdl;
KDL::Path_RoundedComposite* path_;
KDL::Trajectory* traject_;
KDL::Trajectory_Composite* ctraject_;
KDL::VelocityProfile* vel_profile_;

bool computeTrajectory(const double radius, const double eqradius,const double vmax, const double accmax);
void computeTrajectory(const geometry_msgs::Pose::ConstPtr& start);
void publishTrajectory();
void publishCommands();

ros::Publisher pt_pos_pub_, pt_vel_pub_, pt_acc_pub_, path_pub_, pose_array_pub_;
geometry_msgs::Pose first_frame_;

double deg2rad = 3.141592653589793/180;
bool new_trajectory_call = false, traj_finished = true, traj_computed = false;

int main(int argc, char** argv){
  ros::init(argc, argv, "kdl_trajectory_sender");
  ros::NodeHandle nh;
  
  path_pub_ = nh.advertise<nav_msgs::Path>("/KDLTrajGen/path",1);
  pose_array_pub_ = nh.advertise<geometry_msgs::PoseArray>("/KDLTrajGen/pose_array",1);
  pt_pos_pub_ = nh.advertise<geometry_msgs::Pose>("/KDLTrajGen/pos",1);
  pt_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/KDLTrajGen/vel",1);
  pt_acc_pub_ = nh.advertise<geometry_msgs::Twist>("/KDLTrajGen/acc",1);
  
  ros::Subscriber start_traj_sub = nh.subscribe<geometry_msgs::Pose>("/KDLTrajGen/compute_trajectory",1, computeTrajectory);
  
  
  int cnt = 0;
  double current_traj_time = 0.0;
  ros::Rate r(1000);
  KDL::Frame current_pose;
  KDL::Twist current_vel,current_acc;
  geometry_msgs::Pose pos_out;
  geometry_msgs::Twist vel_out, acc_out;
  while(ros::ok()){
    if(traj_finished)
      ros::spinOnce();
    if(new_trajectory_call){
      current_traj_time = 0.0;
    }
    if (traj_computed){
      if (current_traj_time < ctraject_->Duration()){
	current_pose = ctraject_->Pos(current_traj_time);
	current_vel = ctraject_->Vel(current_traj_time);
	current_acc = ctraject_->Acc(current_traj_time);
	
	tf::poseKDLToMsg(current_pose, pos_out);
	tf::twistKDLToMsg(current_vel, vel_out);
	tf::twistKDLToMsg(current_acc, acc_out);
	
	pt_pos_pub_.publish(pos_out);
	pt_vel_pub_.publish(vel_out);
	pt_acc_pub_.publish(acc_out);
	
// 	ROS_INFO_STREAM("Sending point at t="<<current_traj_time);
	current_traj_time += 0.001;
	new_trajectory_call = false;
	traj_finished = false;
      }
      else{
	traj_finished = true;
	traj_computed = false;
      }
    }
    else{
      traj_finished = true;
      traj_computed = false;
    }
    r.sleep();
  }
  ros::shutdown();
  return 1;
}


void computeTrajectory(const geometry_msgs::Pose::ConstPtr& start){
  first_frame_ = *start;
  computeTrajectory(0.01,0.05,0.1,0.2);
  new_trajectory_call = true;
}

bool computeTrajectory(const double radius, const double eqradius,const double vmax, const double accmax)
{
  ROS_INFO("Computing trajectory");
  try
  {
//     KDL::Frame first_frame = KDL::Frame(KDL::Rotation::RPY(0.*deg2rad, 0.*deg2rad, -0.*deg2rad), KDL::Vector(0.5,0,0.5));
    KDL::Frame first_frame, second_frame, third_frame;
    first_frame.p.x();
    tf::poseMsgToKDL(first_frame_, first_frame);
    second_frame = KDL::Frame(first_frame.M, KDL::Vector(first_frame.p.x(), first_frame.p.y() + 0.1, first_frame.p.z() + 0.1));
    third_frame = KDL::Frame(first_frame.M, KDL::Vector(first_frame.p.x(), first_frame.p.y() + 0.2, first_frame.p.z()));
    
    path_ = new KDL::Path_RoundedComposite(radius,eqradius,new KDL::RotationalInterpolation_SingleAxis());
    path_->Add(first_frame);
    path_->Add(second_frame);
    path_->Add(third_frame);
//     path_->Add(KDL::Frame(KDL::Rotation::RPY(0.*deg2rad, -0.*deg2rad, -0.*deg2rad), KDL::Vector(0.5,0.1,.6)));
//     path_->Add(KDL::Frame(KDL::Rotation::RPY(0.*deg2rad, -0.*deg2rad, -0.*deg2rad), KDL::Vector(0.5,0.2,.5)));
    path_->Add(first_frame);
    path_->Finish();

    vel_profile_ = new KDL::VelocityProfile_Trap(vmax,accmax);
    vel_profile_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, vel_profile_);

    ctraject_ = new KDL::Trajectory_Composite();
    ctraject_->Add(traject_);
    ctraject_->Add(new KDL::Trajectory_Stationary(1.0,first_frame));

  } catch(...) {
      ROS_ERROR("Encountered an error while computing KDL trajectory");
      return false;
  }
  publishTrajectory();
  traj_computed = true;
  return true;
}

void publishTrajectory()
{
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
  path_pub_.publish(path_ros);
  pose_array_pub_.publish(pose_array);
}
