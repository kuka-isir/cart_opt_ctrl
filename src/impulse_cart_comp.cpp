#include "cart_opt_ctrl/impulse_cart_comp.hpp"

using namespace RTT;

ImpulseComp::ImpulseComp(const std::string& name) : RTT::TaskContext(name)
{ 
  this->addPort("JointPosition",port_joint_position_in_);
  this->addPort("JointVelocity",port_joint_velocity_in_);
  this->addPort("TrajectoryPointPosOut",port_pnt_pos_out_);
  this->addPort("TrajectoryPointVelOut",port_pnt_vel_out_);
  this->addPort("TrajectoryPointAccOut",port_pnt_acc_out_);
  
  this->addProperty("axis",axis_).doc("Choose axis for impulse");
  this->addProperty("component",component_).doc("Choose between rot or lin impulse");
  this->addProperty("send_impulse",send_).doc("Send the impulse");
  this->addProperty("amplitude",amplitude_).doc("Send the impulse");
  
  // Match all properties (defined in the constructor) 
  // with the rosparams in the namespace : 
  // nameOfThisComponent/nameOftheProperty
  // Equivalent to ros::param::get("CartOptCtrl/p_gains_");
  rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);
}

bool ImpulseComp::configureHook(){ 
  
  send_ = false;
  axis_ = "x";
  component_ = "lin";
  amplitude_ = 0.01;
  
  // Initialise the model, the internal solvers etc
  if( ! arm_.init() ){
    log(RTT::Error) << "Could not init chain utils !" << endlog();
    return false;
  }
  // The number of joints
  const int dof = arm_.getNrOfJoints();
  
  // Resize the vectors
  joint_torque_out_.setZero(dof);
  joint_position_in_.setZero(dof);
  joint_velocity_in_.setZero(dof);
  
  zero_vel_ = KDL::Twist();
  zero_acc_ = KDL::Twist();
  
  // Default params
  ee_frame_ = arm_.getSegmentName( arm_.getNrOfSegments() - 1 );
  return true;
}

bool ImpulseComp::startHook(){ 
  return true;
}

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

void ImpulseComp::updateHook(){ 

  if (send_){
    // Read the current state of the robot
    RTT::FlowStatus fp = this->port_joint_position_in_.read(this->joint_position_in_);
    RTT::FlowStatus fv = this->port_joint_velocity_in_.read(this->joint_velocity_in_);
    
    // Return if not giving anything (might happend during startup)
    if(fp == RTT::NoData || fv == RTT::NoData){
      log(RTT::Error) << "Robot ports empty !" << endlog();
      return;
    }
    
    // Feed the internal model
    arm_.setState(this->joint_position_in_,this->joint_velocity_in_);
    // Make some calculations
    arm_.updateModel();
    
    // Get Current end effector Pose
    start_pose_ = arm_.getSegmentPosition(ee_frame_);

    goal_pose_ = start_pose_;
    
    if(component_ == "lin"){      
      switch (str2int(axis_.c_str())){
        case str2int("x"):
          start_pose_.p.x(start_pose_.p.x() + amplitude_);
          break;
        case str2int("y"):
          start_pose_.p.y(start_pose_.p.y() + amplitude_);
          break;
        case str2int("z"):
          start_pose_.p.z(start_pose_.p.z() + amplitude_);
          break;
        default:
          ROS_ERROR("Choose between x, y ,z in axis");
          send_ = false;
          return;
      }         
    }else {
      if(component_ == "rot"){      
        switch (str2int(axis_.c_str())){
          case str2int("x"):
            start_pose_.M.DoRotX(amplitude_);
            break;
          case str2int("y"):
            start_pose_.M.DoRotY(amplitude_);
            break;
          case str2int("z"):
            start_pose_.M.DoRotZ(amplitude_);
            break;
          default:
            ROS_ERROR("Choose between x, y ,z in axis");
            send_ = false;
            return;
        }
      }
      else{
        ROS_ERROR("Choose between lin and rot for component");
        send_ = false;
        return;
      }
    }
        
    // Send point via ports
    port_pnt_pos_out_.write(start_pose_);
    port_pnt_vel_out_.write(zero_vel_);
    port_pnt_acc_out_.write(zero_acc_);
    
    send_ = false;
  }
  
}

void ImpulseComp::stopHook(){}
