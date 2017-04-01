#include "cart_opt_ctrl/button_comp.hpp"

using namespace RTT;

ButtonComp::ButtonComp(const std::string& name) : RTT::TaskContext(name)
{ 
  this->addPort("GravityOut",port_gravity_out_);
  this->addPort("GripperOut",port_gripper_out_);
  
  short_press_time_ = ros::Duration(0.1);
  long_press_time_ = ros::Duration(1.0);
}

bool ButtonComp::configureHook(){

  serial.reset(new serial::Serial);
  serial->setBaudrate(9600);
  serial->setPort("/dev/ttyACM0");
  try{
      serial->open();
  }catch(...){return false;}
  
  button_active_ = false;
  gripper_last_open_ = true;
  long_press_ = false;
  
  run_gravity_ = false;
  run_gripper_.data = false;
  
  return true;
}

bool ButtonComp::startHook(){ 
  return true;
}

void ButtonComp::updateHook(){   
  
  std::string line = serial->readline();
  
  // Data read from serial must be 0 or 1
  if ((line != "0" ) && (line != "1" ))
    return;
  
  if (line =="1"){
    if(!button_active_){
      button_active_ = true;
      button_timer_ = rtt_rosclock::host_now();
    }
    ros::Duration time_spent_active = rtt_rosclock::host_now() - button_timer_;
    if(time_spent_active > long_press_time_ && !long_press_){
      run_gripper_.data = !run_gripper_.data;
      long_press_ = true;
    }
  }
  else{
    long_press_ = false;
    if(button_active_){
      ros::Duration time_spent_active = rtt_rosclock::host_now() - button_timer_;
      if((time_spent_active < long_press_time_) && (time_spent_active > short_press_time_)){
        run_gravity_ = !run_gravity_;
      }
      button_active_ = false;
    }
  }

  port_gravity_out_.write(run_gravity_);
  port_gripper_out_.write(run_gripper_);
}

void ButtonComp::stopHook(){}
