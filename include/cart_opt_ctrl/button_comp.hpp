#ifndef CARTOPTCTRL_BUTTONCOMP_HPP_
#define CARTOPTCTRL_BUTTONCOMP_HPP_

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <memory>

#include "serial/serial.h"

#include <std_msgs/Bool.h>

class ButtonComp : public RTT::TaskContext{
  public:
    ButtonComp(const std::string& name);
    virtual ~ButtonComp(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    
    
  protected:
    // Output ports
    RTT::OutputPort<bool> port_gravity_out_; 
    RTT::OutputPort<std_msgs::Bool> port_gripper_out_;
    
    bool button_active_, gripper_last_open_, long_press_;
    ros::Duration short_press_time_, long_press_time_;
    ros::Time button_timer_;
    std::unique_ptr<serial::Serial> serial;
    
    bool run_gravity_;
    std_msgs::Bool run_gripper_;
};

ORO_LIST_COMPONENT_TYPE( ButtonComp )
#endif // CARTOPTCTRL_BUTTONCOMP_HPP_