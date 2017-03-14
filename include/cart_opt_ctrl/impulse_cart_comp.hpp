#ifndef CARTOPTCTRL_IMPULSECOMP_HPP_
#define CARTOPTCTRL_IMPULSECOMP_HPP_

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <memory>
#include <rtt_ros_kdl_tools/tools.hpp>
#include <rtt_ros_kdl_tools/chain_utils.hpp>

#include <kdl/utilities/error.h>

class ImpulseComp : public RTT::TaskContext{
  public:
    ImpulseComp(const std::string& name);
    virtual ~ImpulseComp(){}

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    
  protected:
    // Input ports
    RTT::InputPort<Eigen::VectorXd> port_joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in_;
    
    // Output ports
    RTT::OutputPort<KDL::Frame> port_pnt_pos_out_;
    RTT::OutputPort<KDL::Twist> port_pnt_vel_out_;
    RTT::OutputPort<KDL::Twist> port_pnt_acc_out_;
    
    KDL::Frame current_pos_;
    KDL::Twist current_vel_, current_acc_;
    
    rtt_ros_kdl_tools::ChainUtils arm_;
    Eigen::VectorXd joint_torque_out_,
                joint_position_in_,
                joint_velocity_in_;

    bool send_;
    double amplitude_;
    std::string ee_frame_, axis_, component_;
    
    KDL::Frame start_pose_, goal_pose_;
    KDL::Twist zero_vel_, zero_acc_;
};

ORO_LIST_COMPONENT_TYPE( ImpulseComp )
#endif // CARTOPTCTRL_IMPULSECOMP_HPP_
