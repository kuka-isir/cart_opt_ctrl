#include "cart_opt_ctrl/cart_opt_ctrl.hpp"
#include "eigen_conversions/eigen_kdl.h"


using namespace RTT;
using namespace KDL;

CartOptCtrl::CartOptCtrl(const std::string& name):RTT::TaskContext(name)
{
    this->addPort("JointPosition",this->port_joint_position_in);
    this->addPort("JointVelocity",this->port_joint_velocity_in);
    this->addPort("JointTorqueCommand",this->port_joint_torque_out);
    this->addPort("TrajectoryPointIn",this->port_traj_in);
    
    this->addProperty("P_gain",this->P_gain).doc("Proportional gain");
    this->addProperty("D_gain",this->D_gain).doc("Derivative gain");
}

bool CartOptCtrl::configureHook()
{
    // Initialise the model, the internal solvers etc
    if( ! this->arm.init() )
    {
        log(RTT::Error) << "Could not init chain utils !" << endlog();
        return false;
    }
    // The number of joints
    const int dof  = this->arm.getNrOfJoints();
    
    // Not using Matrix<double,6,1> becquse of ops script limitations
    this->P_gain.resize(6);
    this->D_gain.resize(6);
    
    // Default gains, works but stiff
    this->P_gain << 1000,1000,1000,300,300,300;
    this->D_gain << 50,50,50,10,10,10;
    
    // Match all properties (defined in the constructor) 
    // with the rosparams in the namespace : 
    // nameOfThisComponent/nameOftheProperty
    // Equivalent to ros::param::get("CartOptCtrl/P_gain");
    rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);
    
    // Let's use the last segment
    this->ee_frame = arm.getSegmentName( arm.getNrOfSegments() - 1 );
    
    // For now we have 0 constraints for now
    int number_of_constraints = 0;
    this->qpoases_solver.reset(new qpOASES::SQProblem(dof,number_of_constraints));
    
    // Resize and set torque at zero
    this->joint_torque_out.setZero(dof);
    this->joint_position_in.setZero(dof);
    this->joint_velocity_in.setZero(dof);
    
    
    // QPOases options
    qpOASES::Options options;
    // This options enables regularisation (required) and disable
    // some checks to be very fast !
    options.setToMPC();
    this->qpoases_solver->setOptions(options);
    // Remove verbosity
    this->qpoases_solver->setPrintLevel(qpOASES::PL_NONE/*qpOASES::PL_DEBUG*/);
    
    return true;
}

bool CartOptCtrl::startHook()
{
    this->has_first_command = false;
}



void CartOptCtrl::updateHook()
{  
   
    // Read the current state of the robot
    RTT::FlowStatus fp = this->port_joint_position_in.read(this->joint_position_in);
    RTT::FlowStatus fv = this->port_joint_velocity_in.read(this->joint_velocity_in);
    
    // Return if not giving anything (might happend during startup)
    if(fp == RTT::NoData || fv == RTT::NoData)
    {
        return;
    }
    
    // Feed the internal model
    arm.setState(this->joint_position_in,this->joint_velocity_in);
    // Make some calculations
    arm.updateModel();
    
    // Get Current end effector Pose
    X_curr = arm.getSegmentPosition(this->ee_frame);

    KDL::SetToZero(Xd_traj);
    KDL::SetToZero(Xdd_traj);  
    
    if(this->port_traj_in.read(this->traj_pt_in) != RTT::NoData)
    {
        
        X_traj = this->traj_pt_in.GetFrame();
        Xd_traj = this->traj_pt_in.GetTwist();
        Xdd_traj = this->traj_pt_in.GetAccTwist();
        
        has_first_command = true;
    }
    
    
    // First step, initialise the first X,Xd,Xdd desired
    if(!has_first_command)
    {
        
        X_traj = X_curr;
        
        has_first_command = true;
    }
    
    X_err = diff( X_curr , X_traj );
    Xd_err = diff( Xd_curr , Xd_traj);
    

    KDL::Twist kp_,kd_;
    
    tf::twistEigenToKDL(this->P_gain,kp_);
    tf::twistEigenToKDL(this->D_gain,kd_);
    
    Xdd_des.vel = Xdd_traj.vel + kp_.vel * ( X_err.vel ) + kd_.vel * ( Xd_err.vel );
    Xdd_des.rot = Xdd_traj.rot + kp_.rot * ( X_err.rot ) + kd_.rot * ( Xd_err.rot );
    
    Eigen::Matrix<double,6,1> xdd_des;
    tf::twistKDLToEigen(Xdd_des,xdd_des);
    
    KDL::Jacobian& J = arm.getSegmentJacobian(this->ee_frame);
    
    KDL::JntSpaceInertiaMatrix& M_inv = arm.getInertiaInverseMatrix();
    
    KDL::JntArray& coriolis = arm.getCoriolisTorque();
    
    KDL::JntArray& gravity = arm.getGravityTorque();
    
    KDL::Twist& Jdotqdot = arm.getSegmentJdotQdot(this->ee_frame);
    
    Eigen::Matrix<double,6,1> jdot_qdot;
    tf::twistKDLToEigen(Jdotqdot,jdot_qdot);
    
    // We put it in the form ax + b
    Eigen::MatrixXd a;
    a.resize(6,arm.getNrOfJoints());
    
    Eigen::Matrix<double,6,1> b;
    
    a = - J.data * M_inv.data;
    b = - a * ( coriolis.data + gravity.data ) + xdd_des - jdot_qdot;
    
    // Matrices for qpOASES
    Eigen::MatrixXd H;
    H.resize(arm.getNrOfJoints(),arm.getNrOfJoints());
    
    Eigen::VectorXd g;
    g.resize(arm.getNrOfJoints());
    
    H = 2 * a.transpose() * a;
    g = 2 * a.transpose() * b;
    
    // TODO: get this from URDF
    Eigen::VectorXd torque_max;
    torque_max.resize(arm.getNrOfJoints());
    torque_max.setConstant(100); // N.m

    Eigen::VectorXd torque_min;
    torque_min.resize(arm.getNrOfJoints());
    torque_min = -torque_max; // N.m
    
    // Compute bounds
    Eigen::VectorXd lb,ub;
    lb.resize(arm.getNrOfJoints());
    ub.resize(arm.getNrOfJoints());
    
    //TODO : write constraints for q and qdot
    lb = torque_min;
    ub = torque_max;
    
    // number of allowed compute steps
    int nWSR = 1000; 
    
    // Let's compute !
    qpOASES::returnValue ret;
    static bool qpoases_initialized = false;
    
    if(!qpoases_initialized)
    {
        // Initialise the problem, once it has found a solution, we can hotstart
        ret = qpoases_solver->init(H.data(),g.data(),NULL,lb.data(),ub.data(),NULL,NULL,nWSR);
        if(ret == qpOASES::SUCCESSFUL_RETURN)
        {
            qpoases_initialized = true;
        }
    }
    else
    {
        ret = qpoases_solver->hotstart(H.data(),g.data(),NULL,lb.data(),ub.data(),NULL,NULL,nWSR);
    }
    
    
    if(ret == qpOASES::SUCCESSFUL_RETURN)
    {
        // Get the solution
        qpoases_solver->getPrimalSolution(this->joint_torque_out.data());
        // Remove gravity because Kuka already adds it
        this->joint_torque_out -= arm.getGravityTorque().data;
        // Send torques to the robot
        this->port_joint_torque_out.write(this->joint_torque_out);
    }

}


void CartOptCtrl::stopHook()
{
    this->has_first_command = false;
}

