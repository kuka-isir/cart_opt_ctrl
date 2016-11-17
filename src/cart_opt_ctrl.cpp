#include "cart_opt_ctrl/cart_opt_ctrl.hpp"
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames_io.hpp>
#include <kdl_conversions/kdl_msg.h>

using namespace RTT;
using namespace KDL;

CartOptCtrl::CartOptCtrl(const std::string& name):RTT::TaskContext(name)
{
    this->addPort("JointPosition",this->port_joint_position_in);
    this->addPort("JointVelocity",this->port_joint_velocity_in);
    this->addPort("JointTorqueCommand",this->port_joint_torque_out);
    this->addPort("TrajectoryPointIn",this->port_traj_point_in);
    this->addPort("TrajectoryJointIn",this->port_traj_joint_in);
    this->addPort("PoseDesired",this->port_x_des);
    
    this->addProperty("FrameOfInterest",this->ee_frame).doc("The robot frame to track the trajectory");
    this->addProperty("P_gain",this->P_gain).doc("Proportional gain");
    this->addProperty("D_gain",this->D_gain).doc("Derivative gain");
    this->addProperty("P_joint_gain",this->P_joint_gain).doc("Proportional gain");
    this->addProperty("D_joint_gain",this->D_joint_gain).doc("Derivative gain");
    this->addProperty("Alpha",this->Alpha).doc("weight for joint goal in QP");
    this->addProperty("Regularisation",this->Regularisation).doc("weight for the regularisation in QP");
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
    
    // Not using Matrix<double,6,1> because of ops script limitations
    this->P_gain.resize(6);
    this->D_gain.resize(6);
    this->P_joint_gain.resize(dof);
    this->D_joint_gain.resize(dof);
    this->qdd_des.resize(dof);
    this->q_traj.resize(dof);
    this->qd_traj.resize(dof);
    this->qdd_traj.resize(dof);
    this->q_curr.resize(dof);
    this->qd_curr.resize(dof);
    this->qdd_curr.resize(dof);
    
    // Let's use the last segment to track by default
    this->ee_frame = arm.getSegmentName( arm.getNrOfSegments() - 1 );
    
    // Default gains, works but stiff
    this->P_gain << 1000,1000,1000,300,300,300;
    this->D_gain << 50,50,50,10,10,10;
    this->P_joint_gain << 450.0, 450.0, 80.0, 450.0, 80.0, 20.0, 1.0;
    this->D_joint_gain << 20.0, 20.0, 1.5, 20.0, 1.5, 1.0, 0.05;
    
    // Default Alpha value
    this->Alpha = 1e-03;
    this->Regularisation = 1e-05;
    
    // Match all properties (defined in the constructor) 
    // with the rosparams in the namespace : 
    // nameOfThisComponent/nameOftheProperty
    // Equivalent to ros::param::get("CartOptCtrl/P_gain");
    rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);
    
    
    // For now we have 0 constraints for now
    int number_of_variables = dof;
    int number_of_constraints = dof;
    this->qpoases_solver.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints,qpOASES::HST_POSDEF));
    
    // Resize and set torque at zero
    this->joint_torque_out.setZero(dof);
    this->joint_position_in.setZero(dof);
    this->joint_velocity_in.setZero(dof);
    
    // QPOases options
    qpOASES::Options options;
    // This options enables regularisation (required) and disable
    // some checks to be very fast !
    // options.setToDefault();
    options.setToMPC(); // setToReliable() // setToDefault()
    options.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
    this->qpoases_solver->setOptions( options );
    this->qpoases_solver->setPrintLevel(qpOASES::PL_NONE); // PL_HIGH for full output, PL_NONE for... none
    
    return true;
}

bool CartOptCtrl::startHook()
{
    this->has_first_command = false;
    return true;
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
    Xd_curr = arm.getSegmentVelocity(this->ee_frame);
    q_curr = arm.getJointPositions();
    qd_curr = arm.getJointVelocities();

    // Initialize the desired velocity and acceleration to zero
    KDL::SetToZero(Xd_traj);
    KDL::SetToZero(Xdd_traj);
    KDL::SetToZero(qd_traj);
    KDL::SetToZero(qdd_traj);
    
    // If we get a new trajectory point to track
    if((this->port_traj_point_in.read(this->traj_pt_in) != RTT::NoData) && (this->port_traj_joint_in.read(this->traj_joint_in) != RTT::NoData) )
    {
        // Then overrride the desired
        X_traj = this->traj_pt_in.GetFrame();
        Xd_traj = this->traj_pt_in.GetTwist();
        Xdd_traj = this->traj_pt_in.GetAccTwist();
        q_traj = this->traj_joint_in.q;
        qd_traj = this->traj_joint_in.qdot;
        qdd_traj = this->traj_joint_in.qdotdot;
        
        has_first_command = true;
    }
    
    // First step, initialise the first X,Xd,Xdd desired
    if(!has_first_command)
    {
        // Stay at the same position
        X_traj = X_curr;
        q_traj = q_curr;
        
        has_first_command = true;
    }
    
    // Debug publish in ROS
    geometry_msgs::Pose x_des_pos_out;
    tf::poseKDLToMsg(X_traj,x_des_pos_out);
    this->port_x_des.write(x_des_pos_out);
    
    
    // Compute errors
    X_err = diff( X_curr , X_traj );
    Xd_err = diff( Xd_curr , Xd_traj);
    
    
    // Apply PD 
    for( unsigned int i=0; i<6; ++i )
    {
        Xdd_des(i) = Xdd_traj(i) + this->P_gain(i) * ( X_err(i) ) + this->D_gain(i) * ( Xd_err(i) );
    }
    for(unsigned int i=0; i< this->arm.getNrOfJoints(); ++i){
        qdd_des(i) = qdd_traj(i) + this->P_joint_gain(i) * ( q_traj(i) - q_curr(i) ) + this->D_joint_gain(i) * ( qd_traj(i) - qd_curr(i) );
    }    
    
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
    // M(q).qdd + B(qd) + G(q) = T
    // --> qdd = Minv.( T - B - G)
    // Xd = J.qd
    // --> Xdd = Jdot.qdot + J.qdd
    // --> Xdd = Jdot.qdot + J.Minv.( T - B - G)
    // --> Xdd = Jdot.qdot + J.Minv.T - J.Minv.( B + G )
    // And we have Xdd_des = Xdd_traj + P_gain.( X_des - X_curr) + D_gain.( Xd_des - Xd_curr)
    // ==> We want to compute min(T) || Xdd - Xdd_des ||²
    // If with replace, we can put it in the form ax + b
    // With a = J.Minv
    //      b = - J.Minv.( B + G ) + Jdot.qdot - Xdd_des

    Eigen::Matrix<double,6,Eigen::Dynamic> a;
    a.resize(6,arm.getNrOfJoints());
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> a2;
    a2.resize(arm.getNrOfJoints(),arm.getNrOfJoints());
    
    Eigen::Matrix<double,6,1> b;
    Eigen::Matrix<double,Eigen::Dynamic,1> b2;
    b2.resize(arm.getNrOfJoints(),1);
    
    a.noalias() = J.data * M_inv.data;
    a2.noalias() = M_inv.data;
    b.noalias() = - a * ( coriolis.data + gravity.data ) + jdot_qdot - xdd_des;
    b2.noalias() = - a2 * ( coriolis.data + gravity.data ) - qdd_des;
    
    // Matrices for qpOASES
    // NOTE: We need RowMajor (see qpoases doc)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
    H.resize(arm.getNrOfJoints(),arm.getNrOfJoints());
    
    Eigen::VectorXd g(arm.getNrOfJoints());
    
    Eigen::MatrixXd regularisation;
    regularisation.resize( arm.getNrOfJoints(), arm.getNrOfJoints() );
    regularisation.setIdentity();
    regularisation *= this->Regularisation;
    
    H = 2.0 * a.transpose() * a + this->Alpha* 2.0 * a2.transpose() * a2 + regularisation;
    g = 2.0 * a.transpose() * b + this->Alpha* 2.0 * a2.transpose() * b2;
    
    // TODO: get this from URDF
    Eigen::VectorXd torque_max;
    torque_max.resize(arm.getNrOfJoints());
    torque_max << 200,200,100,100,100,30,30; // N.m

    Eigen::VectorXd torque_min;
    torque_min.resize(arm.getNrOfJoints());
    torque_min = -torque_max; // N.m
    
    // Compute bounds
    Eigen::VectorXd lb(arm.getNrOfJoints()),
                    ub(arm.getNrOfJoints());
    
    //TODO : write constraints for q and qdot
    lb = torque_min;
    ub = torque_max;
    
    
    // Update Constraints
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(arm.getNrOfJoints(),arm.getNrOfJoints());
    Eigen::VectorXd lbA(arm.getNrOfJoints()),
                    ubA(arm.getNrOfJoints()),
                    qd_min(arm.getNrOfJoints()),
                    qd_max(arm.getNrOfJoints());
    
    qd_min.setConstant(-1.0);
    qd_max.setConstant(1.0);
    
    A = arm.getInertiaInverseMatrix().data;
    
    double horizon_dt = 0.015;
    
    Eigen::VectorXd nonLinearTerms(arm.getNrOfJoints());
    nonLinearTerms = arm.getInertiaInverseMatrix().data * ( coriolis.data + gravity.data );    
    
    // TODO: adapt this
    // for( int i; i<arm.getNrOfJoints(); ++i )
    // {
    //     if( fabs(current_jnt_vel[i]) < 1.0e-12 ) // avoid division by zero, por favor
    //         continue;
    //     
    //     // lower bound
    //     tlim = 2.0*( jnt_pos_limit_lower[i] - current_jnt_pos[i] ) / current_jnt_vel[i];
    //     if( tlim < 1.0e-12 ) //  tlim < 0 is of no interest, and tlim = 0 the problem is undefined
    //         continue;
    //     if( (0 <= tlim) && (tlim <= horizon) )
    //         ddq_lower[i] = fmax( ddq_lower[i], -current_jnt_vel[i] / tlim  ); // ddq >= dq^2 / (2(q-qmin))
    //         
    //     // upper bound
    //     tlim = 2.0*( jnt_pos_limit_upper[i] - current_jnt_pos[i] ) / current_jnt_vel[i];
    //     if( tlim < 1.0e-12 ) //  tlim < 0 is of no interest, and tlim = 0 the problem is undefined
    //         continue;
    //     if( (0 <= tlim) && (tlim <= horizon) )
    //         ddq_upper[i] = fmin( ddq_upper[i], -current_jnt_vel[i] / tlim  ); // ddq <= dq^2 / (2(q-qmax))
    // }

    lbA = (( qd_min - this->joint_velocity_in ) / horizon_dt + nonLinearTerms).cwiseMax(
                    2*(arm.getJointLowerLimit() - this->joint_position_in - this->joint_velocity_in * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
    
    ubA = (( qd_max - this->joint_velocity_in ) / horizon_dt + nonLinearTerms).cwiseMin(
                    2*(arm.getJointUpperLimit() - this->joint_position_in - this->joint_velocity_in * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
    
    
    // number of allowed compute steps
    int nWSR = 1e6; 
    
    // Let's compute !
    qpOASES::returnValue ret;
    static bool qpoases_initialized = false;
    
    if(!qpoases_initialized)
    {
        // Initialise the problem, once it has found a solution, we can hotstart
        ret = qpoases_solver->init(H.data(),g.data(),A.data(),lb.data(),ub.data(),lbA.data(),ubA.data(),nWSR);
        
        // Keep init if it didn't work
        if(ret == qpOASES::SUCCESSFUL_RETURN)
        {
            qpoases_initialized = true;
        }
    }
    else
    {
        // Otherwise let's reuse the previous solution to find a solution faster
        ret = qpoases_solver->hotstart(H.data(),g.data(),A.data(),lb.data(),ub.data(),lbA.data(),ubA.data(),nWSR);
        
        if(ret != qpOASES::SUCCESSFUL_RETURN)
        {
            qpoases_initialized = false;
        }
    }
    
    // Zero grav if not found
    // TODO: find a better alternative
    this->joint_torque_out.setZero();
    
    if(ret == qpOASES::SUCCESSFUL_RETURN)
    {
        // Get the solution
        qpoases_solver->getPrimalSolution(this->joint_torque_out.data());
        // Remove gravity because Kuka already adds it
        this->joint_torque_out -= arm.getGravityTorque().data;
    }

    // Send torques to the robot
    this->port_joint_torque_out.write(this->joint_torque_out);
}


void CartOptCtrl::stopHook()
{
    this->has_first_command = false;
}

