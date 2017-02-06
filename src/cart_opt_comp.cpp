#include "cart_opt_ctrl/cart_opt_comp.hpp"
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
    this->addPort("TrajectoryPointPosIn",this->port_pnt_pos_in);
    this->addPort("TrajectoryPointVelIn",this->port_pnt_vel_in);
    this->addPort("TrajectoryPointAccIn",this->port_pnt_acc_in);
    this->addPort("PoseDesired",this->port_x_des);
    this->addPort("JointPosVelIn",this->port_joint_pos_vel_in);
    this->addPort("PoseErrorOut",this->port_error_out);
    this->addPort("ButtonPressed",this->port_button_pressed_in);
    
    this->addProperty("FrameOfInterest",this->ee_frame).doc("The robot frame to track the trajectory");
    this->addProperty("P_gain",this->P_gain).doc("Proportional gain");
    this->addProperty("D_gain",this->D_gain).doc("Derivative gain");
    this->addProperty("Position_Saturation",this->Position_Saturation).doc("Position saturation");
    this->addProperty("Orientation_Saturation",this->Orientation_Saturation).doc("Orientation saturation");
    this->addProperty("Regularisation_Weight",this->Regularisation_Weight).doc("Weight for the regularisation in QP");
    this->addProperty("Compensate_Gravity",this->Compensate_Gravity).doc("Do we need to compensate gravity ?");
    this->addProperty("Damping_Weight",this->Damping_Weight).doc("Weight for the damping in regularisation");
    this->addProperty("Torque_Max",this->Torque_Max).doc("Max torque for each joint");
    this->addProperty("Joint_Velocity_Max",this->Joint_Velocity_Max).doc("Max velocity for each joint");
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
    this->Torque_Max.resize(dof);
    this->Joint_Velocity_Max.resize(dof);
    
    // Default params
    this->ee_frame = arm.getSegmentName( arm.getNrOfSegments() - 1 );
    this->P_gain << 1000,1000,1000,1000,1000,1000;
    this->D_gain << 22,22,22,22,22,22;
    this->Position_Saturation = 0.01;
    this->Orientation_Saturation = M_PI/100;
    this->Regularisation_Weight = 1e-05;
    this->Compensate_Gravity = true;
    this->Damping_Weight = 1.0;
    // TODO: get this from URDF
    this->Torque_Max << 175,175,99,99,99,37,37 ;
    this->Joint_Velocity_Max << 1.0,1.0,1.0,1.0,1.0,1.0,1.0;
    
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
    
    // Initialize start values
    button_pressed = false;
    transition_gain = 1.0;
    
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
        log(RTT::Error) << "Robot ports empty !" << endlog();
        return;
    }
    
    // Feed the internal model
    arm.setState(this->joint_position_in,this->joint_velocity_in);
    // Make some calculations
    arm.updateModel();
    
    // Get Current end effector Pose
    X_curr = arm.getSegmentPosition(this->ee_frame);
    Xd_curr = arm.getSegmentVelocity(this->ee_frame);

    // Initialize the desired velocity and acceleration to zero
    KDL::SetToZero(Xd_traj);
    KDL::SetToZero(Xdd_traj);
    
    // If we get a new trajectory point to track
    if((this->port_pnt_pos_in.read(this->pt_pos_in) != RTT::NoData) && (this->port_pnt_vel_in.read(this->pt_vel_in) != RTT::NoData) && (this->port_pnt_acc_in.read(this->pt_acc_in) != RTT::NoData))
    {
        // Then overrride the desired
      X_traj = pt_pos_in;
      Xd_traj = pt_vel_in;
      Xdd_traj = pt_acc_in;
        
        has_first_command = true;
    }
    else{
      log(RTT::Warning) << "Trajectory ports empty !" << endlog();
      return;
    }
    
    // First step, initialise the first X,Xd,Xdd desired
    if(!has_first_command)
    {
        // Stay at the same position
        X_traj = X_curr;
        has_first_command = true;
    }
    
    // Debug publish in ROS
    geometry_msgs::Pose x_des_pos_out;
    tf::poseKDLToMsg(X_traj,x_des_pos_out);
    geometry_msgs::PoseStamped x_des_pos_stamped_out;
    x_des_pos_stamped_out.pose = x_des_pos_out;
    x_des_pos_stamped_out.header.frame_id = "link_0";
    x_des_pos_stamped_out.header.stamp = rtt_rosclock::host_now();
    
    this->port_x_des.write(x_des_pos_stamped_out);
    trajectory_msgs::JointTrajectoryPoint joint_pos_vel;
    for(int i=0; i< arm.getNrOfJoints(); i++){
      joint_pos_vel.positions.push_back(joint_position_in(i));
      joint_pos_vel.velocities.push_back(joint_velocity_in(i));
    }
    this->port_joint_pos_vel_in.write(joint_pos_vel);
      
    // Compute errors
    X_err = diff( X_curr , X_traj );
    Xd_err = diff( Xd_curr , Xd_traj);
    
    geometry_msgs::Twist error_twist_ros;
    tf::twistKDLToMsg(X_err, error_twist_ros);
    port_error_out.write(error_twist_ros);
    
    // Saturate the pose error
    for(unsigned int i=0; i<3; ++i )
    {
      if(X_err(i) >0)
        X_err(i) = std::min(Position_Saturation, X_err(i));
      else
        X_err(i) = std::max(-Position_Saturation, X_err(i));
    }
    for(unsigned int i=3; i<6; ++i )
    {
      if(X_err(i) >0)
        X_err(i) = std::min(Orientation_Saturation, X_err(i));
      else
        X_err(i) = std::max(-Orientation_Saturation, X_err(i));
    }    
    
    // Apply PD 
    for( unsigned int i=0; i<6; ++i )
    {
        Xdd_des(i) = Xdd_traj(i) + this->P_gain(i) * ( X_err(i) ) + this->D_gain(i) * ( Xd_err(i) );
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
    // With a = 00001J.Minv
    //      b = - J.Minv.( B + G ) + Jdot.qdot - Xdd_des
    
    Eigen::MatrixXd decoupling_position, decoupling_orientation;
    decoupling_position.resize(6,6);
    decoupling_orientation.resize(6,6);
    decoupling_position.setIdentity();
    decoupling_orientation.setIdentity();
    decoupling_position(3,3)=0;
    decoupling_position(4,4)=0;
    decoupling_position(5,5)=0;
    decoupling_orientation(0,0)=0;
    decoupling_orientation(1,1)=0;
    decoupling_orientation(2,2)=0;
    
    if(this->port_button_pressed_in.read(this->button_pressed_msg) != RTT::NoData){
      button_pressed = button_pressed_msg.data;
    }
    if (button_pressed)
      transition_gain = 0.0;
    else
      transition_gain = std::min(1.0,transition_gain + 0.1 * Regularisation_Weight);
    decoupling_position *= transition_gain;
    decoupling_orientation *= transition_gain;
        
    Eigen::Matrix<double,6,Eigen::Dynamic> a, ao;
    a.resize(6,arm.getNrOfJoints());
    ao.resize(6,arm.getNrOfJoints());
    a.noalias() = decoupling_position* J.data * M_inv.data;
    ao.noalias() = decoupling_orientation* J.data * M_inv.data;
    
    Eigen::Matrix<double,6,1> b, bo;
    b.noalias() = - a * ( coriolis.data + gravity.data ) + decoupling_position* jdot_qdot - decoupling_position *xdd_des;
    bo.noalias() = - ao * ( coriolis.data + gravity.data ) + decoupling_orientation* jdot_qdot - decoupling_orientation *xdd_des;
    
    // Matrices for qpOASES
    // NOTE: We need RowMajor (see qpoases doc)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
    H.resize(arm.getNrOfJoints(),arm.getNrOfJoints());
    
    Eigen::VectorXd g(arm.getNrOfJoints());
    
    Eigen::MatrixXd regularisation;
    regularisation.resize( arm.getNrOfJoints(), arm.getNrOfJoints() );
    regularisation.setIdentity();
    regularisation *= this->Regularisation_Weight;
    
    Eigen::MatrixXd damping_regularisation;
    damping_regularisation.resize( arm.getNrOfJoints(), arm.getNrOfJoints() );
    damping_regularisation.setIdentity();
    damping_regularisation *= Damping_Weight;
    
    H = 2.0 * a.transpose() * a + 2.0 * ao.transpose() * ao + 2.0* regularisation;
    if (Compensate_Gravity)
      g = 2.0 * a.transpose() * b +  2.0 * ao.transpose()* bo - 2.0* (regularisation * (gravity.data - damping_regularisation* joint_velocity_in));
    else
      g = 2.0 * a.transpose() * b +  2.0 * ao.transpose()* bo;
    
    // Compute bounds
    Eigen::VectorXd lb(arm.getNrOfJoints()),
                    ub(arm.getNrOfJoints());
    lb = -Torque_Max;
    ub = Torque_Max;    
    
    // Update Constraints
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(arm.getNrOfJoints(),arm.getNrOfJoints());
    Eigen::VectorXd lbA(arm.getNrOfJoints()),
                    ubA(arm.getNrOfJoints()),
                    qd_min(arm.getNrOfJoints()),
                    qd_max(arm.getNrOfJoints());
    
    qd_max = Joint_Velocity_Max;
    qd_min = -Joint_Velocity_Max;
    
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
    else{
      log(RTT::Error) << "QPOases failed!" << endlog();
    }

    // Send torques to the robot
    this->port_joint_torque_out.write(this->joint_torque_out);
}


void CartOptCtrl::stopHook()
{
    this->has_first_command = false;
}

