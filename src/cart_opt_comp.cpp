#include "cart_opt_ctrl/cart_opt_comp.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace RTT;
using namespace KDL;

CartOptCtrl::CartOptCtrl(const std::string& name):RTT::TaskContext(name)
{
  // Orocos ports
  this->addPort("JointPosition",port_joint_position_in_);
  this->addPort("JointVelocity",port_joint_velocity_in_);
  this->addPort("TrajectoryPointPosIn",port_pnt_pos_in_);
  this->addPort("TrajectoryPointVelIn",port_pnt_vel_in_);
  this->addPort("TrajectoryPointAccIn",port_pnt_acc_in_);
  this->addPort("JointTorqueCommand",port_joint_torque_out_);

  this->addPort("PoseEEDesired",port_x_ee_des_);
  this->addPort("PoseEECurrent",port_x_ee_);
  this->addPort("PoseO0OM",port_x_O0OM_);
  this->addPort("PoseTarget",port_x_target_);
  this->addPort("PoseXmError",port_Xm_err);
  this->addPort("PoseXError",port_X_err);


  this->addPort("Encoder",port_encoder_in_);
  this->addPort("FTSensor",port_ft_sensor_in_);
  this->addPort("Ec_constraint",port_Ec_constraints);
  this->addPort("XddDes",port_xdd_des_);
  this->addPort("Xd_Out",port_Xd_out_);
  this->addPort("positioning_error",port_positioning_error);
  this->addPort("pointing_error",port_pointing_error);
  this->addPort("Xdd_out",port_Xdd_out);
  this->addPort("Integral_term",port_integral_term);
  this->addPort("Xdd_des_const",port_xdd_des_const_);
  this->addPort("Xd_curr_",port_Xd_out_const_);
  this->addPort("Data_Thomas",port_Data_Thomas);
  this->addPort("XddFiltered",port_xdd_filtered);
  this->addPort("Jm",port_Jm);
  this->addPort("J",port_J);


  // Orocos properties/ROS params
  this->addProperty("frame_of_interest",ee_frame_).doc("The robot frame to track the trajectory");
  this->addProperty("p_gains",p_gains_).doc("Proportional gains");
  this->addProperty("d_gains",d_gains_).doc("Derivative gains");
  this->addProperty("i_gains",i_gains_).doc("Integral gains");

  this->addProperty("position_saturation",position_saturation_).doc("Position saturation");
  this->addProperty("orientation_saturation",orientation_saturation_).doc("Orientation saturation");
  this->addProperty("regularisation_weight",regularisation_weight_).doc("Weight for the regularisation in QP");
  this->addProperty("compensate_gravity",compensate_gravity_).doc("Do we need to compensate gravity ?");
  this->addProperty("damping_weight",damping_weight_).doc("Weight for the damping in regularisation");
  this->addProperty("integral_pos_saturation",integral_pos_saturation).doc("Position Saturation of the integral term");
  this->addProperty("integral_ori_saturation",integral_ori_saturation).doc("Orientation Saturation of the integral term");
  this->addProperty("Ec_lim",Ec_lim).doc("Potential energy limit");
  this->addProperty("F_lim",F_lim).doc("Maximum pushing force");

  this->addProperty("X_target",targetx).doc("X position of the target");
  this->addProperty("Y_target",targety).doc("Y position of the target");;
  this->addProperty("Z_target",targetz).doc("Z position of the target");;
  this->addProperty("Moving_target",moving_target).doc("0 = no, 1 = yes");
  this->addProperty("Xx",Xx(0)).doc("0 = no, 1 = yes");
  this->addProperty("Xy",Xx(1)).doc("0 = no, 1 = yes");
  this->addProperty("Xx2",O0OM.p.data[0]).doc("0 = no, 1 = yes");
  this->addProperty("Xy2",O0OM.p.data[1]).doc("0 = no, 1 = yes");
  this->addProperty("Xz2",O0OM.p.data[2]).doc("0 = no, 1 = yes");
  this->addProperty("start_pointing",start_pointing).doc("0 = no, 1 = yes");

  // Service to get current cartesian pose
  this->addOperation("getCurrentPose",&CartOptCtrl::getCurrentPose,this,RTT::ClientThread);
}

bool CartOptCtrl::getCurrentPose(cart_opt_ctrl::GetCurrentPose::Request& req, cart_opt_ctrl::GetCurrentPose::Response& resp){
  tf::poseKDLToMsg(X_curr_,resp.current_pose);
  resp.success = true;
  return true;
}

KDL::Rotation CartOptCtrl::PointTarget(KDL::Frame frame_target, KDL::Frame frame_des)
{
  //Direction axe z en soustrayant le point actuel vs le point visé

  KDL::Vector dir_des = frame_target.p - frame_des.p;
  double ray_length = dir_des.Norm();
  dir_des.Normalize();


  //Calcul de la direction actuelle selon z
  KDL::Vector axe_curr_z = X_curr_.M * KDL::Vector(0,0,1);



  //Calcul de l'angle entre la direction actuelle et la direction desirée
  double theta = acos(KDL::dot(dir_des,axe_curr_z));

  //Calcul de l'axe de rotation autour des z
  KDL::Vector axe_rot = axe_curr_z*dir_des;

  KDL::Vector axe_rot_plot = axe_rot;

  axe_rot.Normalize();

  double x,y,z;
  x = axe_rot(0);
  y = axe_rot(1);
  z = axe_rot(2);


  // Matrice de rotation angle(theta),origine(frame_des.p)
  Eigen::Matrix<double,3,3>mat;
  mat.setZero();

  mat(0,0) = cos(theta) + x*x * (1-cos(theta));
  mat(0,1) = x*y*(1-cos(theta)) - z*sin(theta);
  mat(0,2) = x*z*(1-cos(theta)) + y*sin(theta);

  mat(1,0) = y*x*(1-cos(theta)) + z*sin(theta);
  mat(1,1) = cos(theta) + y*y*(1-cos(theta));
  mat(1,2) = y*z*(1-cos(theta)) - x*sin(theta);

  mat(2,0) = z*x*(1-cos(theta))-y*sin(theta);
  mat(2,1) = z*y*(1-cos(theta)) + x*sin(theta);
  mat(2,2) = cos(theta) + z*z*(1-cos(theta));

  KDL::Rotation matrot(mat(0,0),mat(0,1),mat(0,2),mat(1,0),mat(1,1),mat(1,2),mat(2,0),mat(2,1),mat(2,2));

  // Rotation de theta autour de l'axe orthogonal à la direction selon z
  KDL::Rotation rot2 = matrot*frame_des.M;

  return rot2;
}

Eigen::Matrix<double,2,1> CartOptCtrl::GetPointingTarget(KDL::Frame frame,KDL::Frame frame_target)
{
  // Projection on frame target plan
  Eigen::Matrix<double,2,1> X_projection;
  KDL::Vector dir = frame.M * KDL::Vector(0,0,1);

  X_projection(0) = frame.p.x()- dir(0)/dir(2)*(frame.p.z()-frame_target.p.z()); // X projection on the plan (0,0,targetx)
  X_projection(1) = frame.p.y()- dir(1)/dir(2)*(frame.p.z()-frame_target.p.z()); // Y projection on the plan (0,0,targety)

  return X_projection;
}

double CartOptCtrl::FrictionCompensation(int JointNumber,double TorqueFriction)
{
  if (joint_torque_out_(JointNumber) <-torque_thresh)
    return joint_torque_out_(JointNumber)+= -TorqueFriction;
  else if (joint_torque_out_(JointNumber) > torque_thresh)
    return joint_torque_out_(JointNumber) += TorqueFriction;
  else
    return joint_torque_out_(JointNumber);
}



bool CartOptCtrl::configureHook(){
  // Initialise the model, the internal solvers etc
  if( ! arm_.init() ){
    log(RTT::Error) << "Could not init chain utils !" << endlog();
    return false;
  }
  // The number of joints
  const int dof = arm_.getNrOfJoints();
  xd_curr_filtered_.setZero();
  xd_curr_.setZero();
  xd_curr_filtered_old_.setZero();
  xd_curr_old_.setZero();
  xdd_filtered_old_.setZero();
  xdd_old_.setZero();

  target.resize(3);

  // Resize the gains vectors
  p_gains_.resize(6);
  d_gains_.resize(6);
  i_gains_.resize(6);

  torque_max_.resize(dof);
  jnt_vel_max_.resize(dof);

  damping_weight_.resize(dof);
  joint_torque_out_.setZero(dof);
  joint_position_in_.setZero(dof);
  joint_velocity_in_.setZero(dof);

  friction_jnt_4 = 0.6;
  friction_jnt_5 = 0.45;
  friction_jnt_6 = 0.3;
  torque_thresh = 0.1;

  // Default params
  ee_frame_ = arm_.getSegmentName( arm_.getNrOfSegments() - 1 );
  link_6 =  "iiwa_link_6";

  integral_pos_saturation = 0.0;
  integral_ori_saturation = 0.0;
  position_saturation_ = 0.01;
  orientation_saturation_ = M_PI/100;

  horizon = 15;
  Ec_lim = 0.5;
  F_lim = 0.5;
  regularisation_weight_ = 1e-05;
  damping_weight_  << 1.0,1.0,1.0,1.0,1.0,1.0,1.0;
  compensate_gravity_ = true;
  start_pointing = false;
  moving_target=false;

  // TODO: get this from URDF
  torque_max_ << 175,175,99,99,99,37,37 ;
  jnt_vel_max_ << 1.0,1.0,1.0,1.0,1.0,1.0,1.0;

  // Match all properties (defined in the constructor)
  // with the rosparams in the namespace :
  // nameOfThisComponent/nameOftheProperty
  // Equivalent to ros::param::get("CartOptCtrl/p_gains_");
  rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);

  int number_of_variables = dof;
  number_of_constraints = dof;
  qpoases_solver_.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints,qpOASES::HST_POSDEF));

  // QPOases options
  qpOASES::Options options;
  // This options enables regularisation (required) and disable
  // some checks to be very fast !
  // options.setToDefault();
  options.setToMPC(); // setToReliable() // setToDefault()
  options.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
  options.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
  qpoases_solver_->setOptions( options );
  qpoases_solver_->setPrintLevel(qpOASES::PL_NONE); // PL_HIGH for full output, PL_NONE for... none

  //Butterworth filter
  // [n,Wn] = buttord(40/500,200/500,3,20)
  // [b,a] = butter(n,Wn)
  // induce 8ms delay

  a_butter.setZero(3);b_butter.setZero(3);
  a_butter << 1.0000  , -1.8583  ,  0.8677;
  b_butter <<  0.0023  ,  0.0047 ,   0.0023;
  filtered_acc.setZero();
  filtered_vel.setZero();
  old_acc.setZero();
  old_vel.setZero();

  ff_xd.setZero();
  X_err_old.setZero();
  ff_xd_old.setZero();
  ff_xdd.setZero();

  return true;
}

bool CartOptCtrl::startHook(){
  // Initialize start values
  has_first_command_ = false;

  return true;
}

void CartOptCtrl::updateHook(){
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
  X_curr_ = arm_.getSegmentPosition(ee_frame_);
  Xd_curr_ = arm_.getSegmentVelocity(ee_frame_);

  // Initialize the desired velocity and acceleration to zero
  KDL::SetToZero(Xd_traj_);
  KDL::SetToZero(Xdd_traj_);

  frame_target = Frame(Rotation::RPY(0.0,0.0,0.0),Vector(targetx,targety,targetz));

  // If we get a new trajectory point to track
  if((port_pnt_pos_in_.read(pt_pos_in_) != RTT::NoData) && (port_pnt_vel_in_.read(pt_vel_in_) != RTT::NoData) && (port_pnt_acc_in_.read(pt_acc_in_) != RTT::NoData)){
    // Then overwrite the desired
    X_traj_ = pt_pos_in_;
    Xd_traj_ = pt_vel_in_;
    Xdd_traj_ = pt_acc_in_;

    if (moving_target)
      frame_target = Frame(Rotation::RPY(0.0,0.0,0.0),Vector(pt_pos_in_.p.x(),pt_pos_in_.p.y(),0.0));

    KDL::Rotation rotz = PointTarget(frame_target,X_curr_);

    has_first_command_ = true;
//     if(start_pointing)
//       X_traj_ = KDL::Frame(rotz,pt_pos_in_.p);
//     else
      X_traj_ = KDL::Frame(pt_pos_in_.M,pt_pos_in_.p);
  }
  else{
    log(RTT::Warning) << "Trajectory ports empty !" << endlog();
    X_traj_ = X_curr_;
  }

  // First step, initialise the first X,Xd,Xdd desired
  if(!has_first_command_){
    // Stay at the same position
    X_traj_ = X_curr_;
    has_first_command_ = true;
    X_traj_ = KDL::Frame(X_curr_.M,X_curr_.p);
  }

  // Compute errors
  X_err_ = diff( X_curr_ , X_traj_ );
  Xd_err_ = diff( Xd_curr_ , Xd_traj_);

  trajectory_msgs::JointTrajectoryPoint joint_pos_vel;
  for(int i=0; i< arm_.getNrOfJoints(); i++){
    joint_pos_vel.positions.push_back(joint_position_in_(i)*180.0/3.14159);
    joint_pos_vel.velocities.push_back(joint_velocity_in_(i)*180.0/3.14159);
  }

  geometry_msgs::Twist Xd_twist_ros;
  tf::twistKDLToMsg(Xd_traj_, Xd_twist_ros);
  port_Xd_out_.write(Xd_twist_ros);

  // Projection on target plan
  Xx = GetPointingTarget(X_curr_,frame_target);
  pointing_error.data = sqrt(pow(frame_target.p.x()-Xx(0),2)+pow(frame_target.p.y()-Xx(1),2))*1000; // Pointing_error in mm;

  positioning_error.data = X_err_.vel.Norm()*1000; // Positioning_error in mm;

  port_pointing_error.write(pointing_error);
  port_positioning_error.write(positioning_error);

  tf::twistKDLToEigen(Xd_curr_, xd_curr_);

  // Saturate the pose error
  for(unsigned int i=0; i<3; ++i ){
    if(X_err_(i) >0)
      X_err_(i) = std::min( position_saturation_, X_err_(i));
    else
      X_err_(i) = std::max(-position_saturation_, X_err_(i));
  }
//   for(unsigned int i=3; i<6; ++i ){
//     if(X_err_(i) >0)
//       X_err_(i) = std::min( orientation_saturation_, X_err_(i));
//     else
//       X_err_(i) = std::max(-orientation_saturation_, X_err_(i));
//   }

  //Integral term and saturation
  for( unsigned int i=0; i<3; ++i )
  {
     integral_term(i) += X_err_(i);
     if (fabs(integral_term(i)) > 0.1)
	    integral_term(i) = sgn(integral_term(i))*0.1;
     if (i_gains_(i)*integral_term(i)>0)
      integral_term_sat(i) = std::min(i_gains_(i)*integral_term(i),integral_pos_saturation);
     else
      integral_term_sat(i) = std::max(i_gains_(i)*integral_term(i),-integral_pos_saturation);
  }
//     for( unsigned int i=3; i<6; ++i )
//   {
//      integral_term(i) += X_err_(i);
//      if (fabs(integral_term(i)) > 0.4	)
// 	    integral_term(i) = sgn(integral_term(i))*0.4;
//      if (i_gains_(i)*integral_term(i)>0)
//       integral_term_sat(i) = std::min(i_gains_(i)*integral_term(i),integral_ori_saturation);
//      else
//       integral_term_sat(i) = std::max(i_gains_(i)*integral_term(i),-integral_ori_saturation);
//   }

  tf::twistKDLToMsg(Xdd_traj_,integral_term_msg);
  port_integral_term.write(integral_term_msg);

  // Apply PID
  for( unsigned int i=0; i<3; ++i )
    Xdd_des(i) = Xdd_traj_(i) + p_gains_(i) * ( X_err_(i) ) - d_gains_(i) * ( Xd_curr_(i) ) + integral_term_sat(i) ;
  for( unsigned int i=3; i<6; ++i )
    Xdd_des(i) = Xdd_traj_(i) + p_gains_(i) * ( X_err_(i) ) - d_gains_(i) * ( Xd_curr_(i) ) + integral_term_sat(i) ;

  Eigen::Matrix<double,6,1> xdd_des,xddm_des;
  tf::twistKDLToEigen(Xdd_des,xdd_des);
  tf::twistKDLToEigen(Xddm_des,xddm_des);

  tf::twistKDLToMsg(Xdd_traj_,xdd_des_msg);
   if (!has_first_command_)
    xd_curr_filtered_ = xd_curr_;
  else{
    for(int i=0; i<6 ; i++)
    {
      xd_curr_filtered_(i) = 0.95 * xd_curr_filtered_(i) + 0.05 * xd_curr_(i);
     }
  }
  tf::matrixEigenToMsg(xd_curr_,xd_curr_filtered_msg_);
  port_Xd_out_const_.write(xd_curr_filtered_msg_);

   if (!has_first_command_)
    Xdd_filtered  = xdd_des;
  else{
    for(int i=0; i<6 ; i++)
    {
      Xdd_filtered(i) = 0.95 * Xdd_filtered(i) + 0.05 * xdd_des(i);
    }
  }

  tf::matrixEigenToMsg(Xdd_filtered,xdd_filtered_msg_);
  KDL::Frame O0O6 = arm_.getSegmentPosition(link_6);
  KDL::Jacobian J = arm_.getSegmentJacobian(ee_frame_);
  KDL::JntSpaceInertiaMatrix& M_inv = arm_.getInertiaInverseMatrix();
  KDL::JntArray& coriolis = arm_.getCoriolisTorque();
  KDL::JntArray& gravity = arm_.getGravityTorque();
  KDL::Twist& Jdotqdot = arm_.getSegmentJdotQdot(ee_frame_);
  Eigen::Matrix<double,6,1> jdot_qdot,jmdot_qdot;
  tf::twistKDLToEigen(Jdotqdot,jdot_qdot);

  Eigen::MatrixXd regularisation = regularisation_weight_ * M_inv.data;
  Eigen::MatrixXd damping = damping_weight_.asDiagonal();

  //Calcul de la direction actuelle selon z
  KDL::Vector z_0 = KDL::Vector(0,0,1);
  KDL::Vector axe_curr_z = X_curr_.M * z_0;

  ray_length2 = - KDL::dot(X_curr_.p,z_0)/KDL::dot(axe_curr_z,z_0);
  KDL::Frame O0OM = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(X_curr_.p + ray_length2*axe_curr_z));
  KDL::Vector O0OMd = -1/KDL::dot(z_0,axe_curr_z)*z_0*(axe_curr_z*(X_curr_.p-ray_length2*axe_curr_z*Xd_curr_.rot));

  O6OM = O0O6.Inverse()*O0OM;

//   KDL::Vector z6 = O0O6.M * KDL::Vector(0,0,1);
//   KDL::Twist J7;
//   J7.vel = O6OM.p * z6;
//   J7.rot = z6;

  KDL::Jacobian Jm = arm_.getSegmentJacobian(ee_frame_);
  KDL::Twist Jmdotqdot = arm_.getSegmentJdotQdot(ee_frame_);


  Jm.changeRefPoint(-O6OM.p);
  Jmdotqdot.RefPoint(-O6OM.p);
  tf::twistKDLToEigen(Jmdotqdot,jmdot_qdot);

  KDL::Twist Xm_err_ = diff( O0OM , frame_target );

   //Integral term and saturation
  for( unsigned int i=0; i<3; ++i )
  {
     integral_m_term(i) += Xm_err_(i);
     if (fabs(integral_m_term(i)) > 0.1)
	    integral_m_term(i) = sgn(integral_m_term(i))*0.1;
     if (i_gains_(i)*integral_m_term(i)>0)
      integral_m_term_sat(i) = std::min(i_gains_(i)*integral_m_term(i),integral_pos_saturation);
     else
      integral_m_term_sat(i) = std::max(i_gains_(i)*integral_m_term(i),-integral_pos_saturation);
  }


  // Apply PID
  for( unsigned int i=0; i<3; ++i )
    Xddm_des(i) = p_gains_(i) * ( Xm_err_(i) ) - d_gains_(i) * ( O0OMd(i) ) + integral_m_term_sat(i) ;

  // Matrices for qpOASES
  // NOTE: We need RowMajor (see qpoases doc)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
  H.resize(arm_.getNrOfJoints(),arm_.getNrOfJoints());
  Eigen::VectorXd g(arm_.getNrOfJoints());

  // Regularisation task
  // Can be tau, tau-g or tau-g-b*qdot
  H = 2.0 * regularisation;
  if (compensate_gravity_)
    g = - 2.0* (regularisation * (gravity.data - damping* joint_velocity_in_));

  // Write cartesian tasks
  // The cartesian tasks can be decoupling by axes
  Eigen::Matrix<double,3,Eigen::Dynamic> a;
  a.resize(3,arm_.getNrOfJoints());
  Eigen::Matrix<double,3,1> b;

  a.noalias() =  J.data.block(0,0,3,7) *  M_inv.data;
  b.noalias() = (- a * ( coriolis.data + gravity.data ) + jdot_qdot.block(0,0,3,1) - xdd_des.block(0,0,3,1));

  H +=  2.0 * a.transpose() * a;
  g +=  2.0 * a.transpose() * b;

  if(start_pointing){
    a.noalias() =  Jm.data.block(0,0,3,7) *  M_inv.data;
    b.noalias() = (- a * ( coriolis.data + gravity.data ) + jmdot_qdot.block(0,0,3,1) - xddm_des.block(0,0,3,1));

    H +=  2.0 * a.transpose() * a;
    g +=  2.0 * a.transpose() * b;
  }
  else{
    a.noalias() =  J.data.block(3,0,3,7) *  M_inv.data;
    b.noalias() = (- a * ( coriolis.data + gravity.data ) + jdot_qdot.block(3,0,3,1) - xdd_des.block(3,0,3,1));

    H +=  2.0 * a.transpose() * a;
    g +=  2.0 * a.transpose() * b;
  }

  // Compute bounds
  Eigen::VectorXd lb(arm_.getNrOfJoints()),
		  ub(arm_.getNrOfJoints());
  lb = -torque_max_;
  ub = torque_max_;

  // Update Constraints
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(number_of_constraints,arm_.getNrOfJoints());
  Eigen::VectorXd lbA(number_of_constraints),
		  ubA(number_of_constraints),
		  qd_min(arm_.getNrOfJoints()),
		  qd_max(arm_.getNrOfJoints());

  qd_max = jnt_vel_max_;
  qd_min = -jnt_vel_max_;

  double horizon_dt = horizon*this->getPeriod();

  Eigen::VectorXd nonLinearTerms(arm_.getNrOfJoints());
  nonLinearTerms = arm_.getInertiaInverseMatrix().data * ( coriolis.data + gravity.data );

  Eigen::Matrix<double,6,6> Lambda;
  Lambda.setZero();
  delta_x.setZero();

  double horizon_ec = sqrt(2.0 * Ec_lim/(F_lim*(p_gains_(0) * position_saturation_ + integral_pos_saturation)));

   // Filter current speed for kinetic energy computation
  if (!has_first_command_)
    xd_curr_filtered_ = xd_curr_;
  else{
    for(int i=0; i<6 ; i++)
      xd_curr_filtered_(i) = 0.98 * xd_curr_filtered_(i) + 0.02 * xd_curr_(i);
  }

  // Ec current and Ec next
  Lambda = (J.data * M_inv.data * J.data.transpose()).inverse();
  delta_x = xd_curr_filtered_ * horizon_ec + 0.5 * xdd_des * horizon_ec * horizon_ec;
  double ec_curr = 0.5 * xd_curr_filtered_.transpose() * Lambda * xd_curr_filtered_;
  double B_ec = ec_curr + delta_x.transpose() * Lambda * (jdot_qdot - J.data * nonLinearTerms);

  Eigen::Matrix<double,1,7> A_ec;
  A_ec = delta_x.transpose() * Lambda * J.data * M_inv.data;

  lbA.block(0,0,7,1) = (( qd_min - joint_velocity_in_ ) / horizon_dt + nonLinearTerms).cwiseMax(
  2*(arm_.getJointLowerLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );

  ubA.block(0,0,7,1) = (( qd_max - joint_velocity_in_ ) / horizon_dt + nonLinearTerms).cwiseMin(
  2*(arm_.getJointUpperLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );

//   lbA(7) = -100000000.0  - B_ec;
//   ubA(7) = Ec_lim - B_ec;

   A.block(0,0,7,7) = arm_.getInertiaInverseMatrix().data;
//   A.block(7,0,1,7) = A_ec;

  // number of allowed compute steps
  int nWSR = 1e6;

  // Let's compute !
  qpOASES::returnValue ret;
  static bool qpoases_initialized = false;

  if(!qpoases_initialized){
    // Initialise the problem, once it has found a solution, we can hotstart
    ret = qpoases_solver_->init(H.data(),g.data(),A.data(),lb.data(),ub.data(),lbA.data(),ubA.data(),nWSR);

    // Keep init if it didn't work
    if(ret == qpOASES::SUCCESSFUL_RETURN)
      qpoases_initialized = true;
  }
  else{
    // Otherwise let's reuse the previous solution to find a solution faster
    ret = qpoases_solver_->hotstart(H.data(),g.data(),A.data(),lb.data(),ub.data(),lbA.data(),ubA.data(),nWSR);

    if(ret != qpOASES::SUCCESSFUL_RETURN)
      qpoases_initialized = false;
  }

  // Zero grav if no solution found
  // TODO: find a better alternative
  joint_torque_out_.setZero();

  if(ret == qpOASES::SUCCESSFUL_RETURN){
    // Get the solution
    qpoases_solver_->getPrimalSolution(joint_torque_out_.data());
    // Remove gravity because Kuka already adds it
    joint_torque_out_ -= arm_.getGravityTorque().data;
  }
  else
    log(RTT::Error) << "QPOases failed!" << endlog();



  double Ec_constraint = B_ec+A_ec*(joint_torque_out_+arm_.getGravityTorque().data);

  //   Joint Friction compensation
  FrictionCompensation(4,friction_jnt_4);
  FrictionCompensation(5,friction_jnt_5);
  FrictionCompensation(6,friction_jnt_6);

  tf::matrixEigenToMsg(delta_x,delta_x_msg);

  Eigen::Matrix<double,5,1> Ec_constraints;
  Ec_constraints(0) = Ec_lim;
  Ec_constraints(1) = Ec_constraint;
  Ec_constraints(2) = ec_curr;
  Ec_constraints(3) = A_ec*(joint_torque_out_+arm_.getGravityTorque().data);
  Ec_constraints(4) = horizon_ec;
  tf::matrixEigenToMsg(Ec_constraints,Ec_constraints_msg);
  port_Ec_constraints.write(Ec_constraints_msg);

  port_xdd_des_.write(xdd_des_msg);
  port_xdd_des_const_.write(xdd_filtered_msg_);

    geometry_msgs::PoseStamped X_target_stamped_msg;
  geometry_msgs::Pose X_target_msg;
  tf::poseKDLToMsg(frame_target,X_target_msg);
  X_target_stamped_msg.header.stamp = rtt_rosclock::host_now();
  X_target_stamped_msg.header.frame_id = arm_.getRootLink();
  X_target_stamped_msg.pose = X_target_msg;
  port_x_target_.write(X_target_stamped_msg);

  geometry_msgs::PoseStamped XO0OM_stamped_msg;
  geometry_msgs::Pose XO0Om_msg;
  tf::poseKDLToMsg(O0OM,XO0Om_msg);
  XO0OM_stamped_msg.header.stamp = rtt_rosclock::host_now();
  XO0OM_stamped_msg.header.frame_id = arm_.getRootLink();
  XO0OM_stamped_msg.pose = XO0Om_msg;
  port_x_O0OM_.write(XO0OM_stamped_msg);

  geometry_msgs::PoseStamped X_ee_stamped_msg;
  geometry_msgs::Pose X_ee_msg;
  tf::poseKDLToMsg(X_curr_,X_ee_msg);
  X_ee_stamped_msg.header.stamp = rtt_rosclock::host_now();
  X_ee_stamped_msg.header.frame_id = arm_.getRootLink();
  X_ee_stamped_msg.pose = X_ee_msg;
  port_x_ee_.write(X_ee_stamped_msg);

  geometry_msgs::PoseStamped X_ee_des_stamped_msg;
  geometry_msgs::Pose X_ee_des_msg;
  tf::poseKDLToMsg(X_traj_,X_ee_des_msg);
  X_ee_des_stamped_msg.header.stamp = rtt_rosclock::host_now();
  X_ee_des_stamped_msg.header.frame_id = arm_.getRootLink();
  X_ee_des_stamped_msg.pose = X_ee_des_msg;
  port_x_ee_des_.write(X_ee_des_stamped_msg);



  geometry_msgs::TwistStamped Xm_err_stamped_msg;
  geometry_msgs::Twist Xm_err_msg;
  tf::twistKDLToMsg(Xm_err_,Xm_err_msg);
  Xm_err_stamped_msg.header.stamp = rtt_rosclock::host_now();
  Xm_err_stamped_msg.header.frame_id = arm_.getRootLink();
  Xm_err_stamped_msg.twist = Xm_err_msg;
  port_Xm_err.write(Xm_err_stamped_msg);

  geometry_msgs::TwistStamped X_err_stamped_msg;
  geometry_msgs::Twist X_err_msg;
  tf::twistKDLToMsg(X_err_,X_err_msg);
  X_err_stamped_msg.header.stamp = rtt_rosclock::host_now();
  X_err_stamped_msg.header.frame_id = arm_.getRootLink();
  X_err_stamped_msg.twist = Xm_err_msg;
  port_X_err.write(X_err_stamped_msg);

  std_msgs::Float64MultiArray joint_torque_msg;
  tf::matrixEigenToMsg(joint_torque_out_,joint_torque_msg);
  port_joint_torque_.write(joint_torque_msg);

  // Send torques to the robot
  port_joint_torque_out_.write(joint_torque_out_);


}


void CartOptCtrl::stopHook(){
  has_first_command_ = false;
}
