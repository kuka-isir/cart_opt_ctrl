#include "cart_opt_ctrl/cart_opt_comp.hpp"

using namespace RTT;
using namespace KDL;

CartOptCtrl::CartOptCtrl(const std::string& name):RTT::TaskContext(name)
{
  // Orocos ports
  this->addPort("JointPosition",port_joint_position_in_);
  this->addPort("JointVelocity",port_joint_velocity_in_);
  this->addPort("JointTorqueCommand",port_joint_torque_out_);
  this->addPort("TrajectoryPointPosIn",port_pnt_pos_in_);
  this->addPort("TrajectoryPointVelIn",port_pnt_vel_in_);
  this->addPort("TrajectoryPointAccIn",port_pnt_acc_in_);
  this->addPort("PoseDesired",port_x_des_);
  this->addPort("JointPosVelIn",port_joint_pos_vel_in_);
  this->addPort("PoseErrorOut",port_error_out_);
  this->addPort("ButtonPressed",port_button_pressed_in_);
  
  // Orocos properties/ROS params
  this->addProperty("frame_of_interest",ee_frame_).doc("The robot frame to track the trajectory");
  this->addProperty("base_frame",base_frame_).doc("The robot frame to track the trajectory");
  this->addProperty("p_gains",p_gains_).doc("Proportional gains");
  this->addProperty("i_gains",i_gains_).doc("Integral gains");
  this->addProperty("d_gains",d_gains_).doc("Derivative gains");
  this->addProperty("position_saturation",position_saturation_).doc("Position saturation");
  this->addProperty("orientation_saturation",orientation_saturation_).doc("Orientation saturation");
  this->addProperty("integral_pos_saturation",integral_pos_saturation_).doc("Integral position saturation");
  this->addProperty("integral_rot_saturation",integral_rot_saturation_).doc("Integral orientation saturation");
  this->addProperty("regularisation_weight",regularisation_weight_).doc("Weight for the regularisation in QP");
  this->addProperty("compensate_gravity",compensate_gravity_).doc("Do we need to compensate gravity ?");
  this->addProperty("viscous_walls",viscous_walls_).doc("Do we need viscous wall around constraints ?");
  this->addProperty("max_viscous_coeff",max_viscous_coeff_).doc("Coefficient for viscous walls");
  this->addProperty("viscous_walls_thickness",viscous_walls_thickness_).doc("Thickness of the viscous walls");
  this->addProperty("damping_weight",damping_weight_).doc("Weight for the damping in regularisation");
  this->addProperty("torque_max",torque_max_).doc("Max torque for each joint");
  this->addProperty("joint_vel_max",jnt_vel_max_).doc("Max velocity for each joint");
  this->addProperty("cart_min_constraints",cart_min_constraints_).doc("Max cartesian position constraints");
  this->addProperty("cart_max_constraints",cart_max_constraints_).doc("Min cartesian position constraints");
  this->addProperty("horizon_steps",horizon_steps_).doc("Number of period to anticipate");
  this->addProperty("ec_lim",ec_lim_).doc("Max Ec limit");
  
  select_components_.resize(6);
  select_axes_.resize(select_components_.size());
  for(int i = 0; i<select_components_.size() ; i++){
    std::string name = "select_components_"+ std::to_string(i);
    this->addProperty(name,select_components_[i]).doc("Selection of cartesian components for the task");
    name = "select_axes_"+ std::to_string(i);
    this->addProperty(name,select_axes_[i]).doc("Selection of the axis to use for the task");
  }
  
  // Service to get current cartesian pose
  this->addOperation("getCurrentPose",&CartOptCtrl::getCurrentPose,this,RTT::ClientThread);
}

bool CartOptCtrl::getCurrentPose(cart_opt_ctrl::GetCurrentPose::Request& req, cart_opt_ctrl::GetCurrentPose::Response& resp){
  tf::poseKDLToMsg(X_curr_,resp.current_pose);
  resp.success = true;
  return true;
}

bool CartOptCtrl::configureHook(){
  // Initialise the model, the internal solvers etc
  if( ! arm_.init() ){
    log(RTT::Error) << "Could not init chain utils !" << endlog();
    return false;
  }
  // The number of joints
  const int dof = arm_.getNrOfJoints();
  number_of_constraints_ = dof + 3 + 1;

  // Resize the vectors and matrices
  p_gains_.resize(6);
  i_gains_.resize(6);
  d_gains_.resize(6);
  torque_max_.resize(dof);
  jnt_vel_max_.resize(dof);
  damping_weight_.resize(dof);
  cart_min_constraints_.resize(3);
  cart_max_constraints_.resize(3);
  for(int i = 0; i<select_components_.size() ; i++){
    select_components_[i].resize(6);
    select_axes_[i].resize(dof);
  }
  joint_torque_out_.resize(dof);
  joint_position_in_.resize(dof);
  joint_velocity_in_.resize(dof);
  H_.resize(dof,dof);  
  g_.resize(dof);
  a_.resize(6,dof);
  lb_.resize(dof);
  ub_.resize(dof);
  A_.resize(number_of_constraints_,dof);
  lbA_.resize(number_of_constraints_);
  ubA_.resize(number_of_constraints_);
  qd_min_.resize(dof);
  qd_max_.resize(dof);
  J_.resize(dof);
  M_inv_.resize(dof);
  coriolis_.resize(dof);
  gravity_.resize(dof);
  nonLinearTerms_.resize(dof);
  x_max_.resize(6);
  x_min_.resize(6);
  joint_pos_vel_.positions.resize(dof);
  joint_pos_vel_.velocities.resize(dof);
  
  // Matices init
  H_.setZero(dof, dof);
  g_.setZero(dof);
  a_.setZero(6,dof);
  lb_.setZero(dof);
  ub_.setZero(dof);
  A_.setZero(number_of_constraints_,dof);
  lbA_.setZero(number_of_constraints_);
  ubA_.setZero(number_of_constraints_);
  qd_min_.setZero(dof);
  qd_max_.setZero(dof);
  nonLinearTerms_.setZero(dof);
  x_max_.setZero(6);
  x_min_.setZero(6);
  joint_torque_out_.setZero(dof);
  joint_position_in_.setZero(dof);
  joint_velocity_in_.setZero(dof);
  KDL::SetToZero(integral_error_);
  
  // Default params
  ee_frame_ = arm_.getSegmentName( arm_.getNrOfSegments() - 1 );
  p_gains_ << 1000,1000,1000,1000,1000,1000;
  i_gains_ << 0,0,0,0,0,0;
  d_gains_ << 22,22,22,22,22,22;
  position_saturation_ = 0.01;
  orientation_saturation_ = M_PI/100;
  integral_pos_saturation_ = 0.0;
  integral_rot_saturation_ = 0.0;
  regularisation_weight_ = 1e-05;
  max_viscous_coeff_ = 100;
  viscous_walls_thickness_ = 0.3;
  damping_weight_  << 1.0,1.0,1.0,1.0,1.0,1.0,1.0;
  cart_min_constraints_.setConstant(3, -10.0);
  cart_max_constraints_.setConstant(3, 10.0);
  horizon_steps_ = 15.0; 
  compensate_gravity_ = true;
  viscous_walls_ = true;
  for(int i = 1; i<select_components_.size() ; i++){
    select_components_[i].setZero(6);
    select_axes_[i].setZero(dof);
  }
  select_components_[0].setOnes(6);
  select_axes_[0].setOnes(dof);
  // TODO: get this from URDF
  torque_max_ << 175,175,99,99,99,37,37 ;
  jnt_vel_max_ << 1.0,1.0,1.0,1.0,1.0,1.0,1.0;
  ec_lim_ = 2.0;
  ec_next_filtered_ = 0;
  
  // Match all properties (defined in the constructor) 
  // with the rosparams in the namespace : 
  // nameOfThisComponent/nameOftheProperty
  // Equivalent to ros::param::get("CartOptCtrl/p_gains_");
  rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);

  // QPOases init
  int number_of_variables = dof;
  number_of_constraints_ = dof + 3 + 1;
  qpoases_solver_.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints_,qpOASES::HST_POSDEF));
  
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
  
  return true;
}

bool CartOptCtrl::startHook(){
  // Initialize start values
  has_first_command_ = false;
  button_pressed_ = false;
  transition_gain_ = 1.0;   
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
  
  // If we get a new trajectory point to track
  if((port_pnt_pos_in_.read(pt_pos_in_) != RTT::NoData) && (port_pnt_vel_in_.read(pt_vel_in_) != RTT::NoData) && (port_pnt_acc_in_.read(pt_acc_in_) != RTT::NoData)){
    // Then overwrite the desired
    X_traj_ = pt_pos_in_;
    Xd_traj_ = pt_vel_in_;
    Xdd_traj_ = pt_acc_in_;
  }
  
  // First step, initialise the first X,Xd,Xdd desired
  // Stay at the same position
  if(!has_first_command_)
    X_traj_ = X_curr_;
  
  // Compute errors
  X_err_ = diff( X_curr_ , X_traj_ );
  Xd_err_ = diff( Xd_curr_ , Xd_traj_);
  
  // Debug publish current pose in ROS
  tf::poseKDLToMsg(X_traj_,x_des_pos_out_);
  x_des_pos_stamped_out_.pose = x_des_pos_out_;
  x_des_pos_stamped_out_.header.frame_id = base_frame_;
  x_des_pos_stamped_out_.header.stamp = rtt_rosclock::host_now();
  port_x_des_.write(x_des_pos_stamped_out_);
  
  // Debug publish current position and velocity in ROS
  for(int i=0; i< arm_.getNrOfJoints(); i++){
    joint_pos_vel_.positions[i] = joint_position_in_(i);
    joint_pos_vel_.velocities[i] = joint_velocity_in_(i);
  }
  port_joint_pos_vel_in_.write(joint_pos_vel_);
  
  // Debug publish pose error in ROS
  tf::twistKDLToMsg(X_err_, error_twist_ros_);
  port_error_out_.write(error_twist_ros_);
  
  // Saturate the pose error
  for(unsigned int i=0; i<3; ++i ){
    if(X_err_(i) >0)
      X_err_(i) = std::min(position_saturation_, X_err_(i));
    else
      X_err_(i) = std::max(-position_saturation_, X_err_(i));
  }
  for(unsigned int i=3; i<6; ++i ){
    if(X_err_(i) >0)
      X_err_(i) = std::min(orientation_saturation_, X_err_(i));
    else
      X_err_(i) = std::max(-orientation_saturation_, X_err_(i));
  }
  
  // Saturate the integral term
  for(unsigned int i=0; i<3; ++i ){
    if (i_gains_(i) > 0){
      integral_error_(i) += X_err_(i) * this->getPeriod();
      if(integral_error_(i) >0)
        integral_error_(i) = std::min(integral_pos_saturation_ / i_gains_(i), integral_error_(i));
      else
        integral_error_(i) = std::max(-integral_pos_saturation_ / i_gains_(i), integral_error_(i));
    }
    else
      integral_error_(i) = 0;
  }
  for(unsigned int i=3; i<6; ++i ){
    if (i_gains_(i) > 0){
      integral_error_(i) += X_err_(i) * this->getPeriod();
      if(integral_error_(i) >0)
        integral_error_(i) = std::min(integral_rot_saturation_ / i_gains_(i), integral_error_(i));
      else
        integral_error_(i) = std::max(-integral_rot_saturation_ / i_gains_(i), integral_error_(i));
    }
    else
      integral_error_(i) = 0;
  }  
  
  // Apply PD 
  for( unsigned int i=0; i<6; ++i )
    Xdd_des_(i) = Xdd_traj_(i) + p_gains_(i) * ( X_err_(i) ) + i_gains_(i) * integral_error_(i) - d_gains_(i) * ( Xd_curr_(i) );
  tf::twistKDLToEigen(Xdd_des_,xdd_des_);
  
  // Update current Matrices and vectors
  J_ = arm_.getSegmentJacobian(ee_frame_);
  M_inv_ = arm_.getInertiaInverseMatrix();
  coriolis_ = arm_.getCoriolisTorque();
  gravity_ = arm_.getGravityTorque();
  Jdotqdot_ = arm_.getSegmentJdotQdot(ee_frame_);
  tf::twistKDLToEigen(Jdotqdot_,jdot_qdot_);
  nonLinearTerms_ = M_inv_.data * ( coriolis_.data + gravity_.data );
  tf::twistKDLToEigen(Xd_curr_, xd_curr_);
  tf::vectorKDLToEigen(X_curr_.p, x_curr_lin_);
  x_curr_.block(0,0,3,1) = x_curr_lin_;
  
  // We put it in the form ax + b
  // M(q).qdd + B(qd) + G(q) = T
  // --> qdd = Minv.( T - B - G)
  // Xd = J.qd
  // --> Xdd = Jdot.qdot + J.qdd
  // --> Xdd = Jdot.qdot + J.Minv.( T - B - G)
  // --> Xdd = Jdot.qdot + J.Minv.T - J.Minv.( B + G )
  // And we have Xdd_des = Xdd_traj_ + p_gains_.( X_des - X_curr_) + d_gains_.( Xd_des - Xd_curr_)
  // ==> We want to compute min(T) || Xdd - Xdd_des ||²
  // If with replace, we can put it in the form ax + b
  // With a = J.Minv
  //      b = - J.Minv.( B + G ) + Jdot.qdot - Xdd_des

  // Regularisation task
  // Can be tau, tau-g or tau-g-b*qdot
  H_ = 2.0 * regularisation_weight_ * M_inv_.data;
  if (compensate_gravity_)
    g_ = - 2.0* (regularisation_weight_ * M_inv_.data * (gravity_.data - damping_weight_.asDiagonal() * joint_velocity_in_));
  
  // Read button press port
  this->port_button_pressed_in_.read(button_pressed_);
  
  // If button is pressed leave only the regularisation task
  // Then progressively introduce the cartesian task
  if (button_pressed_)
    transition_gain_ = 0.0;
  else
    transition_gain_ = std::min(1.0,transition_gain_ + 0.001 * regularisation_weight_);
  
  // Write cartesian tasks
  // The cartesian tasks can be decoupling by axes  
  for(int i=0; i<select_components_.size();i++){    
    a_.noalias() =  J_.data * select_axes_[i].asDiagonal() * M_inv_.data;
    b_.noalias() = (- a_ * ( coriolis_.data + gravity_.data ) + jdot_qdot_ - xdd_des_);
    
    H_ += transition_gain_ * 2.0 * a_.transpose() * select_components_[i].asDiagonal() * a_;  
    g_ += transition_gain_ * 2.0 * a_.transpose() * select_components_[i].asDiagonal() * b_;
  }  
  
  // Torque bounds update 
  lb_ = -torque_max_;
  ub_ = torque_max_;    
  
  // Joint velocity bounds update
  qd_max_ = jnt_vel_max_;
  qd_min_ = -jnt_vel_max_;
  
  // Update horizon
  double horizon_dt = horizon_steps_* this->getPeriod();
  
  // Joint position and velocity constraints
  A_.block(0,0,arm_.getNrOfJoints(),arm_.getNrOfJoints()) = M_inv_.data;

  // TODO adapt this ??
//   for( int i; i<arm_.getNrOfJoints(); ++i ){
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
//   }

  lbA_.block(0,0,arm_.getNrOfJoints(),1) = (( qd_min_ - joint_velocity_in_ ) / horizon_dt + nonLinearTerms_).cwiseMax(
      2*(arm_.getJointLowerLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms_ );
  
  ubA_.block(0,0,arm_.getNrOfJoints(),1) = (( qd_max_ - joint_velocity_in_ ) / horizon_dt + nonLinearTerms_).cwiseMin(
      2*(arm_.getJointUpperLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms_ );
  
  // Cartesian position constraints
  A_.block(7,0,3,arm_.getNrOfJoints()) = (J_.data*M_inv_.data).block(0,0,3,arm_.getNrOfJoints());
  x_max_.block(0,0,3,1) = cart_max_constraints_;
  x_min_.block(0,0,3,1) = cart_min_constraints_;
  ubA_.block(7,0,3,1) = (2*(x_max_ - x_curr_ - horizon_dt * J_.data * joint_velocity_in_)/(horizon_dt*horizon_dt) - jdot_qdot_ + J_.data * nonLinearTerms_).block(0,0,3,1);
  lbA_.block(7,0,3,1) = (2*(x_min_ - x_curr_ - horizon_dt * J_.data * joint_velocity_in_)/(horizon_dt*horizon_dt) - jdot_qdot_ + J_.data * nonLinearTerms_).block(0,0,3,1);
  
  // Ec constraint
  Lambda_ = (J_.data * M_inv_.data * J_.data.transpose()).inverse();
  delta_x_ = xd_curr_ * horizon_dt + 0.5 * xdd_des_ * horizon_dt * horizon_dt;
  double ec_curr = 0.5 * xd_curr_.transpose() * Lambda_ * xd_curr_;
  double ec_next = ec_curr + delta_x_.transpose() * Lambda_ * (jdot_qdot_ - J_.data * nonLinearTerms_);

  // Filter estimation of next kinetic energy
  if (!has_first_command_)
    ec_next_filtered_ = ec_next;
  else
    ec_next_filtered_ = 0.95 * ec_next_filtered_ + 0.05 * ec_next;

  A_.block(10,0,1,arm_.getNrOfJoints()) = delta_x_.transpose() * Lambda_ * J_.data * M_inv_.data;
  ubA_(10) = ec_lim_ - ec_next_filtered_;
  lbA_(10) = -100000000.0 - ec_next_filtered_;
  
  // Viscous walls around cartesian constraints
  if(viscous_walls_){
    KDL::Twist twist(KDL::Vector(0,0,0),KDL::Vector(0,0,0));
    Eigen::Matrix<double,6,1> eigen_twist;
    for(int i=0; i<cart_min_constraints_.size(); i++){
      if (xd_curr_(i)<0){
        twist.vel.data[i] = Xd_curr_.vel.data[i];
        tf::twistKDLToEigen(twist, eigen_twist);
        g_ +=  2.0* max_viscous_coeff_*(1-1/(1+std::exp(((x_curr_(i)-viscous_walls_thickness_-cart_min_constraints_(i))*(2/-viscous_walls_thickness_)-1)*6)))* regularisation_weight_ * M_inv_.data *J_.data.transpose()* eigen_twist;
      }
    }
    for(int i=0; i<cart_max_constraints_.size(); i++){
      if (xd_curr_(i)>0){
        twist.vel.data[i] = Xd_curr_.vel.data[i];
        tf::twistKDLToEigen(twist, eigen_twist);
        g_ +=  2.0* max_viscous_coeff_*(1-1/(1+std::exp(((x_curr_(i)+viscous_walls_thickness_-cart_max_constraints_(i))*(2/viscous_walls_thickness_)-1)*6)))* regularisation_weight_ * M_inv_.data *J_.data.transpose()* eigen_twist;
      }
    }
  }
  
  // number of allowed compute steps
  int nWSR = 1e6; 
  
  // Let's compute !
  qpOASES::returnValue ret;
  static bool qpoases_initialized = false;
  
  if(!qpoases_initialized){
    // Initialise the problem, once it has found a solution, we can hotstart
    ret = qpoases_solver_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);
    
    // Keep init if it didn't work
    if(ret == qpOASES::SUCCESSFUL_RETURN)
      qpoases_initialized = true;
  }
  else{
    // Otherwise let's reuse the previous solution to find a solution faster
    ret = qpoases_solver_->hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);
      
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

  // Send torques to the robot
  port_joint_torque_out_.write(joint_torque_out_);
  has_first_command_ = true;
}

void CartOptCtrl::stopHook(){
  has_first_command_ = false;
}
