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
  this->addProperty("d_gains",d_gains_).doc("Derivative gains");
  this->addProperty("position_saturation",position_saturation_).doc("Position saturation");
  this->addProperty("orientation_saturation",orientation_saturation_).doc("Orientation saturation");
  this->addProperty("regularisation_weight",regularisation_weight_).doc("Weight for the regularisation in QP");
  this->addProperty("compensate_gravity",compensate_gravity_).doc("Do we need to compensate gravity ?");
  this->addProperty("damping_weight",damping_weight_).doc("Weight for the damping in regularisation");
  this->addProperty("torque_max",torque_max_).doc("Max torque for each joint");
  this->addProperty("joint_vel_max",jnt_vel_max_).doc("Max velocity for each joint");
  this->addProperty("cart_min_constraints",cart_min_constraints_).doc("Max velocity for each joint");
  this->addProperty("cart_max_constraints",cart_max_constraints_).doc("Max velocity for each joint");
  
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
  
  // Resize the vectors
  p_gains_.resize(6);
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
  joint_torque_out_.setZero(dof);
  joint_position_in_.setZero(dof);
  joint_velocity_in_.setZero(dof);
  
  // Default params
  ee_frame_ = arm_.getSegmentName( arm_.getNrOfSegments() - 1 );
  p_gains_ << 1000,1000,1000,1000,1000,1000;
  d_gains_ << 22,22,22,22,22,22;
  position_saturation_ = 0.01;
  orientation_saturation_ = M_PI/100;
  regularisation_weight_ = 1e-05;
  damping_weight_  << 1.0,1.0,1.0,1.0,1.0,1.0,1.0;
  cart_min_constraints_.setConstant(3, -10.0);
  cart_max_constraints_.setConstant(3, 10.0);
  compensate_gravity_ = true;
  for(int i = 1; i<select_components_.size() ; i++){
    select_components_[i].setZero(6);
    select_axes_[i].setZero(dof);
  }
  select_components_[1].setOnes(6);
  select_axes_[1].setOnes(dof);
  // TODO: get this from URDF
  torque_max_ << 175,175,99,99,99,37,37 ;
  jnt_vel_max_ << 1.0,1.0,1.0,1.0,1.0,1.0,1.0;
  
  // Match all properties (defined in the constructor) 
  // with the rosparams in the namespace : 
  // nameOfThisComponent/nameOftheProperty
  // Equivalent to ros::param::get("CartOptCtrl/p_gains_");
  rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);
  
  // For now we have 0 constraints for now
  int number_of_variables = dof;
  number_of_constraints_ = dof + 3;
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
      
    has_first_command_ = true;
  }
  else{
    log(RTT::Warning) << "Trajectory ports empty !" << endlog();
    return;
  }
  
  // First step, initialise the first X,Xd,Xdd desired
  if(!has_first_command_){
    // Stay at the same position
    X_traj_ = X_curr_;
    has_first_command_ = true;
  }
  
  // Compute errors
  X_err_ = diff( X_curr_ , X_traj_ );
  Xd_err_ = diff( Xd_curr_ , Xd_traj_);
  
  // Debug publish in ROS
  geometry_msgs::Pose x_des_pos_out;
  tf::poseKDLToMsg(X_traj_,x_des_pos_out);
  geometry_msgs::PoseStamped x_des_pos_stamped_out;
  x_des_pos_stamped_out.pose = x_des_pos_out;
  x_des_pos_stamped_out.header.frame_id = base_frame_;
  x_des_pos_stamped_out.header.stamp = rtt_rosclock::host_now();
  port_x_des_.write(x_des_pos_stamped_out);
  trajectory_msgs::JointTrajectoryPoint joint_pos_vel;
  for(int i=0; i< arm_.getNrOfJoints(); i++){
    joint_pos_vel.positions.push_back(joint_position_in_(i));
    joint_pos_vel.velocities.push_back(joint_velocity_in_(i));
  }
  port_joint_pos_vel_in_.write(joint_pos_vel);
  geometry_msgs::Twist error_twist_ros;
  tf::twistKDLToMsg(X_err_, error_twist_ros);
  port_error_out_.write(error_twist_ros);
  
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
  
  // Apply PD 
  for( unsigned int i=0; i<6; ++i )
    Xdd_des(i) = Xdd_traj_(i) + p_gains_(i) * ( X_err_(i) ) - d_gains_(i) * ( Xd_curr_(i) );
  
  Eigen::Matrix<double,6,1> xdd_des;
  tf::twistKDLToEigen(Xdd_des,xdd_des);
      
  KDL::Jacobian& J = arm_.getSegmentJacobian(ee_frame_);
  
  KDL::JntSpaceInertiaMatrix& M_inv = arm_.getInertiaInverseMatrix();
  
  KDL::JntArray& coriolis = arm_.getCoriolisTorque();
  
  KDL::JntArray& gravity = arm_.getGravityTorque();
  
  KDL::Twist& Jdotqdot = arm_.getSegmentJdotQdot(ee_frame_);
  
  Eigen::Matrix<double,6,1> jdot_qdot;
  tf::twistKDLToEigen(Jdotqdot,jdot_qdot);
  
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
  // With a = 00001J.Minv
  //      b = - J.Minv.( B + G ) + Jdot.qdot - Xdd_des

  Eigen::MatrixXd regularisation = regularisation_weight_* M_inv.data;
  Eigen::MatrixXd damping = damping_weight_.asDiagonal();
  
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
  
  // Read button press port
  if(this->port_button_pressed_in_.read(button_pressed_msg_) != RTT::NoData){
    button_pressed_ = button_pressed_msg_.data;
  }
  
  // If button is pressed leave only the regularisation task
  // Then progressively introduce the cartesian task
  if (button_pressed_)
    transition_gain_ = 0.0;
  else
    transition_gain_ = std::min(1.0,transition_gain_ + 0.001 * regularisation_weight_);
  
  // Write cartesian tasks
  // The cartesian tasks can be decoupling by axes
  Eigen::Matrix<double,6,Eigen::Dynamic> a;
  a.resize(6,arm_.getNrOfJoints());
  Eigen::Matrix<double,6,1> b;
  Eigen::MatrixXd select_axis, select_cartesian_component;
  for(int i=0; i<select_components_.size();i++){
    select_axis = select_axes_[i].asDiagonal();
    select_cartesian_component = select_components_[i].asDiagonal();
    
    a.noalias() =  J.data * select_axis * M_inv.data;
    b.noalias() = (- a * ( coriolis.data + gravity.data ) + jdot_qdot - xdd_des);
    
    H += transition_gain_ * 2.0 * a.transpose() * select_cartesian_component * a;  
    g += transition_gain_ * 2.0 * a.transpose() * select_cartesian_component * b;
  }
  
  // Compute bounds
  Eigen::VectorXd lb(arm_.getNrOfJoints()),
		  ub(arm_.getNrOfJoints());
  lb = -torque_max_;
  ub = torque_max_;    
  
  // Update Constraints
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(number_of_constraints_,arm_.getNrOfJoints());
  Eigen::VectorXd lbA(number_of_constraints_),
		  ubA(number_of_constraints_),
		  qd_min(arm_.getNrOfJoints()),
		  qd_max(arm_.getNrOfJoints());
  
  qd_max = jnt_vel_max_;
  qd_min = -jnt_vel_max_;
  
  A.block(0,0,7,7) = arm_.getInertiaInverseMatrix().data;
  A.block(7,0,3,7) = (J.data*arm_.getInertiaInverseMatrix().data).block(0,0,3,7);
  
  // TODO Param this ???
  double horizon_dt = 0.015;
  
  Eigen::VectorXd nonLinearTerms(arm_.getNrOfJoints());
  nonLinearTerms = arm_.getInertiaInverseMatrix().data * ( coriolis.data + gravity.data );    
  
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

  lbA.block(0,0,arm_.getNrOfJoints(),1) = (( qd_min - joint_velocity_in_ ) / horizon_dt + nonLinearTerms).cwiseMax(
      2*(arm_.getJointLowerLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
  
  ubA.block(0,0,arm_.getNrOfJoints(),1) = (( qd_max - joint_velocity_in_ ) / horizon_dt + nonLinearTerms).cwiseMin(
      2*(arm_.getJointUpperLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
  
  Eigen::Matrix<double,6,1> lbA_cart, ubA_cart;
  Eigen::VectorXd x_max, x_min;
  x_max.setZero(6);
  x_min.setZero(6);
  x_max.block(0,0,3,1) = cart_max_constraints_;
  x_min.block(0,0,3,1) = cart_min_constraints_;
  
  Eigen::Matrix<double,6,1> xd_curr, x_curr;
  Eigen::Matrix<double,3,1> x_curr_lin;
  tf::twistKDLToEigen(Xd_curr_, xd_curr);
  tf::vectorKDLToEigen(X_curr_.p, x_curr_lin);
  x_curr.block(0,0,3,1) = x_curr_lin;

  ubA_cart = ((x_max - x_curr - horizon_dt * J.data * xd_curr)/(horizon_dt*horizon_dt) - jdot_qdot + J.data * M_inv.data *(coriolis.data + gravity.data)).block(0,0,3,1);
  lbA_cart = ((x_min - x_curr - horizon_dt * J.data * xd_curr)/(horizon_dt*horizon_dt) - jdot_qdot + J.data * M_inv.data *(coriolis.data + gravity.data)).block(0,0,3,1);
  
  lbA.block(7,0,3,1) = lbA_cart;
  ubA.block(7,0,3,1) = ubA_cart;
  
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

  // Send torques to the robot
  port_joint_torque_out_.write(joint_torque_out_);
}


void CartOptCtrl::stopHook(){
  has_first_command_ = false;
}
