#include "cart_opt_ctrl/cart_opt_comp.hpp"

using namespace RTT;
using namespace KDL;

CartOptCtrl::CartOptCtrl(const std::string& name):RTT::TaskContext(name)
{
  // Orocos ports
  this->addPort("JointPosition",port_joint_position_in_);
  this->addPort("JointVelocity",port_joint_velocity_in_);
  this->addPort("JointTorqueCommand",port_joint_torque_out_);
  this->addPort("JointTorqueInfo",port_joint_torque_);
  this->addPort("TrajectoryPointPosIn",port_pnt_pos_in_);
  this->addPort("TrajectoryPointVelIn",port_pnt_vel_in_);
  this->addPort("TrajectoryPointAccIn",port_pnt_acc_in_);
  this->addPort("PoseDesired",port_x_des_);
  this->addPort("PoseTarget",port_x_target_);
  this->addPort("PoseCurrent",port_x_mes_);
  this->addPort("JointPosVelIn",port_joint_pos_vel_in_);
  this->addPort("PoseErrorOut",port_error_out_);
  this->addPort("ButtonPressed",port_button_pressed_in_);
  this->addPort("Ec_constraint",port_Ec_constraints);
  this->addPort("force_info",port_force_info);
  this->addPort("delta_x",port_delta_x_info);
  this->addPort("XddDes",port_xdd_des_);
  this->addPort("Xd_Out",port_Xd_out_);
  this->addPort("positioning_error",port_positioning_error);
  this->addPort("pointing_error",port_pointing_error);
  this->addPort("kp_x_err",port_kp_x_err);
  this->addPort("Xdd_out",port_Xdd_out);
  this->addPort("Integral_term",port_integral_term);
  this->addPort("Xdd_des_const",port_xdd_des_const_);
  this->addPort("Xd_curr_const",port_Xd_out_const_);
  this->addPort("Ec_constraint2",port_Ec_constraint2);


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
  this->addProperty("torque_max",torque_max_).doc("Max torque for each joint");
  this->addProperty("joint_vel_max",jnt_vel_max_).doc("Max velocity for each joint");
  this->addProperty("integral_pos_saturation",integral_pos_saturation).doc("Position Saturation of the integral term");
  this->addProperty("integral_ori_saturation",integral_ori_saturation).doc("Orientation Saturation of the integral term");
  this->addProperty("Ec_lim",Ec_lim).doc("Potential energy limit");
  this->addProperty("F_lim",F_lim).doc("Maximum pushing force");
  this->addProperty("horizon",horizon).doc("Time horizon (ms)");
  this->addProperty("rotation",rotation).doc("Time horizon (ms)");
  this->addProperty("static_pointing",static_pointing).doc("static = true , moving = false");
  this->addProperty("point_to_target",point_to_target).doc("Point to target");

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
  this->addOperation("getPointingPose",&CartOptCtrl::getPointingPose,this,RTT::ClientThread);
}

bool CartOptCtrl::getCurrentPose(cart_opt_ctrl::GetCurrentPose::Request& req, cart_opt_ctrl::GetCurrentPose::Response& resp){
  tf::poseKDLToMsg(X_curr_,resp.current_pose);
  resp.success = true;
  return true;
}

bool CartOptCtrl::getPointingPose(cart_opt_ctrl::GetCurrentPose::Request& req, cart_opt_ctrl::GetCurrentPose::Response& resp){
  KDL::Frame link_6_pos = arm_.getSegmentPosition(link_6_frame);
  frame_target = Frame(Rotation::RPY(0.0,0.0,0.0),Vector(target(0),target(1),target(2)));
  KDL::Rotation rotz = PointTarget(frame_target,X_curr_);
  tf::poseKDLToMsg(KDL::Frame(rotz,X_curr_.p),resp.current_pose);
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
  KDL::Vector axe_curr_z = frame_des.M * KDL::Vector(0,0,1);

  //Calcul de l'angle entre la direction actuelle et la direction desirée
  double theta = acos(KDL::dot(dir_des,axe_curr_z));

  //Calcul de l'axe de rotation autour des z
  KDL::Vector axe_rot = axe_curr_z*dir_des;
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

 double CartOptCtrl::KineticEnergy()
  {
    Eigen::Matrix<double,6,6> Lambda;
    Eigen::Matrix<double,6,1> Xd_mes_;
    KDL::Jacobian jacobian = arm_.getSegmentJacobian(ee_frame_);
    KDL::Twist Xd_mes = arm_.getSegmentVelocity(ee_frame_);
    tf::twistKDLToEigen(Xd_mes,Xd_mes_);
    KDL::JntSpaceInertiaMatrix mass_kdl_inverse = arm_.getInertiaInverseMatrix();

    Lambda = ( jacobian.data * mass_kdl_inverse.data * jacobian.data.transpose() ).inverse();

    return 0.5 * Xd_mes_.transpose() * Lambda * Xd_mes_;
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
  regularisation_weight_.resize(dof);
  damping_weight_.resize(dof);
  perturbation = false;
  for(int i = 0; i<select_components_.size() ; i++){
    select_components_[i].resize(6);
    select_axes_[i].resize(dof);
  }
  joint_torque_out_.setZero(dof);
  joint_position_in_.setZero(dof);
  joint_velocity_in_.setZero(dof);

  // Default params
  ee_frame_ = arm_.getSegmentName( arm_.getNrOfSegments() - 1 );

  link_6_frame = "link_6";
  target.resize(3);
  p_gains_ << 1000,1000,1000,1000,1000,1000;
  d_gains_ << 22,22,22,22,22,22;
  integral_pos_saturation = 0.0;
  integral_ori_saturation = 0.0;
  position_saturation_ = 0.01;
  orientation_saturation_ = M_PI/100;
  horizon = 15;
  regularisation_weight_ << 1e-05,1e-05,1e-05,1e-05,1e-05,1e-05,1e-05;
  damping_weight_  << 1.0,1.0,1.0,1.0,1.0,1.0,1.0;
  compensate_gravity_ = true;
  static_pointing = false;
  point_to_target = false;
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
  number_of_constraints = dof+1;
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
  // [n,Wn] = buttord(40/500,150/500,3,60)
  // [b,a] = butter(n,Wn)

  a_butter.setZero(6);b_butter.setZero(6);
  a_butter << 1.00000 , -4.18692 ,  7.06849 , -6.00846 ,  2.56961 , -0.44204;
  b_butter << 2.1443e-05 ,  1.0721e-04 ,  2.1443e-04 ,  2.1443e-04 ,  1.0721e-04 ,  2.1443e-05;
  filtered_acc.setZero();
  filtered_vel.setZero();
  old_acc.setZero();
  old_vel.setZero();

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

  if (static_pointing)
  {
    target << X_traj_.p.x() ,0.0 ,0.0;
  }
  else
    target << X_traj_.p.x() , X_traj_.p.y() ,0.0;

  KDL::Frame link_6_pos = arm_.getSegmentPosition(link_6_frame);
  frame_target = Frame(Rotation::RPY(0.0,0.0,0.0),Vector(target(0),target(1),target(2)));
  KDL::Rotation rotz = PointTarget(frame_target,link_6_pos);

  // If we get a new trajectory point to track
  if((port_pnt_pos_in_.read(pt_pos_in_) != RTT::NoData) && (port_pnt_vel_in_.read(pt_vel_in_) != RTT::NoData) && (port_pnt_acc_in_.read(pt_acc_in_) != RTT::NoData)){
    // Then overwrite the desired
    X_traj_ = pt_pos_in_;
    Xd_traj_ = pt_vel_in_;
    Xdd_traj_ = pt_acc_in_;
//
    has_first_command_ = true;
//     X_traj_ = KDL::Frame(rotz,pt_pos_in_.p);
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
//     X_traj_ = KDL::Frame(rotz,X_curr_.p);
  }

  // Compute errors
  X_err_ = diff( X_curr_ , X_traj_ );
  Xd_err_ = diff( Xd_curr_ , Xd_traj_);

  // Debug publish in ROS
  geometry_msgs::Pose x_des_pos_out;
  tf::poseKDLToMsg(X_traj_,x_des_pos_out);
  geometry_msgs::PoseStamped x_des_pos_stamped_out;
  x_des_pos_stamped_out.pose = x_des_pos_out;
  x_des_pos_stamped_out.header.frame_id = "table_link";
  x_des_pos_stamped_out.header.stamp = rtt_rosclock::host_now();
  port_x_des_.write(x_des_pos_stamped_out);

  geometry_msgs::Pose x_mes_pos_out;
  tf::poseKDLToMsg(X_curr_,x_mes_pos_out);
  geometry_msgs::PoseStamped x_mes_pos_stamped_out;
  x_mes_pos_stamped_out.pose = x_mes_pos_out;
  x_mes_pos_stamped_out.header.frame_id = "table_link";
  x_mes_pos_stamped_out.header.stamp = rtt_rosclock::host_now();
  port_x_mes_.write(x_mes_pos_stamped_out);

  geometry_msgs::Pose x_target_pos_out;
  tf::poseKDLToMsg(frame_target,x_target_pos_out);
  geometry_msgs::PoseStamped x_target_pos_stamped_out;
  x_target_pos_stamped_out.pose = x_target_pos_out;
  x_target_pos_stamped_out.header.frame_id = "table_link";
  x_target_pos_stamped_out.header.stamp = rtt_rosclock::host_now();
  port_x_target_.write(x_target_pos_stamped_out);


  trajectory_msgs::JointTrajectoryPoint joint_pos_vel;
  for(int i=0; i< arm_.getNrOfJoints(); i++){
    joint_pos_vel.positions.push_back(joint_position_in_(i)*180.0/3.14159);
    joint_pos_vel.velocities.push_back(joint_velocity_in_(i)*180.0/3.14159);
  }

//     port_joint_pos_vel_in_.write(joint_torque_out_);

  geometry_msgs::Twist Xd_twist_ros;
  tf::twistKDLToMsg(Xd_curr_, Xd_twist_ros);
  port_Xd_out_.write(Xd_twist_ros);

  // Projection on target plan
  Eigen::Matrix<double,2,1> Xx;
  Xx = GetPointingTarget(Frame(X_curr_.M,X_curr_.p),frame_target);
  pointing_error.data = sqrt(pow(target(0)-Xx(0),2)+pow(target(1)-Xx(1),2))*1000; // Pointing_error in mm;

  positioning_error.data=X_err_.vel.Norm();

  port_pointing_error.write(pointing_error);
  port_positioning_error.write(positioning_error);


  // Saturate the pose error
  for(unsigned int i=0; i<3; ++i ){
    if(X_err_(i) >0)
      X_err_(i) = std::min( position_saturation_, X_err_(i));
    else
      X_err_(i) = std::max(-position_saturation_, X_err_(i));
  }
  for(unsigned int i=3; i<6; ++i ){
    if(X_err_(i) >0)
      X_err_(i) = std::min( orientation_saturation_, X_err_(i));
    else
      X_err_(i) = std::max(-orientation_saturation_, X_err_(i));
  }

  //Integral term and saturation
  for( unsigned int i=0; i<3; ++i )
  {
     integral_term(i) += X_err_(i);
     if (i_gains_(i)*integral_term(i)>0)
       integral_term_sat(i) = std::min(i_gains_(i)*integral_term(i),integral_pos_saturation);
     else
       integral_term_sat(i) = std::max(i_gains_(i)*integral_term(i),-integral_pos_saturation);
  }
    for( unsigned int i=3; i<6; ++i )
  {
     integral_term(i) += X_err_(i);
     if (i_gains_(i)*integral_term(i)>0)
       integral_term_sat(i) = std::min(i_gains_(i)*integral_term(i),integral_ori_saturation);
     else
       integral_term_sat(i) = std::max(i_gains_(i)*integral_term(i),-integral_ori_saturation);
  }

  tf::twistKDLToMsg(integral_term_sat,integral_term_msg);
  port_integral_term.write(integral_term_msg);


  geometry_msgs::Twist error_twist_ros;
  tf::twistKDLToMsg(X_err_, error_twist_ros);
  port_error_out_.write(error_twist_ros);

  // Apply PID
  for( unsigned int i=0; i<3; ++i )
    Xdd_des(i) = Xdd_traj_(i) + p_gains_(i) * ( X_err_(i) ) - d_gains_(i) * ( Xd_curr_(i) ) + integral_term_sat(i) ;
  for( unsigned int i=3; i<6; ++i )
    Xdd_des(i) = 		p_gains_(i) * ( X_err_(i) ) - d_gains_(i) * ( Xd_curr_(i) ) + integral_term_sat(i) ;


  Eigen::Matrix<double,6,1> xdd_des;
  tf::twistKDLToEigen(Xdd_des,xdd_des);
  tf::twistKDLToMsg(Xdd_des,xdd_des_msg);

  // Buterworth filter

   for (int i = 0; i<6 ; i++)
   {
    filtered_acc(5,i) = 0;
    filtered_vel(5,i) = 0;
    old_acc(5,i) =Xdd_des(i);
    old_vel(5,i) =Xd_curr_(i);
    for (int k = 0; k<6 ; k++)
    {
      filtered_acc(5,i) += b_butter(k) * old_acc(5-k,i);
      filtered_vel(5,i) += b_butter(k) * old_vel(5-k,i);
    }
    for (int l = 1; l<6 ; l++)
    {
      filtered_acc(5,i) -= a_butter(l) * filtered_acc(5-l,i);
      filtered_vel(5,i) -= a_butter(l) * filtered_vel(5-l,i);
    }

    for (int j = 0;j<5 ; j++)
    {
      old_acc(j,i) = old_acc(j+1,i);
      old_vel(j,i) = old_vel(j+1,i);
      filtered_acc(j,i) = filtered_acc(j+1,i);
      filtered_vel(j,i) = filtered_vel(j+1,i);
    }

    Xdd_filtered = filtered_acc.block(0,0,1,6).transpose();
    X_curr_filtered = filtered_vel.block(0,0,1,6).transpose();
    tf::matrixEigenToMsg(Xdd_filtered,xdd_des_const_float_msg);    

   }


  //tf::twistKDLToMsg(Xdd_des,xdd_des_const_msg);

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
  // With a = J.Minv
  //      b = - J.Minv.( B + G ) + Jdot.qdot - Xdd_des

//   Eigen::MatrixXd regularisation = regularisation_weight_.asDiagonal() * M_inv.data;
  Eigen::MatrixXd regularisation = regularisation_weight_[0] * M_inv.data;
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
  button_selection.setZero();
  if (button_pressed_){
    transition_gain_ = 0.0;
    button_selection = Eigen::MatrixXd::Identity(6,6);
    button_selection.topLeftCorner(3,3).setZero();
  }
  else{
    transition_gain_ = std::min(1.0,transition_gain_ + 0.001 * regularisation_weight_[0]);
    button_selection = Eigen::MatrixXd::Identity(6,6);
  }


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

    H += transition_gain_ * 2.0 * /*button_selection **/ a.transpose() * select_cartesian_component * a;
    g += transition_gain_ * 2.0 * /*button_selection **/ a.transpose() * select_cartesian_component * b;
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

   Eigen::Matrix<double,6,6> Lambda;
   Lambda.setZero();

   Lambda = (J.data*M_inv.data*J.data.transpose()).inverse();

   double Ec_curr = KineticEnergy();
   Eigen::Matrix<double,6,1> Xd_curr;Xd_curr.setZero();
   Eigen::Matrix<double,1,6> delta_x;delta_x.setZero();
   tf::twistKDLToEigen( Xd_curr_ , Xd_curr);
   geometry_msgs::Twist Xd_twist_ros_filt;
//    std_msgs::Float64MultiArray Xd_twist_ros_filt;
   tf::twistKDLToMsg(Xd_curr_, Xd_twist_ros_filt);
   //tf::matrixEigenToMsg(J.data*joint_velocity_in_, Xd_twist_ros_filt);
   port_Xd_out_const_.write(Xd_twist_ros_filt);


   double horizon_ec = sqrt(2.0 * Ec_lim/(F_lim*(p_gains_(0) * position_saturation_ + integral_pos_saturation)));
//    cout << horizon_ec << endl;
   delta_x = (horizon_ec * X_curr_filtered + 0.5 * horizon_ec * horizon_ec * Xdd_filtered).transpose();

   Eigen::Matrix<double,1,7> A_ec;
   double B_ec;

   A_ec = delta_x * Lambda * J.data * M_inv.data;
   B_ec = Ec_curr + delta_x * Lambda * (jdot_qdot - J.data * M_inv.data *(coriolis.data + gravity.data));

   lbA.block(0,0,7,1) = (( qd_min - joint_velocity_in_ ) / horizon_dt + nonLinearTerms).cwiseMax(
   2*(arm_.getJointLowerLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );

   ubA.block(0,0,7,1) = (( qd_max - joint_velocity_in_ ) / horizon_dt + nonLinearTerms).cwiseMin(
  2*(arm_.getJointUpperLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );

  lbA(7) = 0.0 - B_ec;
  ubA(7) = Ec_lim - B_ec;


  A.block(0,0,7,7) = arm_.getInertiaInverseMatrix().data;
  A.block(7,0,1,7) = A_ec;

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


  tf::matrixEigenToMsg(delta_x,delta_x_msg);

  Eigen::Matrix<double,4,1> Ec_constraints;
  Ec_constraints(0) = Ec_constraint;
  Ec_constraints(1) = Ec_lim;
  Ec_constraints(2) = horizon_ec;
  Ec_constraints(3) = F_lim;
  tf::matrixEigenToMsg(Ec_constraints,Ec_constraints_msg);
  port_Ec_constraints.write(Ec_constraints_msg);

  Eigen::Matrix<double,6,1> Xdd_out;
  Xdd_out = jdot_qdot + J.data * M_inv.data * ( joint_torque_out_ - coriolis.data );
  tf::matrixEigenToMsg(Xdd_out,Xdd_out_msg);
  port_Xdd_out.write(Xdd_out_msg);

  Eigen::Matrix<double,6,1> force_info;
  force_info = Lambda *Xdd_out;
  tf::matrixEigenToMsg(force_info,force_info_msg);

  Eigen::Matrix<double,3,1> kp_x_err;
  for (int i=0;i<3;i++)
    kp_x_err(i) = p_gains_(i) * ( X_err_(i) );
  tf::matrixEigenToMsg(kp_x_err,kp_x_err_msg);
  port_kp_x_err.write(kp_x_err_msg);

  Ec_constraints(0) = Ec_constraint;
  Ec_constraints(1) = Ec_lim;
  tf::matrixEigenToMsg(Ec_constraints,Ec_constraints_msg);
  port_Ec_constraints.write(Ec_constraints_msg);


  Ec_constraint2.data = Ec_curr ;

  port_Ec_constraint2.write(Ec_constraint2);

  port_force_info.write(force_info_msg);
  port_delta_x_info.write(delta_x_msg);
  port_xdd_des_.write(xdd_des_msg);
  port_xdd_des_const_.write(xdd_des_const_float_msg);

  std_msgs::Float64MultiArray joint_torque_msg;
  tf::matrixEigenToMsg(joint_torque_out_,joint_torque_msg);
  port_joint_torque_.write(joint_torque_msg);

  // Send torques to the robot
  port_joint_torque_out_.write(joint_torque_out_);


}


void CartOptCtrl::stopHook(){
  has_first_command_ = false;
}
