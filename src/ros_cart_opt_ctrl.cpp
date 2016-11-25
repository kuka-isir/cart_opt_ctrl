#include <cart_opt_ctrl/ros_cart_opt_ctrl.hpp>
#include <pluginlib/class_list_macros.h>

namespace cart_opt_controllers {
  
  std::string getLeafNamespace(const ros::NodeHandle& nh)
  {
    const std::string complete_ns = nh.getNamespace();
    std::size_t id   = complete_ns.find_last_of("/");
    return complete_ns.substr(id + 1);
  }
  
  JointTrajectoryController::JointTrajectoryController()
  {
  }

  bool JointTrajectoryController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
  {
    // Store nodehandle
    controller_nh_ = n;
    name_ = getLeafNamespace(controller_nh_);

    // Initialise the model, the internal solvers etc
    if( ! arm_.init() )
    {
      ROS_ERROR("Could not init chain utils !");
      return false;
    }
    
    // Get joint names
    XmlRpc::XmlRpcValue xml_array;
    if( !controller_nh_.getParam("joints", xml_array) ) {
      ROS_ERROR("No 'joints' parameter in controller (namespace '%s')", controller_nh_.getNamespace().c_str());
      return false;
    }
    // Make sure it's an array type
    if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR("The 'joints' parameter is not an array (namespace '%s')",controller_nh_.getNamespace().c_str());
      return false;
    }

    // Get number of joints
    n_joints_ = xml_array.size();
    ROS_INFO_STREAM("Initializing CartOptCtrl's JointTrajectoryController with "<<n_joints_<<" joints.");

    // Get individual joint properties from urdf and parameter server
    joints_.resize(n_joints_);
    joint_position_in.setZero(n_joints_);
    joint_velocity_in.setZero(n_joints_);
    joint_torque_out.setZero(n_joints_);
    P_gain.resize(6);
    D_gain.resize(6);
    P_joint_gain.resize(n_joints_);
    D_joint_gain.resize(n_joints_);
    qdd_des.resize(n_joints_);
    q_traj.resize(n_joints_);
    qd_traj.resize(n_joints_);
    qdd_traj.resize(n_joints_);
    q_curr.resize(n_joints_);
    qd_curr.resize(n_joints_);
    qdd_curr.resize(n_joints_);
    
    // Get joint names to control & Set ros_control joint handle
    for(int i=0; i<n_joints_; i++) 
    {
      if(xml_array[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR("The 'joint_names' parameter contains a non-string element (namespace '%s')",controller_nh_.getNamespace().c_str());
        return false;
      }
      joints_[i] = robot->getHandle(static_cast<std::string>(xml_array[i]));
    }
    ee_frame = arm_.getSegmentName( arm_.getNrOfSegments() - 1 );
    
    // Default gains, works but stiff
    // TODO Param this
    P_gain << 1000,1000,1000,300,300,300;
    D_gain << 50,50,50,10,10,10;
    P_joint_gain << 450.0, 450.0, 80.0, 450.0, 80.0, 20.0, 1.0;
    D_joint_gain << 20.0, 20.0, 1.5, 20.0, 1.5, 1.0, 0.05;
    
    // Default Alpha value
    // TODO Param this
    Alpha = 1e-03;
    Regularisation = 1e-05;
    
    // QPOases options
    int number_of_variables = n_joints_;
    int number_of_constraints = n_joints_;
    this->qpoases_solver.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints,qpOASES::HST_POSDEF));
    qpOASES::Options options;
    // This options enables regularisation (required) and disable
    // some checks to be very fast !
    // options.setToDefault();
    options.setToMPC(); // setToReliable() // setToDefault()
    options.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
    this->qpoases_solver->setOptions( options );
    this->qpoases_solver->setPrintLevel(qpOASES::PL_NONE); // PL_HIGH for full output, PL_NONE for... none

    // Instantiate solvers
    this->fk_solver_vel_.reset(new KDL::ChainFkSolverVel_recursive(this->arm_.Chain()));
    this->jntToJacDotSolver_.reset(new KDL::ChainJntToJacDotSolver(this->arm_.Chain()));
    
    // Action status checking update rate
    double action_monitor_rate = 20.0;
    controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
    action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
    ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");
    
    action_server_.reset(new ActionServer(controller_nh_, "follow_joint_trajectory",
                                        boost::bind(&JointTrajectoryController::goalCB,   this, _1),
                                        boost::bind(&JointTrajectoryController::cancelCB, this, _1),
                                        false));
    action_server_->start();
    query_state_service_ = controller_nh_.advertiseService("query_state",
                                                  &JointTrajectoryController::queryStateService,
                                                  this);

    return true;
  }

  void JointTrajectoryController::starting(const ros::Time& time) 
  {
    // Define an initial command point from the current position
    trajectory_msgs::JointTrajectory initial_command;
    trajectory_msgs::JointTrajectoryPoint initial_point;
    for(int i=0; i<n_joints_; i++) {
      initial_point.positions.push_back(joints_[i].getPosition());
      initial_point.velocities.push_back(joints_[i].getVelocity());
      initial_point.accelerations.push_back(0.0);
    }
    initial_point.time_from_start = ros::Duration(1.0);
    initial_command.points.push_back(initial_point);
    trajectory_command_buffer_.initRT(initial_command);

    // Set new reference flag for initial command point
    new_reference_ = true;
    has_first_command = false;
    ROS_WARN("Controller started");
  }

  void JointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
  {
    // Read the latest commanded trajectory message
    const trajectory_msgs::JointTrajectory &commanded_trajectory = *(trajectory_command_buffer_.readFromRT());

    // Check for a new reference
    if(new_reference_) {
      // Start trajectory immediately if stamp is zero
      if(commanded_trajectory.header.stamp.isZero()) {
        commanded_start_time_ = time;
      } else {
        commanded_start_time_ = commanded_trajectory.header.stamp;
      }
      // Reset point index
      point_index_ = 0;
      // Reset new reference flag
      new_reference_ = false;

      ROS_WARN("Received new reference.");
    }

    bool trajectory_running = commanded_start_time_ <= time + period;
    bool trajectory_incomplete = point_index_ < commanded_trajectory.points.size();

    // Feed the internal model
    for(unsigned int i=0; i<n_joints_; ++i){
      joint_position_in(i) = joints_[i].getPosition();
      joint_velocity_in(i) = joints_[i].getVelocity();
    }
    arm_.setState(joint_position_in,joint_velocity_in);
    // Make some calculations
    arm_.updateModel();

    // Get Current end effector Pose
    X_curr = arm_.getSegmentPosition(this->ee_frame);
    Xd_curr = arm_.getSegmentVelocity(this->ee_frame);
    q_curr = arm_.getJointPositions();
    qd_curr = arm_.getJointVelocities();

    // Initialize the desired velocity and acceleration to zero
    KDL::SetToZero(Xd_traj);
    KDL::SetToZero(Xdd_traj);
    KDL::SetToZero(qd_traj);
    KDL::SetToZero(qdd_traj);
    
    // If we get a new trajectory point to track
    if(has_first_command && trajectory_running && trajectory_incomplete)
    {
      // Then overrride the desired
      for(unsigned int i=0; i<n_joints_; ++i){
        q_traj(i) = commanded_trajectory.points[point_index_].positions[i];
        qd_traj(i) = commanded_trajectory.points[point_index_].velocities[i];
        qdd_traj(i) = commanded_trajectory.points[point_index_].accelerations[i];
      } 
      
      // Compute FK
      KDL::FrameVel traj_pt_vel;
      KDL::JntArrayVel pt_in_vel;
      pt_in_vel.q = q_traj;
      pt_in_vel.qdot = qd_traj;
      KDL::Twist JdotQdot;

      // If FK or JdotQdot solver fails start over 
      if ((this->fk_solver_vel_->JntToCart(pt_in_vel, traj_pt_vel) == 0) &&  (this->jntToJacDotSolver_->JntToJacDot(pt_in_vel, JdotQdot) == 0) ){
        KDL::Jacobian J = this->arm_.getJacobian();
        KDL::Twist xdotdot;
        
        // Xdd = Jd * qd + J * qdd
        for(unsigned int i = 0; i < 6; ++i )
          xdotdot(i) = JdotQdot(i) + J.data(i) * qdd_traj(i);
      
        // Cartesian trajectory point out
        KDL::FrameAcc traj_pt_out_;
        traj_pt_out_ = KDL::FrameAcc(traj_pt_vel.GetFrame(), traj_pt_vel.GetTwist(), xdotdot);
        X_traj = traj_pt_out_.GetFrame();
        Xd_traj = traj_pt_out_.GetTwist();
        Xdd_traj = traj_pt_out_.GetAccTwist();

        // Increment counter
        point_index_++;
        ROS_DEBUG_STREAM_NAMED(name_,"POINT NB : "<<point_index_<<"/"<< commanded_trajectory.points.size());
      }
      else{
        // Stay at the same position
        X_traj = X_curr;
        q_traj = q_curr;
      }
    } 
    // First step, initialise the first X,Xd,Xdd desired
    else
    {
        // Stay at the same position
        X_traj = X_curr;
        q_traj = q_curr;   
        
        has_first_command = true;
    }
    
    // Compute errors
    X_err = diff( X_curr , X_traj );
    Xd_err = diff( Xd_curr , Xd_traj);
    
    // Apply PD 
    for( unsigned int i=0; i<6; ++i )
    {
        Xdd_des(i) = Xdd_traj(i) + this->P_gain(i) * ( X_err(i) ) + this->D_gain(i) * ( Xd_err(i) );
    }
    for(unsigned int i=0; i< this->n_joints_; ++i){
        qdd_des(i) = qdd_traj(i) + this->P_joint_gain(i) * ( q_traj(i) - q_curr(i) ) + this->D_joint_gain(i) * ( qd_traj(i) - qd_curr(i) );
    }    
    
    Eigen::Matrix<double,6,1> xdd_des;
    tf::twistKDLToEigen(Xdd_des,xdd_des);
        
    KDL::Jacobian& J = arm_.getSegmentJacobian(this->ee_frame);
    KDL::JntSpaceInertiaMatrix& M_inv = arm_.getInertiaInverseMatrix();
    KDL::JntArray& coriolis = arm_.getCoriolisTorque();
    KDL::JntArray& gravity = arm_.getGravityTorque();
    KDL::Twist& Jdotqdot = arm_.getSegmentJdotQdot(this->ee_frame);
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
    a.resize(6,n_joints_);
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> a2;
    a2.resize(n_joints_,n_joints_);
    
    Eigen::Matrix<double,6,1> b;
    Eigen::Matrix<double,Eigen::Dynamic,1> b2;
    b2.resize(n_joints_,1);
    
    a.noalias() = J.data * M_inv.data;
    a2.noalias() = M_inv.data;
    b.noalias() = - a * ( coriolis.data + gravity.data ) + jdot_qdot - xdd_des;
    b2.noalias() = - a2 * ( coriolis.data + gravity.data ) - qdd_des;
    
    // Matrices for qpOASES
    // NOTE: We need RowMajor (see qpoases doc)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
    H.resize(n_joints_,n_joints_);
    
    Eigen::VectorXd g(n_joints_);
    
    Eigen::MatrixXd regularisation;
    regularisation.resize(n_joints_, n_joints_ );
    regularisation.setIdentity();
    regularisation *= Regularisation;
    
    H = 2.0 * a.transpose() * a + Alpha* 2.0 * a2.transpose() * a2 + regularisation;
    g = 2.0 * a.transpose() * b + Alpha* 2.0 * a2.transpose() * b2;
    
    // TODO: get this from URDF
    Eigen::VectorXd torque_max;
    torque_max.resize(n_joints_);
    torque_max << 200,200,100,100,100,30,30; // N.m

    Eigen::VectorXd torque_min;
    torque_min.resize(n_joints_);
    torque_min = -torque_max; // N.m
    
    // Compute bounds
    Eigen::VectorXd lb(n_joints_),
                    ub(n_joints_);
    
    //TODO : write constraints for q and qdot
    lb = torque_min;
    ub = torque_max;
    
    
    // Update Constraints
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(n_joints_,n_joints_);
    Eigen::VectorXd lbA(n_joints_),
                    ubA(n_joints_),
                    qd_min(n_joints_),
                    qd_max(n_joints_);
    
    qd_min.setConstant(-1.0);
    qd_max.setConstant(1.0);
    
    A = M_inv.data;
    
    double horizon_dt = 0.015;
    
    Eigen::VectorXd nonLinearTerms(n_joints_);
    nonLinearTerms = M_inv.data * ( coriolis.data + gravity.data );    
    
    // TODO: adapt this
    // for( int i; i<n_joints_; ++i )
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
                    2*(arm_.getJointLowerLimit() - this->joint_position_in - this->joint_velocity_in * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
    
    ubA = (( qd_max - this->joint_velocity_in ) / horizon_dt + nonLinearTerms).cwiseMin(
                    2*(arm_.getJointUpperLimit() - this->joint_position_in - this->joint_velocity_in * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
    
    
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
        this->joint_torque_out -= gravity.data;
    }  

    // Set the lower-level commands
    for(int i=0; i<n_joints_; i++) {
      // Set the command
      joints_[i].setCommand(joint_torque_out(i));
    }
  }

  void JointTrajectoryController::goalCB(ActionServer::GoalHandle gh){
    ROS_WARN("Goal callback");
    ROS_DEBUG_STREAM_NAMED(name_,"Recieved new action goal");

    // Precondition: Running controller
    if (!this->isRunning())
    {
      ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      gh.setRejected(result);
      return;
    }

    // Interpolate trajectory
    // TODO Find a better way
    trajectory_msgs::JointTrajectory copy_traj, interp_traj;
    copy_traj = gh.getGoal()->trajectory;
    interpolate(copy_traj,interp_traj);
    trajectory_command_buffer_.writeFromNonRT(interp_traj);
    new_reference_ = true;
    
    // TODO Preempt active goal
    // TODO Check goal before setting accepted
//     if (update_ok)
//     {
      // Accept new goal
//       preemptActiveGoal();
      gh.setAccepted();
      RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
      rt_active_goal_ = rt_goal;

      // Setup goal status checking timer
      goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_,
                                                      &RealtimeGoalHandle::runNonRealtime,
                                                      rt_goal);
      goal_handle_timer_.start();
//     }
//     else
//     {
//       // Reject invalid goal
//       control_msgs::FollowJointTrajectoryResult result;
//       result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
//       gh.setRejected(result);
//     }
  }
  void JointTrajectoryController::cancelCB(ActionServer::GoalHandle gh){
    ROS_WARN("Cancel callback is empty for now");
    // TODO Stop robot motion
  }
  
  bool JointTrajectoryController::queryStateService(control_msgs::QueryTrajectoryState::Request&  req, control_msgs::QueryTrajectoryState::Response& resp){
    ROS_WARN("Querying state service is empty for now");
    // TODO fill this in
    return true;                            
  }
  
  
  bool JointTrajectoryController::interpolate(trajectory_msgs::JointTrajectory & trajectory_in, trajectory_msgs::JointTrajectory & trajectory_out)
  {
    //TODO Param this
    double sample_duration_  = 0.001;
    
    
    size_t size_in = trajectory_in.points.size();
    double duration_in = trajectory_in.points.back().time_from_start.toSec();
    double interpolated_time = 0.0;
    size_t index_in = 0;

    trajectory_msgs::JointTrajectoryPoint p1, p2, interp_pt;

    trajectory_out = trajectory_in;
    trajectory_out.points.clear();

    while (interpolated_time < duration_in)
    {
      // Increment index until the interpolated time is past the start time.
      while (interpolated_time > trajectory_in.points[index_in + 1].time_from_start.toSec())
      {
        index_in++;
        if (index_in >= size_in)
          return false;

      }
      p1 = trajectory_in.points[index_in];
      p2 = trajectory_in.points[index_in + 1];
      if (!interpolatePt(p1, p2, interpolated_time, interp_pt))
        return false;
        
      trajectory_out.points.push_back(interp_pt);
      interpolated_time += sample_duration_;
    }
    p2 = trajectory_in.points.back();
    p2.time_from_start = ros::Duration(interpolated_time);
    trajectory_out.points.push_back(p2);
    return true;
  }


  bool JointTrajectoryController::interpolatePt(trajectory_msgs::JointTrajectoryPoint & p1,
                                              trajectory_msgs::JointTrajectoryPoint & p2, double time_from_start,
                                              trajectory_msgs::JointTrajectoryPoint & interp_pt)
  {
    double p1_time_from_start = p1.time_from_start.toSec();
    double p2_time_from_start = p2.time_from_start.toSec();

    if (time_from_start >= p1_time_from_start && time_from_start <= p2_time_from_start)
    {
      if (p1.positions.size() == p1.velocities.size() && p1.positions.size() == p1.accelerations.size())
      {
        if (p1.positions.size() == p2.positions.size() && p1.velocities.size() == p2.velocities.size()
            && p1.accelerations.size() == p2.accelerations.size())
        {
          // Copy p1 to ensure the interp_pt has the correct size vectors
          interp_pt = p1;

          KDL::VelocityProfile_Spline spline_calc;

          for (size_t i = 0; i < p1.positions.size(); ++i)
          {
            // Calculated relative times for spline calculation
            double time_from_p1 = time_from_start - p1.time_from_start.toSec();
            double time_from_p1_to_p2 = p2_time_from_start - p1_time_from_start;

            spline_calc.SetProfileDuration(p1.positions[i], p1.velocities[i], p1.accelerations[i], p2.positions[i],
                                            p2.velocities[i], p2.accelerations[i], time_from_p1_to_p2);

            ros::Duration time_from_start_dur(time_from_start);

            interp_pt.time_from_start = time_from_start_dur;
            interp_pt.positions[i] = spline_calc.Pos(time_from_p1);
            interp_pt.velocities[i] = spline_calc.Vel(time_from_p1);
            interp_pt.accelerations[i] = spline_calc.Acc(time_from_p1);
          }
          return true;
        }
        else
          return false;
      }
      else
        return false;
    }
    else
      return false;
  }
} // namespace

PLUGINLIB_EXPORT_CLASS(cart_opt_controllers::JointTrajectoryController, controller_interface::ControllerBase)