#include <cart_opt_ctrl/cart_traj_interp.hpp>

CartTrajInterp::CartTrajInterp(std::string const& name) : TaskContext(name){
  this->addPort("TrajectoryIn",this->port_traj_in_);
  this->addPort("JointPosition",this->port_joint_position_in_);
  this->addPort("JointVelocity",this->port_joint_velocity_in_);
  this->addPort("TrajectoryPointOut",this->port_traj_pt_out_);
  this->addPort("TrajectoryJointOut",this->port_traj_joint_out_);
  
  this->addProperty("SampleDuration",this->sample_duration_).doc("The time between each point to send");
}

bool CartTrajInterp::configureHook(){
  // Initialise the model, the internal solvers etc
  if( !this->arm_.init() )
  {
    RTT::log(RTT::Error) << "Could not init chain utils !" << RTT::endlog();
    return false;
  }
  
  // The number of joints
  dof_ = this->arm_.getNrOfJoints();
  
  // Resize data
  pt_in_.q.data.resize(dof_);
  pt_in_.qdot.data.resize(dof_);
  pt_in_.qdotdot.data.resize(dof_);
  
  // Instantiate solvers
  this->fk_solver_vel_.reset(new KDL::ChainFkSolverVel_recursive(this->arm_.Chain()));
  this->jntToJacDotSolver_.reset(new KDL::ChainJntToJacDotSolver(this->arm_.Chain()));
  
  this->traj_pt_nb_ = 0;
  
  // Sample duration default value
  this->sample_duration_ = 0.001;
  
  // Match all properties (defined in the constructor) 
  // with the rosparams in the namespace : 
  // nameOfThisComponent/nameOftheProperty
  // Equivalent to ros::param::get("CartOptCtrl/P_gain");
  rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);
  
  return true;
}

void CartTrajInterp::updateHook(){
  
  // Read the ports
  RTT::FlowStatus ftraj = this->port_traj_in_.read(this->traj_in_);
  RTT::FlowStatus fp = this->port_joint_position_in_.read(this->joint_position_in_);
  RTT::FlowStatus fv = this->port_joint_velocity_in_.read(this->joint_velocity_in_);
  
  // If we get a new trajectory update it and start with point 0
  if(ftraj == RTT::NewData)
  {
    RTT::log(RTT::Info) << "New trajectory with "<<traj_in_.result.planned_trajectory.joint_trajectory.points.size() <<" points" << RTT::endlog();
    if(!this->interpolate(traj_in_.result.planned_trajectory.joint_trajectory, traj_curr_)){
      RTT::log(RTT::Error) << "Interpolation of the trajectory failed" << RTT::endlog();
      return;
    }
    RTT::log(RTT::Info) << "Interpolated traj with "<<traj_curr_.points.size() <<" points" << RTT::endlog();
    this->traj_pt_nb_ = 0;
  }

  // Return if not giving any robot update (might happend during startup)
  if(fp == RTT::NoData || fv == RTT::NoData)
    return
  
  // Update the chain model
  this->arm_.setState(this->joint_position_in_,this->joint_velocity_in_);
  this->arm_.updateModel();
  
  // Check trajectory is not empty
  if (this->traj_curr_.points.size() < 1)
    return;
  
  // Check trajectory is not finished
  if (this->traj_pt_nb_ > this->traj_curr_.points.size() - 1)
    return;
  
  // Pick current point in trajectory     
  for(unsigned int i = 0; i < this->dof_; ++i){
    this->pt_in_.q(i) = this->traj_curr_.points[this->traj_pt_nb_].positions[i];
    this->pt_in_.qdot(i) = this->traj_curr_.points[this->traj_pt_nb_].velocities[i] ;
    this->pt_in_.qdotdot(i) = this->traj_curr_.points[this->traj_pt_nb_].accelerations[i] ;
  }
  
  // Compute FK
  KDL::FrameVel traj_pt_vel;
  KDL::JntArrayVel pt_in_vel;
  pt_in_vel.q = this->pt_in_.q;
  pt_in_vel.qdot = this->pt_in_.qdot;
  KDL::Twist JdotQdot;
  
  // If FK or JdotQdot solver fails start over 
  if ((this->fk_solver_vel_->JntToCart(pt_in_vel, traj_pt_vel) == 0) &&  (this->jntToJacDotSolver_->JntToJacDot(pt_in_vel, JdotQdot) == 0) ){
        
    KDL::Jacobian J = this->arm_.getJacobian();
    KDL::Twist xdotdot;
    
    // Xdd = Jd * qd + J * qdd
    for(unsigned int i = 0; i < 6; ++i )
      xdotdot(i) = JdotQdot(i) + J.data(i) * this->pt_in_.qdotdot(i);
    
    this->traj_pt_out_ = KDL::FrameAcc(traj_pt_vel.GetFrame(), traj_pt_vel.GetTwist(), xdotdot);
    
    // Send cartesian trajectory point to the controller
    this->port_traj_pt_out_.write(this->traj_pt_out_);
    this->port_traj_joint_out_.write(this->pt_in_);

    // Increment counter
    this->traj_pt_nb_++;
  }
}

bool CartTrajInterp::interpolate(trajectory_msgs::JointTrajectory & trajectory_in, trajectory_msgs::JointTrajectory & trajectory_out)
{
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


bool CartTrajInterp::interpolatePt(trajectory_msgs::JointTrajectoryPoint & p1,
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

bool CartTrajInterp::startHook(){
  return true;
}

void CartTrajInterp::stopHook() {
}