#ifndef CART_OPT_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H
#define CART_OPT_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <qpOASES.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <geometry_msgs/Pose.h>

#define M_PI 3.14159265358979323846  /* pi */

// New interface for cart_opt controller
namespace hardware_interface
{
  class CartOptEffortJointInterface : public JointCommandInterface {};
}

/**
 * \brief Adapter for the CartOptJointEffort hardware interface. 
 * Controller solves a QP to minimise the cartesian error.
 * There is a regularisation on tau and an other on the joints.
 *
 * \code
 * head_controller:
 *   type: "effort_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *   cartesian_gains:
 *     p_gains: [1000.0, 1000.0, 1000.0, 300.0, 300.0, 300.0]
 *     d_gains: [50.0, 50.0, 50.0, 10.0, 10.0, 10.0]
 *   regularisation_weights:
 *     joints: 1.0e-03
 *     tau: 1.0e-05
 *   joint_gains:
 *     p_gains: [450.0, 450.0, 80.0, 450.0, 80.0, 20.0, 1.0]
 *     d_gains: [20.0, 20.0, 1.5, 20.0, 1.5, 1.0, 0.05]
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     #head_1_joint: {trajectory: 0.05, goal: 0.02} # not required because following cartesian path
 *     #head_2_joint: {trajectory: 0.05, goal: 0.02} # not required because following cartesian path
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class State>
class HardwareInterfaceAdapter<hardware_interface::CartOptEffortJointInterface, State>{
  public:
    HardwareInterfaceAdapter() : joint_handles_ptr_(0) {}
    
    void update_gains(){
      // Read params
      XmlRpc::XmlRpcValue xml_array;
      if(nh_controller_.hasParam("cartesian_gains/p_gains")){
	if(nh_controller_.getParam("cartesian_gains/p_gains", xml_array)){ 
	  if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR_STREAM("The cartesian_gains/p_gains parameter is not an array (namespace: " <<nh_controller_.getNamespace() << ").");
	  else{
	    if(xml_array.size()!=6){
	      ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, 6");
	    }else{
	      for(unsigned int i=0; i<6; ++i)
		p_gains_(i) = static_cast<double>(xml_array[i]);
	    }
	  }
	}
      }
      if (nh_controller_.hasParam("cartesian_gains/d_gains")){
	if(nh_controller_.getParam("cartesian_gains/d_gains", xml_array)){ 
	  if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR_STREAM("The cartesian_gains/d_gains parameter is not an array (namespace: " <<nh_controller_.getNamespace() << ").");
	  else{
	    if(xml_array.size()!=6){
	      ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, 6");
	    }else{
	      for(unsigned int i=0; i<6; ++i)
		d_gains_(i) = static_cast<double>(xml_array[i]);
	    }
	  }
	}
      }
      if(nh_controller_.hasParam("joint_gains/p_gains")){
	if(nh_controller_.getParam("joint_gains/p_gains", xml_array)){ 
	  if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR_STREAM("The joint_gains/p_gains parameter is not an array (namespace: " <<nh_controller_.getNamespace() << ").");
	  else{
	    if(xml_array.size()!=arm_.getNrOfJoints()){
	      ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, "<<arm_.getNrOfJoints());
	    }else{
	      for(unsigned int i=0; i<arm_.getNrOfJoints(); ++i)
		p_jnt_gains_(i) = static_cast<double>(xml_array[i]);
	    }
	  }
	}
      }
      if (nh_controller_.hasParam("joint_gains/d_gains")){
	if(nh_controller_.getParam("joint_gains/d_gains", xml_array)){ 
	  if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR_STREAM("The joint_gains/d_gains parameter is not an array (namespace: " <<nh_controller_.getNamespace() << ").");
	  else{
	    if(xml_array.size()!=arm_.getNrOfJoints()){
	      ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, "<<arm_.getNrOfJoints());
	    }
	    else{
	      for(unsigned int i=0; i<arm_.getNrOfJoints(); ++i)
		d_jnt_gains_(i) = static_cast<double>(xml_array[i]);
	    }
	  }
	}
      }
      if (nh_controller_.hasParam("regularisation_weights/joints"))
	nh_controller_.getParam("regularisation_weights/joints", jnt_task_weight_);
      if (nh_controller_.hasParam("regularisation_weights/tau"))
	nh_controller_.getParam("regularisation_weights/tau", regularisation_weight_);
    }
    
    bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh){
      nh_controller_ = controller_nh;
      
      // Store pointer to joint handles
      joint_handles_ptr_ = &joint_handles;

      // Init chain utils
      if( !arm_.init() ){
	ROS_ERROR("Could not init chain utils !");
	return false;
      }
      
      // The number of joints
      const int dof  = arm_.getNrOfJoints();
      ee_frame_ = ee_frame_ = arm_.getSegmentName( arm_.getNrOfSegments() - 1 );
      
      // Resize vectors
      p_gains_.resize(6);
      d_gains_.resize(6);
      p_jnt_gains_.resize(dof);
      d_jnt_gains_.resize(dof);
      qdd_des_.resize(dof);
      q_traj_.resize(dof);
      qd_traj_.resize(dof);
      qdd_traj_.resize(dof);
      q_curr_.resize(dof);
      qd_curr_.resize(dof);
      
      // Resize and set torque at zero
      joint_torque_out_.setZero(dof);
      joint_position_in_.setZero(dof);
      joint_velocity_in_.setZero(dof);
      
      // Default gains, works but stiff
      p_gains_ << 800, 800, 800, 350, 350, 350;
      d_gains_ << 22, 22, 22, 10, 10, 10;
      p_jnt_gains_ << 450.0, 450.0, 80.0, 450.0, 80.0, 20.0, 1.0;
      d_jnt_gains_ << 20.0, 20.0, 1.5, 20.0, 1.5, 1.0, 0.05;
      
      // Default jnt_task_weight_ value
      jnt_task_weight_ = 1e-03;
      regularisation_weight_ = 1e-05;
      
      // Read params
      XmlRpc::XmlRpcValue xml_array;
      if(controller_nh.hasParam("cartesian_gains/p_gains")){
	if(controller_nh.getParam("cartesian_gains/p_gains", xml_array)){ 
	  if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR_STREAM("The cartesian_gains/p_gains parameter is not an array (namespace: " <<controller_nh.getNamespace() << ").");
	  else{
	    if(xml_array.size()!=6){
	      ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, 6");
	    }else{
	      for(unsigned int i=0; i<6; ++i)
		p_gains_(i) = static_cast<double>(xml_array[i]);
	    }
	  }
	}
      }
      if (controller_nh.hasParam("cartesian_gains/d_gains")){
	if(controller_nh.getParam("cartesian_gains/d_gains", xml_array)){ 
	  if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR_STREAM("The cartesian_gains/d_gains parameter is not an array (namespace: " <<controller_nh.getNamespace() << ").");
	  else{
	    if(xml_array.size()!=6){
	      ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, 6");
	    }else{
	      for(unsigned int i=0; i<6; ++i)
		d_gains_(i) = static_cast<double>(xml_array[i]);
	    }
	  }
	}
      }
      if(controller_nh.hasParam("joint_gains/p_gains")){
	if(controller_nh.getParam("joint_gains/p_gains", xml_array)){ 
	  if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR_STREAM("The joint_gains/p_gains parameter is not an array (namespace: " <<controller_nh.getNamespace() << ").");
	  else{
	    if(xml_array.size()!=dof){
	      ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, "<<dof);
	    }else{
	      for(unsigned int i=0; i<dof; ++i)
		p_jnt_gains_(i) = static_cast<double>(xml_array[i]);
	    }
	  }
	}
      }
      if (controller_nh.hasParam("joint_gains/d_gains")){
	if(controller_nh.getParam("joint_gains/d_gains", xml_array)){ 
	  if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
	    ROS_ERROR_STREAM("The joint_gains/d_gains parameter is not an array (namespace: " <<controller_nh.getNamespace() << ").");
	  else{
	    if(xml_array.size()!=dof){
	      ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, "<<dof);
	    }
	    else{
	      for(unsigned int i=0; i<dof; ++i)
		d_jnt_gains_(i) = static_cast<double>(xml_array[i]);
	    }
	  }
	}
      }
      if (controller_nh.hasParam("regularisation_weights/joints"))
	controller_nh.getParam("regularisation_weights/joints", jnt_task_weight_);
      if (controller_nh.hasParam("regularisation_weights/tau"))
	controller_nh.getParam("regularisation_weights/tau", regularisation_weight_);
      
      // Initialize qpoases solver
      int number_of_variables = dof;
      int number_of_constraints = dof;
      qpoases_solver_.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints,qpOASES::HST_POSDEF));
      
      // QPOases options
      qpOASES::Options options;
      options.setToMPC(); // setToReliable() // setToDefault()
      options.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
      options.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
      qpoases_solver_->setOptions( options );
      qpoases_solver_->setPrintLevel(qpOASES::PL_NONE); // PL_HIGH for full output, PL_NONE for... none
      
      // Instantiate other solvers
      fk_solver_vel_.reset(new KDL::ChainFkSolverVel_recursive(arm_.Chain()));
      jntToJacDotSolver_.reset(new KDL::ChainJntToJacDotSolver(arm_.Chain()));
      
      xdes_pub_ = controller_nh.advertise<geometry_msgs::Pose>("/cart_opt/desired", 1);
      xcurr_pub_ = controller_nh.advertise<geometry_msgs::Pose>("/cart_opt/current", 1);
      xerr_pub_ = controller_nh.advertise<geometry_msgs::Twist>("/cart_opt/error", 1);
      
      return true;
    }

    void starting(const ros::Time& /*time*/){    
      if (!joint_handles_ptr_) {
	ROS_ERROR("Could not get joint handles from interface0 !");
	return;
      }

      // Reset zero effort commands
      for (unsigned int i = 0; i < joint_handles_ptr_->size(); ++i)
	(*joint_handles_ptr_)[i].setCommand(0.0);
    }

    void stopping(const ros::Time& /*time*/) {}

    void updateCommand(const ros::Time&     time,
		      const ros::Duration& period,
		      const State&         desired_state,
		      const State&         state_error)
    {
      const unsigned int n_joints = joint_handles_ptr_->size();

      // Update current gains with params
      update_gains();
      
      // Preconditions
      if (!joint_handles_ptr_) {return;}
      assert(n_joints == state_error.position.size());
      assert(n_joints == state_error.velocity.size());

      // Get current state from the joint handles
      for(unsigned int i=0; i<n_joints; ++i){
	joint_position_in_(i) = (*joint_handles_ptr_)[i].getPosition();
	joint_velocity_in_(i) = (*joint_handles_ptr_)[i].getVelocity();
      }
      
      // Feed the internal model
      arm_.setState(joint_position_in_,joint_velocity_in_);
      // Make some calculations
      arm_.updateModel();
      
      // Get current end effector Pose
      X_curr_ = arm_.getSegmentPosition(ee_frame_);
      Xd_curr_ = arm_.getSegmentVelocity(ee_frame_);
      q_curr_ = arm_.getJointPositions();
      qd_curr_ = arm_.getJointVelocities();
      
      // Initialize the desired velocity and acceleration to zero
      KDL::SetToZero(Xd_traj_);
      KDL::SetToZero(Xdd_traj_);
      KDL::SetToZero(qd_traj_);
      KDL::SetToZero(qdd_traj_);
      
      // Then overrride the desired
      for(unsigned int i=0; i<n_joints; ++i){
	q_traj_(i) = desired_state.position[i];
	qd_traj_(i) = desired_state.velocity[i];
	qdd_traj_(i) = desired_state.acceleration[i];
      }
    
      // Compute FK
      KDL::FrameVel traj_pt_vel;
      KDL::JntArrayVel pt_in_vel;
      pt_in_vel.q = q_traj_;
      pt_in_vel.qdot = qd_traj_;
      KDL::Twist JdotQdot;

      // If FK or JdotQdot solver fails start over 
      if ((fk_solver_vel_->JntToCart(pt_in_vel, traj_pt_vel) == 0) &&  (jntToJacDotSolver_->JntToJacDot(pt_in_vel, JdotQdot) == 0) ){
	KDL::Jacobian J = arm_.getJacobian();
	KDL::Twist xdotdot;
	
	// Xdd = Jd * qd + J * qdd
	for(unsigned int i = 0; i < 6; ++i )
	  xdotdot(i) = JdotQdot(i) + J.data(i) * qdd_traj_(i);
      
	// Cartesian trajectory point out
	KDL::FrameAcc traj_pt_out_;
	traj_pt_out_ = KDL::FrameAcc(traj_pt_vel.GetFrame(), traj_pt_vel.GetTwist(), xdotdot);
	X_traj_ = traj_pt_out_.GetFrame();
	Xd_traj_ = traj_pt_out_.GetTwist();
	Xdd_traj_ = traj_pt_out_.GetAccTwist();
      }
      else{
	// Stay at the same position
	X_traj_ = X_curr_;
	q_traj_ = q_curr_;
      }
    
      // Compute cartesian errors
      X_err_ = diff( X_curr_ , X_traj_ );
      Xd_err_ = diff( Xd_curr_ , Xd_traj_);
      
      // Saturate the position and integral errors
      for(unsigned int i=0; i<3; ++i ){
	if(X_err_(i) >0)
	  X_err_(i) = std::min(0.01, X_err_(i));
	else
	  X_err_(i) = std::max(-0.01, X_err_(i));
      }
      for(unsigned int i=3; i<6; ++i ){
	if(X_err_(i) >0)
	  X_err_(i) = std::min(M_PI/100, X_err_(i));
	else
	  X_err_(i) = std::max(-M_PI/100, X_err_(i));
      }    
  //  X_err_(i) = std::min(0.01, std::abs(X_err_(i))) * X_err_(i) / std::abs(X_err_(i)) ;
      
      // Saturate joint position error
      State error_saturate = state_error;
      for(unsigned int i=0; i<n_joints; ++i){
	if(error_saturate.position[i] >0)
	  error_saturate.position[i] = std::min(0.01, error_saturate.position[i]);
	else
	  error_saturate.position[i] = std::max(-0.01, error_saturate.position[i]);
      }
      
      // Apply PD 
      for(unsigned int i=0; i<6; ++i )
	Xdd_des_(i) = Xdd_traj_(i) +p_gains_(i) * X_err_(i) + d_gains_(i) * Xd_err_(i);
      for(unsigned int i=0; i<n_joints; ++i)
	qdd_des_(i) = qdd_traj_(i) + p_jnt_gains_(i) * error_saturate.position[i] + d_jnt_gains_(i) * error_saturate.velocity[i];
      
      Eigen::Matrix<double,6,1> xdd_des;
      tf::twistKDLToEigen(Xdd_des_,xdd_des);
	  
      KDL::Jacobian& J = arm_.getSegmentJacobian(ee_frame_);
      KDL::JntSpaceInertiaMatrix& M_inv = arm_.getInertiaInverseMatrix();
      KDL::JntArray& coriolis = arm_.getCoriolisTorque();
      KDL::JntArray& gravity = arm_.getGravityTorque();
      KDL::Twist& Jdotqdot = arm_.getSegmentJdotQdot(ee_frame_);
      Eigen::Matrix<double,6,1> jdot_qdot;
      tf::twistKDLToEigen(Jdotqdot,jdot_qdot);
      
      Eigen::Matrix<double,6,Eigen::Dynamic> a;
      a.resize(6,n_joints);
      Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> a2;
      a2.resize(n_joints,n_joints);
      
      Eigen::Matrix<double,6,1> b;
      Eigen::Matrix<double,Eigen::Dynamic,1> b2;
      b2.resize(n_joints,1);
      
      a.noalias() = J.data * M_inv.data;
      a2.noalias() = M_inv.data;
      b.noalias() = - a * ( coriolis.data + gravity.data ) + jdot_qdot - xdd_des;
      b2.noalias() = - a2 * ( coriolis.data + gravity.data ) - qdd_des_;
      
      // Matrices for qpOASES
      // NOTE: We need RowMajor (see qpoases doc)
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
      H.resize(n_joints,n_joints);
      
      Eigen::VectorXd g(n_joints);
      
      Eigen::MatrixXd regularisation;
      regularisation.resize(n_joints, n_joints);
      regularisation.setIdentity();
      regularisation *= regularisation_weight_;
      
      H = 2.0 * a.transpose() * a + jnt_task_weight_* 2.0 * a2.transpose() * a2 + regularisation;
      g = 2.0 * a.transpose() * b + jnt_task_weight_* 2.0 * a2.transpose() * b2;
      
      // TODO: get this from URDF
      Eigen::VectorXd torque_max;
      torque_max.resize(n_joints);
      torque_max << 200,200,100,100,100,30,30; // N.m

      Eigen::VectorXd torque_min;
      torque_min.resize(n_joints);
      torque_min = -torque_max; // N.m
      
      // Compute bounds
      Eigen::VectorXd lb(n_joints),
		      ub(n_joints);
      
      //TODO : write constraints for q and qdot
      lb = torque_min;
      ub = torque_max;
      
      // Update Constraints
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(n_joints,n_joints);
      Eigen::VectorXd lbA(n_joints),
		      ubA(n_joints),
		      qd_min(n_joints),
		      qd_max(n_joints);
      
      qd_min.setConstant(-1.0);
      qd_max.setConstant(1.0);
      
      A = M_inv.data;
      
      double horizon_dt = 0.015;
      
      Eigen::VectorXd nonLinearTerms(n_joints);
      nonLinearTerms = M_inv.data * ( coriolis.data + gravity.data );    
      
      lbA = (( qd_min - joint_velocity_in_ ) / horizon_dt + nonLinearTerms).cwiseMax(
		  2*(arm_.getJointLowerLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
      
      ubA = (( qd_max - joint_velocity_in_ ) / horizon_dt + nonLinearTerms).cwiseMin(
		  2*(arm_.getJointUpperLimit() - joint_position_in_ - joint_velocity_in_ * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
      
      
      // number of allowed compute steps
      int nWSR = 1e6; 
      
      // Let's compute !
      qpOASES::returnValue ret;
      static bool qpoases_initialized = false;
      
      if(!qpoases_initialized)
      {
	// Initialise the problem, once it has found a solution, we can hotstart
	ret = qpoases_solver_->init(H.data(),g.data(),A.data(),lb.data(),ub.data(),lbA.data(),ubA.data(),nWSR);
	
	// Keep init if it didn't work
	if(ret == qpOASES::SUCCESSFUL_RETURN)
	{
	  qpoases_initialized = true;
	}
      }
      else{
	// Otherwise let's reuse the previous solution to find a solution faster
	ret = qpoases_solver_->hotstart(H.data(),g.data(),A.data(),lb.data(),ub.data(),lbA.data(),ubA.data(),nWSR);
	
	if(ret != qpOASES::SUCCESSFUL_RETURN){
	  qpoases_initialized = false;
	}
      }
      
      // Zero grav if not found
      // TODO: find a better alternative
      joint_torque_out_.setZero();
      
      if(ret == qpOASES::SUCCESSFUL_RETURN){
	// Get the solution
	qpoases_solver_->getPrimalSolution(joint_torque_out_.data());
	// Remove gravity because Kuka already adds it
	joint_torque_out_ -= gravity.data;
      }
      
      // Send commands
      for (unsigned int i = 0; i < n_joints; ++i){    
	(*joint_handles_ptr_)[i].setCommand(joint_torque_out_(i));
      }
      
      // Debug publish in ROS
      geometry_msgs::Pose xdes, xcurr;
      geometry_msgs::Twist xerr;    
      tf::poseKDLToMsg(X_traj_,xdes);
      tf::poseKDLToMsg(X_curr_,xcurr);
      tf::twistKDLToMsg(X_err_, xerr);
      xdes_pub_.publish(xdes);
      xcurr_pub_.publish(xcurr);
      xerr_pub_.publish(xerr);
    }

  private:
    std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
    ros::NodeHandle nh_controller_;
    
    // CHain chain_utils
    rtt_ros_kdl_tools::ChainUtils arm_;
    Eigen::VectorXd joint_torque_out_,
		    joint_position_in_,
		    joint_velocity_in_;
    
    std::string ee_frame_;
    bool has_first_command_ = false;

    KDL::Frame X_traj_,X_curr_;
    KDL::Twist X_err_,Xd_err_,Xdd_err_;
    KDL::Twist Xd_curr_,Xdd_curr_,Xd_traj_,Xdd_traj_;
    KDL::JntArray q_curr_, qd_curr_, q_traj_, qd_traj_, qdd_traj_;
    KDL::Twist Xdd_des_;
    Eigen::VectorXd qdd_des_;
    double jnt_task_weight_, regularisation_weight_;
    
    Eigen::VectorXd p_gains_,d_gains_,p_jnt_gains_,d_jnt_gains_;

    // Solvers
    boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_solver_vel_;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jntToJacDotSolver_;
    std::unique_ptr<qpOASES::SQProblem> qpoases_solver_;
    
    // Publishers for debug
    ros::Publisher xdes_pub_, xcurr_pub_, xerr_pub_;
};

#endif // header guard