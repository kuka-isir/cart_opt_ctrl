#ifndef CART_OPT_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H
#define CART_OPT_CONTROLLER_HARDWARE_INTERFACE_ADAPTER_H

#include <joint_trajectory_controller/hardware_interface_adapter.h>
#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <qpOASES.hpp>
#include <boost/graph/graph_concepts.hpp>

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
class HardwareInterfaceAdapter<hardware_interface::CartOptEffortJointInterface, State>
{
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
              P_gain(i) = static_cast<double>(xml_array[i]);
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
              D_gain(i) = static_cast<double>(xml_array[i]);
          }
        }
      }
    }
    if(nh_controller_.hasParam("joint_gains/p_gains")){
      if(nh_controller_.getParam("joint_gains/p_gains", xml_array)){ 
        if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
          ROS_ERROR_STREAM("The joint_gains/p_gains parameter is not an array (namespace: " <<nh_controller_.getNamespace() << ").");
        else{
          if(xml_array.size()!=arm.getNrOfJoints()){
            ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, "<<arm.getNrOfJoints());
          }else{
            for(unsigned int i=0; i<arm.getNrOfJoints(); ++i)
              P_joint_gain(i) = static_cast<double>(xml_array[i]);
          }
        }
      }
    }
    if (nh_controller_.hasParam("joint_gains/d_gains")){
      if(nh_controller_.getParam("joint_gains/d_gains", xml_array)){ 
        if (xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray)
          ROS_ERROR_STREAM("The joint_gains/d_gains parameter is not an array (namespace: " <<nh_controller_.getNamespace() << ").");
        else{
          if(xml_array.size()!=arm.getNrOfJoints()){
            ROS_ERROR_STREAM("The array joint_gains/p_gains parameter does not have the right size, "<<arm.getNrOfJoints());
          }
          else{
            for(unsigned int i=0; i<arm.getNrOfJoints(); ++i)
              D_joint_gain(i) = static_cast<double>(xml_array[i]);
          }
        }
      }
    }
    if (nh_controller_.hasParam("regularisation_weights/joints"))
      nh_controller_.getParam("regularisation_weights/joints", Alpha);
    if (nh_controller_.hasParam("regularisation_weights/tau"))
      nh_controller_.getParam("regularisation_weights/tau", Regularisation);
  }
  
  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& controller_nh)
  {
    nh_controller_ = controller_nh;
    
    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    // Init chain utils
    if( !arm.init() )
    {
      ROS_ERROR("Could not init chain utils !");
      return false;
    }
    
    // The number of joints
    const int dof  = arm.getNrOfJoints();
    ee_frame = ee_frame = arm.getSegmentName( arm.getNrOfSegments() - 1 );
    
    // Resize vectors
    P_gain.resize(6);
    D_gain.resize(6);
    P_joint_gain.resize(dof);
    D_joint_gain.resize(dof);
    qdd_des.resize(dof);
    q_traj.resize(dof);
    qd_traj.resize(dof);
    qdd_traj.resize(dof);
    q_curr.resize(dof);
    qd_curr.resize(dof);
    
    // Resize and set torque at zero
    joint_torque_out.setZero(dof);
    joint_position_in.setZero(dof);
    joint_velocity_in.setZero(dof);
    
    // Default gains, works but stiff
    P_gain << 1000, 1000, 1000, 300, 300, 300;
    D_gain << 50, 50, 50, 10, 10, 10;
    P_joint_gain << 450.0, 450.0, 80.0, 450.0, 80.0, 20.0, 1.0;
    D_joint_gain << 20.0, 20.0, 1.5, 20.0, 1.5, 1.0, 0.05;
    
    // Default Alpha value
    Alpha = 1e-03;
    Regularisation = 1e-05;
    
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
              P_gain(i) = static_cast<double>(xml_array[i]);
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
              D_gain(i) = static_cast<double>(xml_array[i]);
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
              P_joint_gain(i) = static_cast<double>(xml_array[i]);
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
              D_joint_gain(i) = static_cast<double>(xml_array[i]);
          }
        }
      }
    }
    if (controller_nh.hasParam("regularisation_weights/joints"))
      controller_nh.getParam("regularisation_weights/joints", Alpha);
    if (controller_nh.hasParam("regularisation_weights/tau"))
      controller_nh.getParam("regularisation_weights/tau", Regularisation);
    
    // Initialize qpoases solver
    int number_of_variables = dof;
    int number_of_constraints = dof;
    qpoases_solver.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints,qpOASES::HST_POSDEF));
    
    // QPOases options
    qpOASES::Options options;
    options.setToMPC(); // setToReliable() // setToDefault()
    options.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
    qpoases_solver->setOptions( options );
    qpoases_solver->setPrintLevel(qpOASES::PL_NONE); // PL_HIGH for full output, PL_NONE for... none
    
     // Instantiate other solvers
    this->fk_solver_vel_.reset(new KDL::ChainFkSolverVel_recursive(arm.Chain()));
    this->jntToJacDotSolver_.reset(new KDL::ChainJntToJacDotSolver(arm.Chain()));
    
    return true;
  }

  void starting(const ros::Time& /*time*/)
  {    
    if (!joint_handles_ptr_) {
      ROS_ERROR("Could not get joint handles from interface0 !");
      return;
    }

    // Reset zero effort commands
    for (unsigned int i = 0; i < joint_handles_ptr_->size(); ++i)
    {
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
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
      joint_position_in(i) = (*joint_handles_ptr_)[i].getPosition();
      joint_velocity_in(i) = (*joint_handles_ptr_)[i].getVelocity();
    }
    
     // Feed the internal model
    arm.setState(joint_position_in,joint_velocity_in);
    // Make some calculations
    arm.updateModel();
    
    // Get current end effector Pose
    X_curr = arm.getSegmentPosition(ee_frame);
    Xd_curr = arm.getSegmentVelocity(ee_frame);
    q_curr = arm.getJointPositions();
    qd_curr = arm.getJointVelocities();
    
    // Initialize the desired velocity and acceleration to zero
    KDL::SetToZero(Xd_traj);
    KDL::SetToZero(Xdd_traj);
    KDL::SetToZero(qd_traj);
    KDL::SetToZero(qdd_traj);
    
    // Then overrride the desired
    for(unsigned int i=0; i<n_joints; ++i){
      q_traj(i) = desired_state.position[i];
      qd_traj(i) = desired_state.velocity[i];
      qdd_traj(i) = desired_state.acceleration[i];
    }
  
    // Compute FK
    KDL::FrameVel traj_pt_vel;
    KDL::JntArrayVel pt_in_vel;
    pt_in_vel.q = q_traj;
    pt_in_vel.qdot = qd_traj;
    KDL::Twist JdotQdot;

    // If FK or JdotQdot solver fails start over 
    if ((this->fk_solver_vel_->JntToCart(pt_in_vel, traj_pt_vel) == 0) &&  (this->jntToJacDotSolver_->JntToJacDot(pt_in_vel, JdotQdot) == 0) ){
      KDL::Jacobian J = this->arm.getJacobian();
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
    }
    else{
      // Stay at the same position
      X_traj = X_curr;
      q_traj = q_curr;
    }
  
    // Compute cartesian errors
    X_err = diff( X_curr , X_traj );
    Xd_err = diff( Xd_curr , Xd_traj);
  
    // Apply PD 
    for(unsigned int i=0; i<6; ++i )
    {
      Xdd_des(i) = Xdd_traj(i) +P_gain(i) * X_err(i) + D_gain(i) * Xd_err(i);
    }
    for(unsigned int i=0; i<n_joints; ++i){
      qdd_des(i) = qdd_traj(i) + P_joint_gain(i) * state_error.position[i] + D_joint_gain(i) * state_error.velocity[i];
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
    b2.noalias() = - a2 * ( coriolis.data + gravity.data ) - qdd_des;
    
    // Matrices for qpOASES
    // NOTE: We need RowMajor (see qpoases doc)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
    H.resize(n_joints,n_joints);
    
    Eigen::VectorXd g(n_joints);
    
    Eigen::MatrixXd regularisation;
    regularisation.resize(n_joints, n_joints);
    regularisation.setIdentity();
    regularisation *= Regularisation;
    
    H = 2.0 * a.transpose() * a + Alpha* 2.0 * a2.transpose() * a2 + regularisation;
    g = 2.0 * a.transpose() * b + Alpha* 2.0 * a2.transpose() * b2;
    
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
    
        lbA = (( qd_min - joint_velocity_in ) / horizon_dt + nonLinearTerms).cwiseMax(
                    2*(arm.getJointLowerLimit() - joint_position_in - joint_velocity_in * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
    
    ubA = (( qd_max - joint_velocity_in ) / horizon_dt + nonLinearTerms).cwiseMin(
                    2*(arm.getJointUpperLimit() - joint_position_in - joint_velocity_in * horizon_dt)/ (horizon_dt*horizon_dt) + nonLinearTerms );
    
    
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
    
    // Send commands
    for (unsigned int i = 0; i < n_joints; ++i)
    {    
      (*joint_handles_ptr_)[i].setCommand(joint_torque_out(i));
    }
  }

private:
  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;
  
  ros::NodeHandle nh_controller_;
  
  // CHain chain_utils
  rtt_ros_kdl_tools::ChainUtils arm;
  Eigen::VectorXd joint_torque_out,
                  joint_position_in,
                  joint_velocity_in;
  KDL::FrameAcc traj_pt_in;
  KDL::JntArrayAcc traj_joint_in;
  
  std::string ee_frame;
  bool has_first_command = false;

  KDL::Frame X_traj,X_curr;
  KDL::Twist X_err,Xd_err,Xdd_err;
  KDL::Twist Xd_curr,Xdd_curr,Xd_traj,Xdd_traj;
  KDL::JntArray q_curr, qd_curr, q_traj, qd_traj, qdd_traj;
  KDL::Twist Xdd_des;
  Eigen::VectorXd qdd_des;
  double Alpha, Regularisation;
  
  Eigen::VectorXd P_gain,D_gain,P_joint_gain,D_joint_gain;

  // Solvers
  boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_solver_vel_;
  boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jntToJacDotSolver_;
  std::unique_ptr<qpOASES::SQProblem> qpoases_solver;
};

#endif // header guard