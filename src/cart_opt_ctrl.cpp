#include "cart_opt_ctrl/cart_opt_ctrl.hpp"

namespace lwr{
using namespace RTT;
using namespace KDL;
using namespace gurobi;

CartOptCtrl::CartOptCtrl(const std::string& name):
t_traj_curr(0),
kp_lin(1000.0),
kp_ang(650.0),
kd_lin(50.0),
kd_ang(20.0),
ready_to_start_(false),
traj_computed(false),
debug_mode_(false),
use_jdot_qdot_(true),
use_f_ext_(true),
use_coriolis_(true),
d_ang_max_(100.0),
dw_max_(0.5),
use_xdd_des_(true),
use_mass_sqrt_(false),
elapsed(0),
use_xd_des_(true),
use_ft_sensor_(false),
model_verbose_(false),
jacobian_solver_type_(WDL_SOLVER),
RTTLWRAbstract(name)
{
//     this->ports()->addPort("PathROS",port_path_ros).doc("");
    this->ports()->addPort("X_curr",port_X_curr).doc("");
    this->ports()->addPort("X_tmp",port_X_tmp).doc("");
    this->ports()->addPort("X_des",port_X_des).doc("");
    this->ports()->addPort("FTData",port_ftdata).doc("The ATI F/T Sensor Input");
    this->addOperation("publishTrajectory",&CartOptCtrl::publishTrajectory,this,RTT::OwnThread);
    this->addOperation("computeTrajectory",&CartOptCtrl::computeTrajectory,this,RTT::OwnThread);
    this->addAttribute("kp_lin",kp_lin);
    this->addAttribute("kp_ang",kp_ang);
    this->addAttribute("kd_lin",kd_lin);
    this->addAttribute("kd_ang",kd_ang);
    this->addAttribute("dw_max",dw_max_);
    this->addAttribute("dx_ang",d_ang_max_);
    this->addAttribute("debug_mode",debug_mode_);
    this->addAttribute("use_jdot_qdot",use_jdot_qdot_);
    this->addAttribute("use_f_ext",use_f_ext_);
    this->addAttribute("use_coriolis",use_coriolis_);
    this->addAttribute("use_xd_des",use_xd_des_);
    this->addAttribute("use_xdd_des",use_xdd_des_);
    this->addAttribute("ReadyToStart",ready_to_start_);
    this->addOperation("setSolverMethod",&CartOptCtrl::setSolverMethod,this,RTT::OwnThread);
    this->addOperation("setSolverVerbose",&CartOptCtrl::setSolverVerbose,this,RTT::OwnThread);
    this->addOperation("setSolverTimeLimit",&CartOptCtrl::setSolverTimeLimit,this,RTT::OwnThread);
    this->addOperation("setSolverBarrierConvergeanceTolerance",&CartOptCtrl::setSolverBarrierConvergeanceTolerance,this,RTT::OwnThread);
    this->addAttribute("jacobian_solver_type",jacobian_solver_type_);
    this->provides("debug")->addAttribute("solver_duration",solver_duration);
    this->provides("debug")->addAttribute("UpdateHookDuration",elapsed);
    this->provides("debug")->addAttribute("WrenchInBase",F_ext);
    this->addAttribute("use_mass_sqrt",use_mass_sqrt_);
    this->addAttribute("use_ft_sensor",use_ft_sensor_);
}
void CartOptCtrl::setSolverMethod(int i)
{
    cart_model_solver_.setMethod(i);
}

void CartOptCtrl::setSolverBarrierConvergeanceTolerance(double t)
{
    cart_model_solver_.setBarrierConvergeanceTolerance(t);
}
void CartOptCtrl::setSolverTimeLimit(double t)
{
    this->cart_model_solver_.setTimeLimit(t);
}

void CartOptCtrl::setSolverVerbose(bool v)
{
    this->cart_model_solver_.setVerbose(v);
}
bool CartOptCtrl::configureHook()
{
    log(Warning) << "Configuring parent" << endlog();
    
    if(false == RTTLWRAbstract::configureHook())
    {
        log(RTT::Fatal) << "Configure parent error" << endlog();
        return false;
    }
    this->getAllComponentRelative();

    log(Warning) << "Configuring ChainJntToJacDotSolver " << endlog();
    jdot_solver.reset(new KDL::ChainJntToJacDotSolver(kdl_chain));
    log(Warning) << "Configuring ChainIkSolverVel_pinv " << endlog();
    pinv_solver.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain));
    log(Warning) << "Configuring ChainIkSolverVel_wdls " << endlog();
    wdls_solver.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain));
    
    log(Warning) << "Getting state" << endlog();
    int cnt = 10;
    while(!updateState() && !cnt--)
    {
        log(Warning) << "Waiting for lwr_fri to publish data " << endlog();
        usleep(5e5);
    }
    
    log(Warning) << "setJointTorqueControlMode" << endlog();
    setJointTorqueControlMode();
    
    log(Warning) << "createStream" << endlog();
    Xd_cmd.setZero();
    X_err.setZero();
    Xd_err.setZero();
    
    port_X_curr.createStream(rtt_roscomm::topic("~"+getName()+"/pos_curr"));
    port_X_tmp.createStream(rtt_roscomm::topic("~"+getName()+"/pos_tmp"));
    port_X_des.createStream(rtt_roscomm::topic("~"+getName()+"/pos_des"));
    port_pose_array.createStream(rtt_roscomm::topic("~"+getName()+"/traj_poses"));
    port_path_ros.createStream(rtt_roscomm::topic("~"+getName()+"/cart_traj_des"));
    port_solver_duration.createStream(rtt_roscomm::topic("~"+getName()+"/solver_duration"));
    port_qdd_max.createStream(rtt_roscomm::topic("~"+getName()+"/qdd_max"));
    port_qdd_min.createStream(rtt_roscomm::topic("~"+getName()+"/qdd_min"));
    port_qdd_des.createStream(rtt_roscomm::topic("~"+getName()+"/qdd_des"));
    
    qdd_min_ros.data.resize(kdl_chain.getNrOfJoints());
    qdd_max_ros.data.resize(kdl_chain.getNrOfJoints());
    qdd_des_ros.data.resize(kdl_chain.getNrOfJoints());
    
    
    qdd_des.resize(kdl_chain.getNrOfJoints());
    mass_kdl.resize(kdl_chain.getNrOfJoints());
    qdd_des_kdl.resize(kdl_chain.getNrOfJoints());
    coriolis.resize(kdl_chain.getNrOfJoints());
    jnt_pos_eigen.resize(kdl_chain.getNrOfJoints());
    jdot.resize(kdl_chain.getNrOfSegments());
    J_ati_base.resize(kdl_chain.getNrOfJoints());
    J_ee_base.resize(kdl_chain.getNrOfJoints());
    // Get initial pose
    getCartesianPosition(cart_pos);
    
    computeTrajectory(0.01,0.05,0.05,0.2);   
     
    mass_inv.resize(kdl_chain.getNrOfJoints(),kdl_chain.getNrOfJoints());
    
    
    log(RTT::Warning) << " Let's go" << endlog();
    
    return cnt;
}
bool CartOptCtrl::computeTrajectory(const double radius, const double eqradius,const double vmax, const double accmax)
{
    log(Warning) << "Computing Trajectory" << endlog();
    try
    {
        updateState();
        fk_vel_solver->JntToCart(jnt_pos_vel_kdl,frame_vel_des_kdl,this->seg_names_idx["ati_link"]);
        frame_des_kdl =  frame_vel_des_kdl.GetFrame();
       
        path = new Path_RoundedComposite(radius,eqradius,new RotationalInterpolation_SingleAxis());
        path->Add(frame_des_kdl);
        path->Add(Frame(Rotation::RPY(99.*deg2rad,        -17.*deg2rad,     -101.*deg2rad),   Vector(-.467,-.448,.576)));
        path->Add(Frame(Rotation::RPY(99.*deg2rad,        -17.*deg2rad,     -101.*deg2rad),   Vector(-.467,-.448,.376)));
        /*path->Add(Frame(Rotation::RPY(88.*deg2rad,        -4.*deg2rad,     -36.*deg2rad),   Vector(-.372,-.527,.505)));
        path->Add(Frame(Rotation::RPY(91.0*deg2rad,         -3.*deg2rad,   -28.*deg2rad), Vector(-.198,-.657,.695)));
        path->Add(Frame(Rotation::RPY(89.0*deg2rad,       -17.*deg2rad,   -32.*deg2rad), Vector(-.219,-.725,.404)));*/
        path->Add(frame_des_kdl);
        // always call Finish() at the end, otherwise the last segment will not be added.
        path->Finish();
        
        velpref = new VelocityProfile_Trap(0.03,0.1);
        velpref->SetProfile(0,path->PathLength());  
        traject = new Trajectory_Segment(path, velpref);
        
        ctraject = new Trajectory_Composite();
        ctraject->Add(traject);
        ctraject->Add(new Trajectory_Stationary(1.0,frame_des_kdl));
        
    } catch(KDL::Error& error) {
            std::cout <<"I encountered this error : " << error.Description() << endlog();
            std::cout << "with the following type " << error.GetType() << endlog();
            return false;
    }
    
    log(Info) << "Trajectory computed ! " << endlog();
    publishTrajectory();
    traj_computed = true;
    return true;
}

void CartOptCtrl::publishTrajectory()
{
    double dt=static_cast<double>(this->getPeriod());
    nav_msgs::Path path_ros;
    path_ros.header.frame_id = root_link;
    log(Debug) << "Creating Path" << endlog();
    geometry_msgs::PoseArray pose_array;
    for (double t=0.0; t <= traject->Duration(); t+= 0.1) {
            Frame current_pose;
            Twist current_vel,current_acc;
            
            current_pose = traject->Pos(t);
            current_vel = traject->Vel(t);
            current_acc = traject->Acc(t);
                        
            geometry_msgs::Pose pose;
            geometry_msgs::PoseStamped pose_st;
            tf::poseKDLToMsg(current_pose,pose);
            pose_array.poses.push_back(pose);
            pose_st.header.frame_id = path_ros.header.frame_id;
            pose_st.pose = pose;
            path_ros.poses.push_back(pose_st);

    }
    
    pose_array.header.frame_id = path_ros.header.frame_id;
    pose_array.header.stamp = rtt_rosclock::host_now();
    
    port_pose_array.write(pose_array);
    log(Debug) << "Sending Path" << endlog();
    port_path_ros.write(path_ros);
    log(Debug) << "Publishing done" << endlog();
}

void CartOptCtrl::updateHook()
{
    ros::Time t_start = rtt_rosclock::host_now();
    //publishTrajectory();
    
    log(Debug) << "Start" << endlog();
    if(!updateState() || !getMassMatrix(mass) || !traj_computed)
        return;
    log(Debug) << "go" << endlog();
    Frame X_des,X_mes;
    Twist Xdd_des,Xd_mes,Xd_des;
    
    X_des = traject->Pos(t_traj_curr);
    
    // Fk -> X last frame
    jnt_to_jac_solver->JntToJac(jnt_pos_kdl,J_ati_base,this->seg_names_idx["ati_link"]);
    jdot_solver->JntToJacDot(jnt_pos_vel_kdl,jdot_qdot,this->seg_names_idx["ati_link"]);
    fk_vel_solver->JntToCart(jnt_pos_vel_kdl,tool_in_base_framevel,this->seg_names_idx["ati_link"]);
    X_mes =  tool_in_base_framevel.GetFrame();
    Xd_mes = tool_in_base_framevel.GetTwist();
    
       
    Frame X_curr = X_mes;
    
    Twist d_err = diff(X_curr,X_des);    
    
    Frame X_tmp(X_curr);
    X_tmp.M = Rotation::Rot(d_err.rot,d_err.rot.Norm()) * X_tmp.M;
    
    if(1)
    {
        // Ros pub
       X_curr_msg.header.frame_id = 
            X_tmp_msg.header.frame_id = 
            X_des_msg.header.frame_id = root_link;
            
       X_curr_msg.header.stamp = 
            X_tmp_msg.header.stamp = 
            X_des_msg.header.stamp = rtt_rosclock::host_now();
            
       tf::poseKDLToMsg(X_curr,X_curr_msg.pose);
       tf::poseKDLToMsg(X_tmp,X_tmp_msg.pose);
       tf::poseKDLToMsg(X_des,X_des_msg.pose);
       
       port_X_curr.write(X_curr_msg);
       port_X_tmp.write(X_tmp_msg);
       port_X_des.write(X_des_msg);
    }
            
    if(use_xd_des_)
        Xd_des = traject->Vel(t_traj_curr);
    else
        SetToZero(Xd_des);
    Twist dd_err = diff(Xd_mes,Xd_des);
    
    
    
    Xdd_des.vel = kp_lin*d_err.vel + kd_lin*dd_err.vel;
    
    Vector dr = kp_ang*d_err.rot;
    dr(0) = clip(dr(0),-d_ang_max_,d_ang_max_);
    dr(1) = clip(dr(1),-d_ang_max_,d_ang_max_);
    dr(2) = clip(dr(2),-d_ang_max_,d_ang_max_);
    
    Xdd_des.rot = dr + kd_ang*(dd_err.rot);
    
    if(use_xdd_des_)
        Xdd_des += traject->Acc(t_traj_curr);
    

    
    
    
    id_dyn_solver->JntToCoriolis(jnt_pos_kdl,jnt_vel_kdl,coriolis_kdl);
    id_dyn_solver->JntToGravity(jnt_pos_kdl,gravity_kdl);
    id_dyn_solver->JntToMass(jnt_pos_kdl,mass_kdl);
    
    Eigen::Matrix<double,6,1> xdd_des_;
    tf::twistKDLToEigen(Xdd_des,xdd_des_);
    
    Eigen::Matrix<double,6,1> jdot_qdot_;
    tf::twistKDLToEigen(jdot_qdot,jdot_qdot_);
    
    try{
        
        ros::Time start_opt = rtt_rosclock::host_now();
#ifndef __XENOMAI__
        struct timeval tbegin,tend;
        gettimeofday(&tbegin,NULL);
#endif
        cart_model_solver_.optimize(jnt_pos,
                                    jnt_vel,
                                    
                                    3.*static_cast<double>(getPeriod()),
                                    J_ati_base.data,
                                    mass_kdl.data,
                                    jdot_qdot_,
                                    coriolis_kdl.data,
                                    gravity_kdl.data,
                                    xdd_des_,
                                    jnt_trq_cmd);

        cart_model_solver_.getQddBounds(qdd_min_ros.data,qdd_max_ros.data,qdd_des_ros.data);
#ifndef __XENOMAI__
        gettimeofday(&tend,NULL);
#endif
        // Don't forget to remove G(q) as Kuka adds it on KRC 
        jnt_trq_cmd-=gravity_kdl.data;
        
        this->solver_duration = (rtt_rosclock::host_now() - start_opt).toSec();
        
        std_msgs::Float32 d;
#ifndef __XENOMAI__
        d.data =  1000*(tend.tv_sec-tbegin.tv_sec)+(tend.tv_usec-tbegin.tv_usec)/1000.;
#else
        d.data = solver_duration;
#endif
        port_solver_duration.write(d);
        port_qdd_des.write(qdd_des_ros);
        port_qdd_min.write(qdd_min_ros);
        port_qdd_max.write(qdd_max_ros);
        
    } catch(GRBException e) {
        log(RTT::Error) << "Error code = " << e.getErrorCode() << endlog();
        log(RTT::Error) << e.getMessage() << endlog();
        log(RTT::Error) << "jnt_pos_kdl"<<jnt_pos_kdl
                        << "jnt_vel_kdl"<<jnt_vel_kdl
                        << "\nJ_ati_base.data\n"<<J_ati_base.data
                        << "\nmass_kdl.data\n" <<mass_kdl.data
                        << "\njdot_qdot_\n" <<jdot_qdot_.transpose()
                        << "\ncoriolis_kdl.data\n" <<coriolis_kdl.data.transpose()
                        << "\ngravity_kdl.data\n" << gravity_kdl.data.transpose()
                        << "\nxdd_des_\n"<< xdd_des_.transpose()
                        << "\njnt_trq_cmd\n"<<jnt_trq_cmd.transpose()
                        <<endlog();
    } catch(...) {
        log(RTT::Error) << "Exception during optimization" << endlog();
    }    
    log(Info) << "trqcmd " << jnt_trq_cmd.transpose() << endlog();
    
    if(!debug_mode_)
        if(!isCommandMode())
            return;
        
    sendJointTorque(jnt_trq_cmd);
    
    // Incremente traj
    if(isReadyToStart()){
        if( t_traj_curr <= traject->Duration())
            t_traj_curr += static_cast<double>(this->getPeriod());
        else
            t_traj_curr = 0.0;
    }
    ros::Duration t_elapsed = rtt_rosclock::host_now() - t_start;
    elapsed = t_elapsed.toSec();
}

}
