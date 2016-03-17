#include "cart_opt_ctrl/cart_opt_ctrl.hpp"


namespace lwr{
using namespace RTT;
using namespace KDL;

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
elapsed(0),
use_xd_des_(true),
use_ft_sensor_(true),
model_verbose_(false),
jacobian_solver_type_(WDL_SOLVER),
kdt_(5.0),
init_pos_acquired(false),
use_sim_clock(false),
trajectory_frame("link_7"),
spacenav_scale_trans(0.0001),
spacenav_scale_rot(0.0001),
n_updates_(0),
RTTLWRAbstract(name)
{
//     this->ports()->addPort("PathROS",port_path_ros).doc("");
    /*this->ports()->addPort("X_curr",port_X_curr).doc("");
    this->ports()->addPort("X_tmp",port_X_tmp).doc("");
    this->ports()->addPort("X_des",port_X_des).doc("");*/
    this->ports()->addPort("ati_ft_data_in",port_ftdata).doc("The ATI F/T Sensor Input");
    this->addOperation("publishTrajectory",&CartOptCtrl::publishTrajectory,this,RTT::OwnThread);
    this->addOperation("computeTrajectory",&CartOptCtrl::computeTrajectory,this,RTT::OwnThread);
    this->addProperty("kp_lin",kp_lin);
    this->addProperty("kp_ang",kp_ang);
    this->addProperty("kd_lin",kd_lin);
    this->addProperty("kd_ang",kd_ang);
    this->addProperty("dw_max",dw_max_);
    this->addProperty("dx_ang",d_ang_max_);
    this->addProperty("spacenav_scale_trans",spacenav_scale_trans);
    this->addProperty("spacenav_scale_rot",spacenav_scale_rot);
    this->addProperty("trajectory_frame",trajectory_frame);
    this->addProperty("kdt",kdt_);
    this->addProperty("debug_mode",debug_mode_);
    this->addProperty("use_jdot_qdot",use_jdot_qdot_);
    this->addProperty("use_f_ext",use_f_ext_);
    this->addProperty("use_coriolis",use_coriolis_);
    this->addProperty("use_xd_des",use_xd_des_);
    this->addProperty("use_xdd_des",use_xdd_des_);
    this->addProperty("ReadyToStart",ready_to_start_);
    this->addProperty("jacobian_solver_type",jacobian_solver_type_);
    this->provides("debug")->addAttribute("solver_duration",solver_duration);
    this->provides("debug")->addAttribute("UpdateHookDuration",elapsed);
    this->provides("debug")->addAttribute("WrenchInBase",F_ext);
    this->addProperty("use_ft_sensor",use_ft_sensor_);
    this->addProperty("use_sim_clock",use_sim_clock);
// Async Optimize
            this->addPort("q",port_q).doc("");
            this->addPort("qdot",port_qdot).doc("");
            this->addPort("jac",port_jacobian).doc("");
            this->addPort("mass",port_mass).doc("");
            this->addPort("dt",port_dt).doc("");
            this->addPort("jdot_qdot",port_jdot_qdot).doc("");
            this->addPort("coriolis",port_coriolis).doc("");
            this->addPort("gravity",port_gravity).doc("");
            this->addPort("xdd_des",port_xdd_des).doc("");
            this->addPort( "optimize", port_optimize_event ).doc( "" );
            this->addPort( "add_torque", port_add_torque ).doc( "" );
}

bool CartOptCtrl::configureHook()
{
    log(Warning) << "Configuring parent" << endlog();
    
    if(false == RTTLWRAbstract::init())
    {
        log(RTT::Fatal) << "Configure parent error" << endlog();
        return false;
    }
    /////port_JointTorqueCommand.disconnect();
    log(Warning) << "Configured" << endlog();
    this->getAllComponentRelative();

    log(Warning) << "Configuring ChainJntToJacDotSolver " << endlog();
    jdot_solver.reset(new KDL::ChainJntToJacDotSolver(kdl_chain));
    
    log(Warning) << "Getting state" << endlog();
    int cnt = 10;
    while(!updateState() && cnt--)
    {
        log(Warning) << "Waiting for lwr_fri to publish data " << endlog();
        usleep(1e6);
    }
    
    log(Warning) << "setJointTorqueControlMode" << endlog();
    setJointTorqueControlMode();
    
    log(Warning) << "createStream" << endlog();
    Xd_cmd.setZero();
    X_err.setZero();
    Xd_err.setZero();
    
    port_spacenav.createStream(rtt_roscomm::topic("spacenav/twist"));
    
    port_X_curr.createStream(rtt_roscomm::topic("~"+getName()+"/pos_curr"));
    port_X_tmp.createStream(rtt_roscomm::topic("~"+getName()+"/pos_tmp"));
    port_X_des.createStream(rtt_roscomm::topic("~"+getName()+"/pos_des"));
    port_pose_array.createStream(rtt_roscomm::topic("~"+getName()+"/traj_poses"));
    port_path_ros.createStream(rtt_roscomm::topic("~"+getName()+"/cart_traj_des"));
    port_solver_duration.createStream(rtt_roscomm::topic("~"+getName()+"/solver_duration"));
    port_qdd_max.createStream(rtt_roscomm::topic("~"+getName()+"/qdd_max"));
    port_qdd_min.createStream(rtt_roscomm::topic("~"+getName()+"/qdd_min"));
    port_qdd_des.createStream(rtt_roscomm::topic("~"+getName()+"/qdd_des"));
    
    port_loop_duration.createStream(rtt_roscomm::topic("~"+getName()+"/loop_duration"));

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
    jnt_acc_kdl.resize(kdl_chain.getNrOfJoints());
    // Get initial pose   
     
    mass_inv.resize(kdl_chain.getNrOfJoints(),kdl_chain.getNrOfJoints());
    
    port_qdot.setDataSample(jnt_pos);
    port_jacobian.setDataSample(J_ati_base.data);
    port_mass.setDataSample(mass_kdl.data);
    port_jdot_qdot.setDataSample(jdot_qdot_);
    port_coriolis.setDataSample(coriolis);
    port_gravity.setDataSample(jnt_grav);
    port_xdd_des.setDataSample(xdd_des_);
    
    traj_computed = computeTrajectory(0.01,0.05,0.05,0.2);
    
    if(use_sim_clock){
        RTT::Logger::Instance()->in(getName());
        RTT::log(RTT::Warning) << "Using ROS Sim Clock" << RTT::endlog();
        rtt_rosclock::use_ros_clock_topic();
        rtt_rosclock::enable_sim();
        rtt_rosclock::set_sim_clock_activity(this);
    }
    
    log(RTT::Warning) << " Let's go" << endlog();
#ifdef __XENOMAI__
    log(RTT::Warning) << "** USING XENOMAI **" << endlog();
#endif
    return true;
}
bool CartOptCtrl::computeTrajectory(const double radius, const double eqradius,const double vmax, const double accmax)
{
    if(!updateState())
        return false;
    log(Warning) << "Computing Trajectory" << endlog();
    try
    {

        fk_vel_solver->JntToCart(jnt_pos_vel_kdl,frame_vel_des_kdl,this->seg_names_idx[trajectory_frame]);
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

        velpref = new VelocityProfile_Trap(vmax,accmax);
        velpref->SetProfile(0,path->PathLength());
        traject = new Trajectory_Segment(path, velpref);

        ctraject = new Trajectory_Composite();
        ctraject->Add(traject);
        ctraject->Add(new Trajectory_Stationary(1.0,frame_des_kdl));
        traj_computed = true;

    } catch(KDL::Error& error) {
            std::cout <<"I encountered this error : " << error.Description() << endlog();
            std::cout << "with the following type " << error.GetType() << endlog();
            return false;
    }

    return true;
}

void CartOptCtrl::publishTrajectory()
{
    if(!traj_computed)
        return;
    double dt=static_cast<double>(this->getPeriod());
    nav_msgs::Path path_ros;
    path_ros.header.frame_id = root_link;
    log(Debug) << "Creating Path" << endlog();
    geometry_msgs::PoseArray pose_array;
    for (double t=0.0; t <= ctraject->Duration(); t+= 0.1) {
            Frame current_pose;
            Twist current_vel,current_acc;
            
            current_pose = ctraject->Pos(t);
            current_vel = ctraject->Vel(t);
            current_acc = ctraject->Acc(t);
                        
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
/*void * CartOptCtrl::optimize(void * model)
{
    while(1)
    {
        if(lwr::run_optimize){
            static_cast<GRBModel*>(model)->optimize();
            lwr::run_optimize=false;
        }
    }
}*/
void CartOptCtrl::updateHook()
{
    static bool first_loop;
    RTT::log(RTT::Debug) << "CartOptCtrl Update at "<<rtt_rosclock::host_now()<< RTT::endlog();
    if(!updateState() /*|| !isCommandMode()*/)
        return;

#ifdef __XENOMAI__
	RTIME t_optimize,t_loop;
	std_msgs::Float64 loop_duration;
	RTIME t_loop_start = rt_timer_read();
	RTT::os::TimeService::ticks tt_start = RTT::os::TimeService::Instance()->getTicks();
#endif
    //ros::Time t_start = rtt_rosclock::host_now();
    //publishTrajectory();
    
    //log(Debug) << "Start" << endlog();

    //log(Debug) << "go" << endlog();    
    static Frame X_des,X_mes;
    static Twist Xdd_des,Xd_mes,Xd_des;
    
    // Fk -> X last frame
    jnt_to_jac_solver->JntToJac(jnt_pos_kdl,J_ati_base,this->seg_names_idx[trajectory_frame]);
    jdot_solver->JntToJacDot(jnt_pos_vel_kdl,jdot_qdot,this->seg_names_idx[trajectory_frame]);
    fk_vel_solver->JntToCart(jnt_pos_vel_kdl,tool_in_base_framevel,this->seg_names_idx[trajectory_frame]);
    X_mes =  tool_in_base_framevel.GetFrame();
    Xd_mes = tool_in_base_framevel.GetTwist();

    if(n_updates_ == 0)
    {
        X_des = X_mes;
    }


    geometry_msgs::Twist spacenav_tw;
    if(port_spacenav.read(spacenav_tw) != RTT::NoData)
    {
        // Bypassing everybody :)
        X_des.M = X_des.M * KDL::Rotation::RPY(spacenav_scale_rot * spacenav_tw.angular.x,
                                              spacenav_scale_rot * spacenav_tw.angular.y,
                                              spacenav_scale_rot * spacenav_tw.angular.z);
        X_des.p = X_des.p + KDL::Vector(spacenav_scale_trans * spacenav_tw.linear.x,
                                                  spacenav_scale_trans * spacenav_tw.linear.y,
                                                  spacenav_scale_trans * spacenav_tw.linear.z);
    }else{
        if(traj_computed)
            X_des = ctraject->Pos(t_traj_curr);
        else{
            if(!init_pos_acquired)
            {
                fk_vel_solver->JntToCart(jnt_pos_vel_kdl,frame_vel_des_kdl,this->seg_names_idx[trajectory_frame]);
                X_des =  frame_vel_des_kdl.GetFrame();
                init_pos_acquired = true;
            }
        }
    }

    
       
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
            
    if(use_xd_des_ && traj_computed)
        Xd_des = ctraject->Vel(t_traj_curr);
    else
        SetToZero(Xd_des);
    Twist dd_err = diff(Xd_mes,Xd_des);
    
    
    
    Xdd_des.vel = kp_lin*d_err.vel + kd_lin*dd_err.vel;
    
    Vector dr = kp_ang*d_err.rot;
    dr(0) = clip(dr(0),-d_ang_max_,d_ang_max_);
    dr(1) = clip(dr(1),-d_ang_max_,d_ang_max_);
    dr(2) = clip(dr(2),-d_ang_max_,d_ang_max_);
    
    Xdd_des.rot = dr + kd_ang*(dd_err.rot);
    
    if(use_xdd_des_ && traj_computed)
            Xdd_des += ctraject->Acc(t_traj_curr);

    id_dyn_solver->JntToCoriolis(jnt_pos_kdl,jnt_vel_kdl,coriolis_kdl);
    id_dyn_solver->JntToGravity(jnt_pos_kdl,gravity_kdl);
    id_dyn_solver->JntToMass(jnt_pos_kdl,mass_kdl);

    tf::twistKDLToEigen(Xdd_des,xdd_des_);
    tf::twistKDLToEigen(jdot_qdot,jdot_qdot_);
    
    //try{
#ifndef __XENOMAI__
        struct timeval tbegin,tend;
        gettimeofday(&tbegin,NULL);
#else
	RTIME t_start = rt_timer_read();
//	timespec ts = {0,0};
//	clock_gettime(CLOCK_HOST_REALTIME,&ts);
//	ros::Time start_opt = ros::Time(ts.tv_sec, ts.tv_nsec);
#endif
// // // // // // //             port_q.write(jnt_pos);
// // // // // // //             port_qdot.write(jnt_vel);
// // // // // // //             port_jacobian.write(J_ati_base.data);
// // // // // // //             port_mass.write(mass_kdl.data);
// // // // // // //             port_dt.write(kdt_*static_cast<double>(getPeriod()));
// // // // // // //             port_jdot_qdot.write(jdot_qdot_);
// // // // // // //             port_coriolis.write(coriolis_kdl.data);
// // // // // // //             port_gravity.write(gravity_kdl.data);
// // // // // // //             port_xdd_des.write(xdd_des_);
// // // // // // //             port_optimize_event.write(true);
    /*update_(jnt_pos,
                                    jnt_vel,
                                    3.*static_cast<double>(getPeriod()),
                                    J_ati_base.data,
                                    mass_kdl.data,
                                    jdot_qdot_,
                                    coriolis_kdl.data,
                                    gravity_kdl.data,
                                    xdd_des_);*/


        /*cart_model_solver_.optimize(jnt_pos,
                                    jnt_vel,
                                    
                                    3.*static_cast<double>(getPeriod()),
                                    J_ati_base.data,
                                    mass_kdl.data,
                                    jdot_qdot_,
                                    coriolis_kdl.data,
                                    gravity_kdl.data,
                                    xdd_des_,
                                    jnt_trq_cmd);

        cart_model_solver_.getQddBounds(qdd_min_ros.data,qdd_max_ros.data,qdd_des_ros.data);*/
        
        cart_model_solver_.update(jnt_pos,
                                    jnt_vel,
                                    kdt_*static_cast<double>(getPeriod()),
                                    J_ati_base.data,
                                    mass_kdl.data,
                                    jdot_qdot_,
                                    coriolis_kdl.data,
                                    gravity_kdl.data,
                                    xdd_des_);
                  
                  
	if(cart_model_solver_.optimize())
	  cart_model_solver_.getTorque(jnt_trq_cmd,false);
	else
	    jnt_trq_cmd.setZero();
                          
        
        if(use_ft_sensor_ && port_ftdata.read(wrench_msg) != RTT::NoData){
            // remove B(q,qdot)
            jnt_vel_kdl.data.setZero();
            jnt_acc_kdl.data.setZero();
            // Get the Wrench in the last segment's frame
            tf::wrenchMsgToKDL(wrench_msg.wrench,wrench_kdl);
            f_ext_kdl.back() = wrench_kdl;
            // Get The full dynamics with external forces
            id_rne_solver->CartToJnt(jnt_pos_kdl,jnt_vel_kdl,jnt_acc_kdl,f_ext_kdl,jnt_trq_kdl);
            // Get Joint Gravity torque
            //id_dyn_solver->JntToGravity(jnt_pos_kdl,gravity_kdl);
            // remove G(q)
            jnt_trq_cmd += jnt_trq_kdl.data - gravity_kdl.data;
            //RTT::log(RTT::Debug) << "CartOptCtrl add_torque "<<jnt_trq_cmd << "due to wrench "<<wrench_kdl<< RTT::endlog();
            //port_add_torque.write(jnt_trq_cmd);
        }
    
    
#ifndef __XENOMAI__
        gettimeofday(&tend,NULL);
#else
	RTIME t_stop = rt_timer_read();
//	clock_gettime(CLOCK_HOST_REALTIME,&ts);
//        ros::Time stop_opt = ros::Time(ts.tv_sec, ts.tv_nsec);
#endif
        // Don't forget to remove G(q) as Kuka adds it on KRC 
        //jnt_trq_cmd-=gravity_kdl.data;

        std_msgs::Float64 d;
#ifndef __XENOMAI__
        d.data =  1000*(tend.tv_sec-tbegin.tv_sec)+(tend.tv_usec-tbegin.tv_usec)/1000.;
#else
	t_optimize = rt_timer_ticks2ns(t_stop-t_start);
	d.data = t_optimize;
//        d.data = (stop_opt - start_opt).toSec();
#endif
    port_solver_duration.write(d);
    port_qdd_des.write(qdd_des_ros);
    port_qdd_min.write(qdd_min_ros);
    port_qdd_max.write(qdd_max_ros);
        
    sendJointTorque(jnt_trq_cmd);
    
    // Incremente traj
    if(isReadyToStart() && traj_computed && isCommandMode()){
        if( t_traj_curr <= ctraject->Duration())
            t_traj_curr += static_cast<double>(this->getPeriod());
        else
            t_traj_curr = 0.0;
    }
#ifdef __XENOMAI__
	RTIME t_loop_stop = rt_timer_read();
        
	t_loop = rt_timer_ticks2ns(t_loop_stop-t_loop_start);
        loop_duration.data = t_loop;
//	loop_duration.data = elapsed;
	port_loop_duration.write(loop_duration);
//	if(t_loop < t_optimize)
//		log(RTT::Error) << "Negative duration"<<endlog();
#endif
   // ros::Duration t_elapsed = rtt_rosclock::host_now() - t_start;
  //  elapsed = t_elapsed.toSec();
    n_updates_++;
}

}
