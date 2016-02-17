#ifndef __LWR_CART_SOLVER_HPP__
#define __LWR_CART_SOLVER_HPP__

#include <cart_opt_ctrl/solvers.hpp>
#include <Eigen/Dense>
#include <algorithm>

#define TORAD 3.141592653589793/180.0
#ifndef LBR_MNJ
#define LBR_MNJ 7
#endif


struct LWRLimits
{
        static const double joint_position_limit_rad[LBR_MNJ];
        static const double joint_velocity_limit_rad[LBR_MNJ];
        static const double joint_torque_limit_Nm[LBR_MNJ];
};
const double LWRLimits::joint_position_limit_rad[LBR_MNJ] = { 170*TORAD,120*TORAD,170*TORAD,120*TORAD,170*TORAD,120*TORAD,170*TORAD};
const double LWRLimits::joint_velocity_limit_rad[LBR_MNJ] = { 112.5*TORAD,112.5*TORAD,112.5*TORAD,112.5*TORAD,180*TORAD,112.5*TORAD,112.5*TORAD};
const double LWRLimits::joint_torque_limit_Nm[LBR_MNJ] = { 200,200,100,100,100,30,30};

        
namespace lwr{
template<template<unsigned int> class CartProblem,unsigned int Ndof=LBR_MNJ>
class LWRCartOptSolver : public CartProblem<Ndof>
{
public:
    LWRCartOptSolver():
    q_lim_(LWRLimits::joint_position_limit_rad),
    qdot_lim_(LWRLimits::joint_velocity_limit_rad),
    trq_lim_(LWRLimits::joint_torque_limit_Nm)
    {
    }
    void update(const Eigen::Matrix<double,LBR_MNJ,1>& q,
                const Eigen::Matrix<double,LBR_MNJ,1>& qdot,
                const double& dt_sec_since_last_optimize,
                const Eigen::Matrix<double,6,LBR_MNJ>& jacobian,
                const Eigen::Matrix<double,LBR_MNJ,LBR_MNJ>& mass,
                const Eigen::Matrix<double,6,1>& jdot_qdot,
                const Eigen::Matrix<double,LBR_MNJ,1>& coriolis,
                const Eigen::Matrix<double,LBR_MNJ,1>& gravity,
                const Eigen::Matrix<double,6,1>& xdd_des
                )
    {
        updateBounds(q,qdot,dt_sec_since_last_optimize,gravity);
        CartProblem<Ndof>::update(jacobian,mass,jdot_qdot,coriolis,gravity,xdd_des);
    }
    //bool optimize(){return gurobi::CartSolverGurobi::optimise();}
    template<class T_Array,class T2_Array,class T3_Array>
    void getQddBounds(  T_Array& qdd_min, 
                        T2_Array& qdd_max,
                        T3_Array& qdd_des
                     )
    {
        for(unsigned int i=0;i<LBR_MNJ;++i){
            qdd_min[i] = lwr_qdd_min_[i];
            qdd_max[i] = lwr_qdd_max_[i];
            qdd_des[i] = lwr_qdd_des_[i];
        }
    }
private :
    void toRad(Eigen::Matrix<double,LBR_MNJ,1>& v)
    {
        for(unsigned int i=0;i<LBR_MNJ;++i)
            v[i] = v[i] * TORAD;
    }
    void updateBounds(const Eigen::Matrix<double,LBR_MNJ,1>& q,
                        const Eigen::Matrix<double,LBR_MNJ,1>& qdot,
                        const double& dt,
                        const Eigen::Matrix<double,LBR_MNJ,1>& gravity)
    {
        updateQddBounds(q,qdot,dt);
        updateTorqueBounds(gravity);
        this->setBounds(lwr_tau_min_,lwr_tau_max_,lwr_qdd_min_,lwr_qdd_max_);
    }
    void updateQddBounds(const Eigen::Matrix<double,LBR_MNJ,1>& q,
                            const Eigen::Matrix<double,LBR_MNJ,1>& qdot,
                            const double& dt)
    {
        for(unsigned int i=0;i<LBR_MNJ;++i)
        {
            // NOTE: here we try to predict where the robot is gonna be in dt (>= 5*period) 
            lwr_qdd_max_[i] = std::min((qdot_lim_[i]  - qdot[i])/dt, (q_lim_[i] - (q[i] + qdot[i]*dt)) * (2./(dt*dt)));
            lwr_qdd_min_[i] = std::max((-qdot_lim_[i]  - qdot[i])/dt, (-q_lim_[i] - (q[i] + qdot[i]*dt)) * (2./(dt*dt)));
            
            if(lwr_qdd_min_[i] > lwr_qdd_max_[i])
            {
                double tmp = lwr_qdd_max_[i];
                lwr_qdd_max_[i] = lwr_qdd_min_[i];
                lwr_qdd_min_[i] = tmp;
            }
        }
    }
    void updateTorqueBounds(const Eigen::Matrix<double,LBR_MNJ,1>& gravity)
    {
        for(unsigned int i=0;i<LBR_MNJ;++i)
        {
            lwr_tau_max_[i] =  trq_lim_[i] - gravity[i];
            lwr_tau_min_[i] =  -trq_lim_[i] - gravity[i];
        }
    }
    const Eigen::Matrix< double, LBR_MNJ, 1 > q_lim_,qdot_lim_,trq_lim_;
    Eigen::Matrix< double, LBR_MNJ, 1 > lwr_qdd_min_,lwr_qdd_max_,lwr_qdd_des_,
                                     lwr_tau_min_,lwr_tau_max_;
   
};


}
#endif
