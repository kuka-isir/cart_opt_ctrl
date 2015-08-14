#ifndef __CART_MODEL_SOLVER_HPP_
#define __CART_MODEL_SOLVER_HPP_
// Gurobi Solver
#include "gurobi_c++.h"
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include <sstream>

#define to_string( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

#define TORAD 3.141592653589793/180.0;        

#ifndef Ndof
#define Ndof 7
#endif

namespace gurobi{

class CartOptSolver
{
public:
    CartOptSolver():
    env_(),
    model_(env_),
    n_dof_(Ndof),
    tau_v_(Ndof),
    qdd_v_(Ndof),
    obj_lin_(0),
    obj_quad_(0),
    ma_b_(Ndof),
    initialized_(false),
    dynamics_constr_(Ndof)
    {
        Q_.setZero();
        q_.setZero();
        setVerbose(false);
    }
    virtual ~CartOptSolver(){}
    
    void setVerbose(bool v)
    {
        model_.getEnv().set(GRB_IntParam_OutputFlag, (v?1:0));
    }
    void setTimeLimit(const double& t)
    {
        if(t>0.)
            model_.getEnv().set(GRB_DoubleParam_TimeLimit,t);
    }
    void setBarrierConvergeanceTolerance(const double& t)
    {
        model_.getEnv().set(GRB_DoubleParam_BarConvTol,t);
    }
    void setBounds(const Eigen::Matrix< double, Ndof, 1  >& tau_min, 
              const Eigen::Matrix< double, Ndof, 1  >& tau_max,
              const Eigen::Matrix< double, Ndof, 1  >& qdd_min, 
              const Eigen::Matrix< double, Ndof, 1  >& qdd_max)
    {
        tau_min_ = tau_min;
        tau_max_ = tau_max;
        qdd_min_ = qdd_min;
        qdd_max_ = qdd_max;
    }
    
    template<class T1,class T2>
    bool optimize(const Eigen::Matrix<double,6,Ndof>& jacobian,
                  const Eigen::Matrix<double,Ndof,Ndof>& mass,
                  const Eigen::Matrix<double,6,1>& jdot_qdot,
                  const Eigen::Matrix<double,Ndof,1>& coriolis,
                  const Eigen::Matrix<double,Ndof,1>& gravity,
                  const Eigen::Matrix<double,6,1>& xdd_des,
                  T1& torque_out,
                  T2& qdd_out
                  )
    {
        updateBounds();
        updateObjData(jacobian,mass,jdot_qdot,coriolis,gravity,xdd_des);
        updateObj();
        updateDynamicsData(mass,coriolis,gravity);
        updateDynamicsConstr();
        try{
            // Find the solution
            model_.optimize();
            
            // Write solution to VectorXd
            getTorque(torque_out);
            getQdd(qdd_out);
            removeData();
            
        } catch(GRBException e) {
            std::cout << "Error code = " << e.getErrorCode() << std::endl;
            std::cout << e.getMessage() << std::endl;
            return false;
        } catch(...) {
            std::cout << "Exception during optimization" << std::endl;
            return false;
        }
        return true;
    }

private:
    template<class T>
    void getTorque(T& torque)
    {
        for(unsigned int i=0;i<Ndof && i<torque.rows();++i)
            torque[i] = tau_v_[i].get(GRB_DoubleAttr_X);
    }
    template<class T>
    void getQdd(T& qdd)
    {
        for(unsigned int i=0;i<Ndof && i<qdd.rows();++i)
            qdd[i] = qdd_v_[i].get(GRB_DoubleAttr_X);
    }
    void updateBounds()
    {     
        addVars(tau_v_,tau_min_,tau_max_);
        addVars(qdd_v_,qdd_min_,qdd_max_);
        model_.update();
    }
    void updateDynamicsData(const Eigen::Matrix<double,Ndof,Ndof>& mass,
			   const Eigen::Matrix<double,Ndof,1>& coriolis,
			   const Eigen::Matrix<double,Ndof,1>& gravity)
    {
	for(unsigned int i=0;i<Ndof;++i){
		ma_b_[i] = 0;
		for(unsigned int j=0;j<Ndof;++j){
                        ma_b_[i] += mass(i,j) * qdd_v_[j];
		}
		ma_b_[i] += coriolis[i] + gravity[i];
	}	
    }
    void updateDynamicsConstr()
    {
        for(unsigned int i=0;i<Ndof;++i)
            dynamics_constr_[i] = model_.addConstr(ma_b_[i] == tau_v_[i]);
    }
    
    void removeData()
    {
        for(unsigned int i=0;i<Ndof;++i){
                model_.remove(dynamics_constr_[i]);
                model_.remove(tau_v_[i]);
                model_.remove(qdd_v_[i]);
        }
    }
    
    void resetModel()
    {
        model_.reset();
    }
    void addVars(std::vector<GRBVar>& vars, 
                 const Eigen::Matrix< double, Ndof, 1  >& min,
                 const Eigen::Matrix< double, Ndof, 1  >& max   /*,
                 const std::string& name,
                 const Eigen::Matrix< double, Ndof, 1  > obj = Eigen::Matrix< double, Ndof, 1  >::Zero(),
                 const std::vector<char> type = std::vector<char>(Ndof,GRB_CONTINUOUS)*/)
    {
        for(unsigned int i=0;i<Ndof;++i)
            vars[i] = model_.addVar(min[i],max[i],0,GRB_CONTINUOUS);//obj[i],type[i],name+"_"+to_string(i));
        //model_.update();
    }
    
    void updateObjData(	const Eigen::Matrix<double,6,Ndof>& jacobian,
		    	const Eigen::Matrix<double,Ndof,Ndof>& mass,
		    	const Eigen::Matrix<double,6,1>& jdot_qdot,
		    	const Eigen::Matrix<double,Ndof,1>& coriolis,
			const Eigen::Matrix<double,Ndof,1>& gravity,
			const Eigen::Matrix<double,6,1>& xdd_des)	
    {
        mass_inv_ = mass.inverse();
        a_ = jacobian * mass_inv_;
        cte_ = jdot_qdot.transpose() - (a_ * (coriolis + gravity)).transpose() - xdd_des.transpose();
        q_ = 2. * (cte_  * a_);
        Q_ = a_.transpose() * a_;

    }
    bool updateObj()
    {
        obj_quad_ = 0;
        obj_lin_ = 0;
	for(unsigned int i=0;i<Ndof;++i)
        {
                obj_lin_ += q_[i] * tau_v_[i]; 
		for(unsigned int j=0;j<Ndof;++j)
			obj_quad_+= tau_v_[i] * Q_(i,j) * tau_v_[j];
        }
        lin_cte_ = cte_ * cte_.transpose();
	model_.setObjective(obj_quad_ + obj_lin_ + lin_cte_,GRB_MINIMIZE);
	//model_.update();
        return true;
    }
private:
    bool initialized_;
    
    const unsigned int n_dof_;
    
    GRBEnv env_;
    
    GRBModel model_;
    
    std::vector<GRBConstr> dynamics_constr_;
    
    GRBLinExpr obj_lin_;
    
    GRBQuadExpr obj_quad_;
    
    std::vector<GRBVar> tau_v_,
                        qdd_v_;
                        
    Eigen::Matrix< double, Ndof, 1 > tau_min_,tau_max_,
                                     qdd_min_,qdd_max_;
                        
    std::vector<GRBLinExpr>     ma_b_;                        
                                
    Eigen::Matrix<double,6,Ndof> a_;
    
    Eigen::Matrix<double,1,6> cte_; 
    
    Eigen::Matrix<double,Ndof,Ndof> mass_inv_,
                                    Q_;
                                    
    Eigen::Matrix<double,1,Ndof> q_;
           
    double lin_cte_;
 
};
}


namespace lwr{
class LWRCartOptSolver : public gurobi::CartOptSolver
{
public:
    LWRCartOptSolver(){
        // According to Kuka manual
        q_lim_ << 170,120,170,120,170,120,170;
        qdot_lim_ << 112.5,112.5,112.5,112.5,180,112.5,112.5;
        trq_lim_ << 200,200,100,100,100,30,30;
        
        toRad(q_lim_);
        toRad(qdot_lim_);
    }
    bool optimize(const Eigen::Matrix<double,Ndof,1>& q,
                const Eigen::Matrix<double,Ndof,1>& qdot,
                const double& dt_sec_since_last_optimize,
                const Eigen::Matrix<double,6,Ndof>& jacobian,
                const Eigen::Matrix<double,Ndof,Ndof>& mass,
                const Eigen::Matrix<double,6,1>& jdot_qdot,
                const Eigen::Matrix<double,Ndof,1>& coriolis,
                const Eigen::Matrix<double,Ndof,1>& gravity,
                const Eigen::Matrix<double,6,1>& xdd_des,
                Eigen::VectorXd& torque_out
                )
    {
        updateBounds(q,qdot,dt_sec_since_last_optimize,gravity);
        return gurobi::CartOptSolver::optimize(jacobian,mass,jdot_qdot,coriolis,gravity,xdd_des,torque_out,lwr_qdd_des_);
    }
    template<class T_Array,class T2_Array,class T3_Array>
    void getQddBounds(  T_Array& qdd_min, 
                        T2_Array& qdd_max,
                        T3_Array& qdd_des
                     )
    {
        for(unsigned int i=0;i<Ndof;++i){
            qdd_min[i] = lwr_qdd_min_[i];
            qdd_max[i] = lwr_qdd_max_[i];
            qdd_des[i] = lwr_qdd_des_[i];
        }
    }
private :
    void toRad(Eigen::Matrix<double,Ndof,1>& v)
    {
        for(unsigned int i=0;i<Ndof;++i)
            v[i] = v[i] * TORAD;
    }
    void updateBounds(const Eigen::Matrix<double,Ndof,1>& q,
                        const Eigen::Matrix<double,Ndof,1>& qdot,
                        const double& dt,
                        const Eigen::Matrix<double,Ndof,1>& gravity)
    {
        updateQddBounds(q,qdot,dt);
        updateTorqueBounds(gravity);
        this->setBounds(lwr_tau_min_,lwr_tau_max_,lwr_qdd_min_,lwr_qdd_max_);
    }
    void updateQddBounds(const Eigen::Matrix<double,Ndof,1>& q,
                            const Eigen::Matrix<double,Ndof,1>& qdot,
                            const double& dt)
    {
        for(unsigned int i=0;i<Ndof;++i)
        {
            // BUG: When dt is too small (~0.001), numerical errors are big and limits become crazy. Use dt = 5*dt (predictive) 
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
    void updateTorqueBounds(const Eigen::Matrix<double,Ndof,1>& gravity)
    {
        for(unsigned int i=0;i<Ndof;++i)
        {
            lwr_tau_max_[i] =  trq_lim_[i] - gravity[i];
            lwr_tau_min_[i] =  -trq_lim_[i] - gravity[i];
        }
    }
    Eigen::Matrix< double, Ndof, 1 > q_lim_,qdot_lim_,trq_lim_,
                                     lwr_qdd_min_,lwr_qdd_max_,lwr_qdd_des_,
                                     lwr_tau_min_,lwr_tau_max_;
   
};
    
}




#endif
