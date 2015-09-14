#ifndef __CART_MODEL_SOLVER_GUROBI_HPP_
#define __CART_MODEL_SOLVER_GUROBI_HPP_
// Gurobi Solver
#include "gurobi_c++.h"

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <sstream>

#define to_string( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()
        
#include <cart_opt_ctrl/cart_solver.hpp>

template<unsigned int Ndof>
class CartOptSolverGurobi: public CartOptSolver<Ndof>
{
public:
    CartOptSolverGurobi():
    env_(),
    model_(env_),
    obj_lin_(0),
    obj_quad_(0),
    ma_b_(Ndof),
    dynamics_constr_(Ndof),
    dynamics_constr_added_(Ndof,false)
    {
        this->name_ = "Gurobi";
        addVars();
        setVerbose(false);
    }
    
    virtual ~CartOptSolverGurobi(){}
    
    void setMethod(const int i)
    {
        model_.getEnv().set(GRB_IntParam_Method, i);
    }
    
    void setVerbose(const bool v)
    {
        model_.getEnv().set(GRB_IntParam_OutputFlag, (v?1:0));
    }
    void setTimeLimit(const double& t)
    {
        model_.getEnv().set(GRB_DoubleParam_TimeLimit,t);
    }
    void setBarrierConvergeanceTolerance(const double& t)
    {
        model_.getEnv().set(GRB_DoubleParam_BarConvTol,t);
    }

    template<class T1,class T2>
    void setLastSolution(T1& torque_out,
                            T2& qdd_out
                            )
    {
        //Does not work for QP problems
        for(unsigned int i=0;i<Ndof;++i)
        {
            tau_v_[i].set(GRB_DoubleAttr_PStart,static_cast<double>(torque_out[i]));
            qdd_v_[i].set(GRB_DoubleAttr_PStart,static_cast<double>(qdd_out[i]));
        }
    }
    
    void modelUpdate()
    {
        model_.update();
    }
    bool optimize(){
        try{
            model_.optimize();
            getTorque(this->torque_out_);
            getQdd(this->qdd_out_);

        } catch(GRBException e) {
            std::cerr << "Error code = " << e.getErrorCode() << std::endl;
            std::cerr << e.getMessage() << std::endl;
            return false;
        } catch(...) {
            std::cerr << "Exception during optimization" << std::endl;
            return false;
        }
        return true;
    }

    template<class T> void getTorque(T& torque,bool only_additionnal_torque=false)
    {
        try{
            for(unsigned int i=0;i<Ndof && i<torque.rows();++i)
                torque[i] = tau_v_[i].get(GRB_DoubleAttr_X);

            if(only_additionnal_torque)
                for(unsigned int i=0;i<Ndof && i<torque.rows();++i)
                    torque[i] -= this->gravity_[i];

        } catch(GRBException e) {}
    }

    template<class T> void getQdd(T& qdd)
    {
        try{
            for(unsigned int i=0;i<Ndof && i<qdd.rows();++i)
                qdd[i] = qdd_v_[i].get(GRB_DoubleAttr_X);
        } catch(GRBException e) {}
    }
    static void * optimizeasync(void * model)
    {
        reinterpret_cast<GRBModel*>(model)->optimize();
    }
private:

    void updateBound(const Eigen::Matrix<double,Ndof,1>& min,
                     const Eigen::Matrix<double,Ndof,1>& max,
                     GRBVar * vars)
    {
        for(unsigned int i=0;i<Ndof;++i)
        {
            vars[i].set(GRB_DoubleAttr_LB,min[i]);
            vars[i].set(GRB_DoubleAttr_UB,max[i]);
        }
    }

    void updateBounds()
    {     
        updateBound(this->tau_min_,this->tau_max_,tau_v_);
        updateBound(this->qdd_min_,this->qdd_max_,qdd_v_);
    }

    void updateDynamicsConstr(const Eigen::Matrix<double,Ndof,Ndof>& mass,
			   const Eigen::Matrix<double,Ndof,1>& b_qqd)
    {
	for(unsigned int i=0;i<Ndof;++i)
        {
            // M(q).qdd + b(q,qd) = T => M(q).qdd - T = -b(q,qd)
            
            if(!dynamics_constr_added_[i])
            {
                M_qdd = 0;
                for(unsigned int j=0;j<Ndof;++j)
                    M_qdd += mass(i,j)*qdd_v_[j];
                dynamics_constr_[i] = model_.addConstr(M_qdd - tau_v_[i] == -b_qqd[i]);
                dynamics_constr_added_[i] = true;
            }else{
                for(unsigned int j=0;j<Ndof;++j)
                    model_.chgCoeff(dynamics_constr_[i],qdd_v_[j],mass(i,j));
                dynamics_constr_[i].set(GRB_DoubleAttr_RHS,-b_qqd[i]);
            }
        }
    }

    void addVars()
    {
        tau_v_ = model_.addVars(Ndof);
        qdd_v_ = model_.addVars(Ndof);
        model_.update();
        for(unsigned int i=0;i<Ndof;++i){
            tau_v_[i].set(GRB_StringAttr_VarName,"tau_"+to_string(i));
            qdd_v_[i].set(GRB_StringAttr_VarName,"qdd_"+to_string(i));
        }
        model_.update();
    }
    
    void updateObj()
    {
        obj_quad_ = 0;
        obj_lin_ = 0;
	for(unsigned int i=0;i<Ndof;++i)
        {
                obj_lin_ += this->q_[i] * tau_v_[i]; 
		for(unsigned int j=0;j<Ndof;++j)
			obj_quad_+= tau_v_[i] * this->Q_(i,j) * tau_v_[j];
        }
        //this->lin_cte_ = this->cte_ * this->cte_.transpose();
	model_.setObjective(obj_quad_ + obj_lin_ /*+ this->lin_cte_*/,GRB_MINIMIZE);
    }
private:    
    GRBEnv env_;
    
    GRBModel model_;
    
    std::vector<GRBConstr> dynamics_constr_;
    
    std::vector<bool> dynamics_constr_added_;
    
    GRBLinExpr obj_lin_;
    
    GRBQuadExpr obj_quad_;
    
    GRBVar * tau_v_;
    
    GRBVar * qdd_v_;
                        
    std::vector<GRBLinExpr>     ma_b_;
    
    GRBLinExpr M_qdd;
 
};






#endif
