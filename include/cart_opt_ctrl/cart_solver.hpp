#ifndef __CART_SOLVER_HPP__
#define __CART_SOLVER_HPP__

#include <cart_opt_ctrl/cart_problem.hpp>
template<unsigned int Ndof>
class CartOptSolver: public CartProblem<Ndof>
{
public:
    CartOptSolver():name_("solver"){}
    const std::string getName()const{return name_;}
    void update(const Eigen::Matrix<double,6,Ndof>& jacobian,
                  const Eigen::Matrix<double,Ndof,Ndof>& mass,
                  const Eigen::Matrix<double,6,1>& jdot_qdot,
                  const Eigen::Matrix<double,Ndof,1>& coriolis,
                  const Eigen::Matrix<double,Ndof,1>& gravity,
                  const Eigen::Matrix<double,6,1>& xdd_des
                  )
    {
        this->updateBounds();
        this->updateObjData(jacobian,mass,jdot_qdot,coriolis,gravity,xdd_des);
        this->updateObj();
        this->updateDynamicsConstr(mass,this->b_qqd_);
        this->modelUpdate();        
    }
    virtual void updateBounds() = 0;
    virtual void updateDynamicsConstr(const Eigen::Matrix<double,Ndof,Ndof>& mass,
                                        const Eigen::Matrix<double,Ndof,1>& b_qqd) = 0;
    virtual void updateObj()=0;
    virtual bool optimize() = 0;
    virtual void modelUpdate()= 0;
    template<class T> void getTorque(T& torque,bool only_additionnal_torque=false);
    template<class T> void getQdd(T& qdd);
protected:
    std::string name_;
};

#endif