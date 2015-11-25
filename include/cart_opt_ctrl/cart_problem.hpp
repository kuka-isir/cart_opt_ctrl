#ifndef __CART_SOLVER_HPP__
#define __CART_SOLVER_HPP__

template<unsigned int Ndof>
class CartProblem
{
public:
    CartProblem():name_("solver")
    {
        Q_.setZero();
        q_.setZero();
        torque_out_.setZero();
        qdd_out_.setZero();
    }
    const std::string getName()const{return name_;}
    void update(const Eigen::Matrix<double,6,Ndof>& jacobian,
                  const Eigen::Matrix<double,Ndof,Ndof>& mass,
                  const Eigen::Matrix<double,6,1>& jdot_qdot,
                  const Eigen::Matrix<double,Ndof,1>& coriolis,
                  const Eigen::Matrix<double,Ndof,1>& gravity,
                  const Eigen::Matrix<double,Ndof,1>& add_torque,
                  const Eigen::Matrix<double,6,1>& xdd_des
                  )
    {
        this->updateBounds();
        this->updateObjData(jacobian,mass,jdot_qdot,coriolis,gravity,add_torque,xdd_des);
        this->updateObj();
        this->updateDynamicsConstr(mass,this->b_qqd_);
        this->modelUpdate();        
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

    virtual void updateBounds() = 0;
    virtual void updateDynamicsConstr(const Eigen::Matrix<double,Ndof,Ndof>& mass,
                                        const Eigen::Matrix<double,Ndof,1>& b_qqd) = 0;
    virtual void updateObj()=0;
    virtual bool optimize() = 0;
    virtual void modelUpdate()= 0;
    template<class T> void getTorque(T& torque,bool include_gravity=true);
    template<class T> void getQdd(T& qdd);

protected:
    void updateObjData( const Eigen::Matrix<double,6,Ndof>& jacobian,
                        const Eigen::Matrix<double,Ndof,Ndof>& mass,
                        const Eigen::Matrix<double,6,1>& jdot_qdot,
                        const Eigen::Matrix<double,Ndof,1>& coriolis,
                        const Eigen::Matrix<double,Ndof,1>& gravity,
                        const Eigen::Matrix<double,Ndof,1>& add_torque,
                        const Eigen::Matrix<double,6,1>& xdd_des)
    {
        gravity_ = gravity;
        mass_inv_ = mass.inverse();
        a_ = jacobian * mass_inv_;
        b_qqd_ = coriolis + gravity + add_torque;
        cte_ = jdot_qdot.transpose() - (a_ * b_qqd_).transpose() - xdd_des.transpose();
        q_ = 2. * (cte_  * a_);
        Q_ = a_.transpose() * a_;

    }

    Eigen::Matrix< double, Ndof, 1 > tau_min_,tau_max_,
                                     qdd_min_,qdd_max_,
                                     torque_out_,qdd_out_,
                                     b_qqd_;

    Eigen::Matrix<double,6,Ndof> a_;

    Eigen::Matrix<double,1,6> cte_;

    Eigen::Matrix<double,Ndof,Ndof> mass_inv_;
    Eigen::Matrix<double,Ndof,Ndof> Q_;

    Eigen::Matrix<double,1,Ndof> q_;

    Eigen::Matrix<double,Ndof,1> gravity_;

    double lin_cte_;
    std::string name_;
};

#endif
