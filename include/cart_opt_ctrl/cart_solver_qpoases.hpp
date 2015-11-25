#ifndef __CART_SOLVER_QPOASES_HPP__
#define __CART_SOLVER_QPOASES_HPP__
#include <cart_opt_ctrl/cart_solver.hpp>
#include <cart_opt_ctrl/cart_problem.hpp>
#include <qpOASES.hpp>
#include <iostream>

USING_NAMESPACE_QPOASES

template<unsigned int Ndof>
class CartOptSolverqpOASES: public CartOptSolver<Ndof>
{
public:
    static const unsigned int NConstrs = Ndof;
    static const unsigned int NVars = 2*Ndof;
    CartOptSolverqpOASES():
    nWSR(1000),
    cpulimit(0.0),
    do_init(true),
    cart_prob(NVars,NConstrs)
    {
        this->name_="qpOASES";
        Options options;
        options.enableRegularisation = qpOASES::BT_TRUE;
        //options.setToMPC();
        options.enableEqualities = BT_TRUE;
        cart_prob.setOptions( options );
        cart_prob.setPrintLevel(qpOASES::PL_NONE);
        
        A_.setZero();
        H_.setZero();
        g_.setZero();
        for(unsigned int i=0;i<Ndof;++i)
        {
            A_(i,i) = -1.;
            //H_(Ndof+i,Ndof+i) = 1.;
        }
        setZero(lbA,NConstrs);
        setZero(ubA,NConstrs);
        setZero(lb,NVars);
        setZero(ub,NVars);
        setZero(H,NVars*NVars);
        setZero(g,NVars); 
        setZero(A,NVars*NConstrs);
        setZero(x_out,NVars);
    }
    template<typename T> void setZero(T*array,unsigned int size)
    {
        for(unsigned int i=0;i<size;++i)
            array[i] = 0.0;
    }
    void updateBounds(){
        for(unsigned int i=0;i<Ndof;++i)
        {
            lb[i] = this->tau_min_[i];
            ub[i] = this->tau_max_[i];
            lb[Ndof+i] = this->qdd_min_[i];
            ub[Ndof+i] = this->qdd_max_[i];
        }     
    }
    void updateObj(){
        g_.head(Ndof) = this->q_;
        H_.block(0,0,Ndof,Ndof) = this->Q_;
        
        Eigen::Map<Eigen::Matrix<double,NVars,1> >(g, NVars, 1) = g_;
        Eigen::Map<Eigen::Matrix<double,NVars,NVars,Eigen::RowMajor> >(H, NVars, NVars) = H_;
    }
    void modelUpdate(){}
    void updateDynamicsConstr(const Eigen::Matrix<double,Ndof,Ndof>& mass,
                           const Eigen::Matrix<double,Ndof,1>& b_qqd)
    {
        // M(q).qdd + b(q,qd) = T => M(q).qdd  - T = - b(q,qd)

        //Eigen::Map<Eigen::Matrix<double,Ndof,1> >(lbA, NConstrs, 1) = -b_qqd;
        //Eigen::Map<Eigen::Matrix<double,Ndof,1> >(ubA, NConstrs, 1) = -b_qqd;
        Eigen::Map<Eigen::Matrix<double,Ndof,1> >(eqA, NConstrs, 1) = -b_qqd;
        A_.block(0,Ndof,Ndof,Ndof) = mass;
        Eigen::Map<Eigen::Matrix<double,NConstrs,NVars,Eigen::RowMajor> >(A, NConstrs,NVars) = A_;
        
    }
    template<class T> void getTorque(T& torque,bool include_gravity=false){
        if(ret_ == qpOASES::SUCCESSFUL_RETURN)
            if(include_gravity)
                for(unsigned int i=0;i<Ndof;++i)
                    torque[i] = x_out[i];
            else
                for(unsigned int i=0;i<Ndof;++i)
                    torque[i] = x_out[i]-this->gravity_[i];
    }
    template<class T> void getQdd(T& qdd){
        if(ret_ == qpOASES::SUCCESSFUL_RETURN)
            for(unsigned int i=0;i<Ndof;++i)
                qdd[i] = x_out[i+Ndof];
    }
    bool optimize(){
        nWSR = 100;
        if(do_init)
        {
            //nWSR = 1E6;
            ret_ = cart_prob.init( H,g,A,lb,ub,eqA,eqA, nWSR,&cpulimit);
            do_init = false;
        }else
            ret_ = cart_prob.hotstart( H,g,A,lb,ub,eqA,eqA, nWSR,&cpulimit);
        cart_prob.getPrimalSolution(x_out);
        return ret_ == qpOASES::SUCCESSFUL_RETURN;
    }
    SQProblem cart_prob;
    real_t A[NVars*NConstrs],H[NVars*NVars],g[NVars];
    Eigen::Matrix<double,NConstrs,NVars> A_;
    Eigen::Matrix<double,NVars,NVars> H_;
    Eigen::Matrix<double,NVars,1> g_;
    real_t lb[NVars],ub[NVars];
    real_t lbA[NConstrs],ubA[NConstrs],eqA[NVars];
    int nWSR;
    real_t cpulimit;
    bool do_init;
    real_t x_out[NVars];
    returnValue ret_;
};

#endif
