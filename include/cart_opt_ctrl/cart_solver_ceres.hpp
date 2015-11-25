#ifndef __CART_SOLVER_CERES_HPP__
#define __CART_SOLVER_CERES_HPP__
#include <cart_opt_ctrl/cart_problem.hpp>
#include <ceres/ceres.h>


template<unsigned int Ndof>
struct CartOptSolverCeres {
    CartOptSolverCeres(){
        google::InitGoogleLogging("CartOptSolverCeres");
    }
    
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(10.0) - x[0];
     return true;
   }
protected:
    ceres::Problem problem;
};


template<unsigned int Ndof>
class CartOptSolverCostFunctor: public CartProblem<Ndof>
{
public:
    static const unsigned int NConstrs = Ndof;
    static const unsigned int NVars = 2*Ndof;
    CartOptSolverCostFunctor()
    {
        this->name_ = "Ceres";
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
                
        //cart_prob.init( H,g,A,lb,ub,lbA,ubA, nWSR,&cpulimit);
    }
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
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
        
        Eigen::Map<Eigen::Matrix<double,NVars,1> >(g, NVars, 1) = this->g_;
        Eigen::Map<Eigen::Matrix<double,NVars,NVars,Eigen::RowMajor> >(H, NVars, NVars) = this->H_;
        /*for(unsigned int i=0;i<NVars;++i)
            for(unsigned int j=0;j<NVars;++j)
                H[i*NVars+j] = this->H_(j,i);*/
        /*std::cout <<H_<<std::endl;    
        for(unsigned int i=0;i<NVars*NConstrs;++i)
            std::cout <<H[i]<<" ";
        std::cout <<std::endl;*/
        //std::cout <<g_<<std::endl;
        //std::cout <<H_<<std::endl;
        
    }
    void modelUpdate(){}
    void updateDynamicsConstr(const Eigen::Matrix<double,Ndof,Ndof>& mass,
                           const Eigen::Matrix<double,Ndof,1>& b_qqd)
    {
        // M(q).qdd + b(q,qd) = T => M(q).qdd  - T = - b(q,qd)

        Eigen::Map<Eigen::Matrix<double,Ndof,1> >(lbA, NConstrs, 1) = -b_qqd;
        Eigen::Map<Eigen::Matrix<double,Ndof,1> >(ubA, NConstrs, 1) = -b_qqd;
        
        A_.block(0,Ndof,Ndof,Ndof) = mass;
        Eigen::Map<Eigen::Matrix<double,NConstrs,NVars,Eigen::RowMajor> >(A, NConstrs,NVars) = A_;
        /*for(unsigned int i=0;i<NVars;++i)
            for(unsigned int j=0;j<NConstrs;++j)
                A[i*NVars+j] = A_(j,i);*/
        /*std::cout <<mass<<std::endl;
        std::cout <<A_<<std::endl;
        for(unsigned int i=0;i<NVars*NConstrs;++i)
            std::cout <<A[i]<<" ";
        std::cout <<std::endl;*/
        
    }
    template<class T> void getTorque(T& torque,bool include_gravity=false){
            for(unsigned int i=0;i<Ndof;++i)
                torque[i] = x_out[i];
    }
    template<class T> void getQdd(T& qdd){
            for(unsigned int i=0;i<Ndof;++i)
                qdd[i] = x_out[i+Ndof];
    }
    bool optimize(){
        

    }
    //DynamicsConstraints<Ndof> dynamics_contraints;
    double A[NVars*NConstrs],H[NVars*NVars],g[NVars];
    Eigen::Matrix<double,NConstrs,NVars> A_;
    Eigen::Matrix<double,NVars,NVars> H_;
    Eigen::Matrix<double,NVars,1> g_;
    double lb[NVars],ub[NVars];
    double lbA[NConstrs],ubA[NConstrs];
    bool do_init;
    double x_out[NVars];
};
#endif
