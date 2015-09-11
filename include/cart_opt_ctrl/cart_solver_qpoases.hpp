#ifndef __CART_SOLVER_QPOASES_HPP__
#define __CART_SOLVER_QPOASES_HPP__
#include <cart_opt_ctrl/cart_solver.hpp>
#include <cart_opt_ctrl/cart_problem.hpp>
#include <qpOASES.hpp>
#include <iostream>

USING_NAMESPACE_QPOASES
template<unsigned int Ndof>
class DynamicsConstraints: public ConstraintProduct{
public:
    /** Default constructor. */
    DynamicsConstraints(){
        mass_.setZero();
        b_qqd_.setZero();
    }
    virtual int operator()( int constrIndex,
                            const real_t* const x,
                            real_t* const constrValue ) const
    {
        // // M(q).qdd + b(q,qd) = T => M(q).qdd  - T = - b(q,qd)
        //std::cout << "**************************************************UP"<<std::endl;
        int i = constrIndex;
        //std::cout << "constrValue["<<i<<"]"<<std::endl;
        real_t M_qdd=0;
        for(unsigned int j=0;j<Ndof;++j)
                    M_qdd += mass_(i,j)*x[Ndof+j];
        constrValue[i] = M_qdd + b_qqd_[i] - x[i];
        //std::cout << "-------------------------------------------------------"<<std::endl;
        
        return 0;
        
    }
    
    /** Constructor. */
    DynamicsConstraints(int_t _nV,
                        int_t _nC,
                        real_t* _A)
    {
            nV = _nV;
            nC = _nC;
            A  = _A;
    }
    void updateDynamicsConstr(const Eigen::Matrix<double,Ndof,Ndof>& mass,
                           const Eigen::Matrix<double,Ndof,1>& b_qqd)
    {
        mass_ = mass;
        b_qqd_ = b_qqd;
    }
    /** Destructor. */
    virtual ~DynamicsConstraints( ) {};
    
    /** Assignment operator (flat copy). */
    DynamicsConstraints& operator=( const DynamicsConstraints& rhs)
    {
            if ( this != &rhs )
            {
                    nV = rhs.nV;
                    nC = rhs.nC;
                    A  = rhs.A;
            }
            return *this;
    };
protected:
        int_t nV;               /**< Number of variables. */
        int_t nC;               /**< Number of constraints. */
        real_t* A;              /**< Pointer to full constraint matrix (typically not needed!). */
        Eigen::Matrix<double,Ndof,Ndof> mass_; /**< Mass Matrix */
        Eigen::Matrix<double,Ndof,1> b_qqd_;   /**< Coriolis + Gravity terms */
};

template<unsigned int Ndof>
class CartOptSolverqpOASES: public CartOptSolver<Ndof>
{
public:
    static const unsigned int NConstrs = Ndof;
    static const unsigned int NVars = 2*Ndof;
    CartOptSolverqpOASES():
    nWSR(10000000),
    cpulimit(0.0),
    do_init(true),
    cart_prob(NVars,NConstrs)
    //dynamics_contraints(NVars,NConstrs,this->A)
    {
        Options options;
        //options.enableEqualities = BT_TRUE;
        cart_prob.setOptions( options );
        //cart_prob.setConstraintProduct(&dynamics_contraints);
        cart_prob.setPrintLevel(qpOASES::PL_DEBUG_ITER);
        
        A_.setZero();
        H_.setZero();
        g_.setZero();//setConstant(1.0);
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
        H_.block(0,0,Ndof,Ndof) = 2.0*this->Q_;
        
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
    template<class T> void getTorque(T& torque,bool only_additionnal_torque=false){
        if(ret_ == qpOASES::SUCCESSFUL_RETURN)
            for(unsigned int i=0;i<Ndof;++i)
                torque[i] = x_out[i];
    }
    template<class T> void getQdd(T& qdd){
        if(ret_ == qpOASES::SUCCESSFUL_RETURN)
            for(unsigned int i=0;i<Ndof;++i)
                qdd[i] = x_out[i+Ndof];
    }
    bool optimize(){
        
        if(do_init)
        {
            //if(SUCCESSFUL_RETURN == cart_prob.init( H,g,A,lb,ub,lbA,ubA, nWSR,&cpulimit))
                ret_ = cart_prob.init( H,g,A,lb,ub,lbA,ubA, nWSR,&cpulimit);
                do_init = false;
        }else
            ret_ = cart_prob.hotstart( H,g,A,lb,ub,lbA,ubA, nWSR,&cpulimit);
        cart_prob.getPrimalSolution(x_out);
    }
    SQProblem cart_prob;
    //DynamicsConstraints<Ndof> dynamics_contraints;
    real_t A[NVars*NConstrs],H[NVars*NVars],g[NVars];
    Eigen::Matrix<double,NConstrs,NVars> A_;
    Eigen::Matrix<double,NVars,NVars> H_;
    Eigen::Matrix<double,NVars,1> g_;
    real_t lb[NVars],ub[NVars];
    real_t lbA[NConstrs],ubA[NConstrs];
    int_t nWSR;
    real_t cpulimit;
    bool do_init;
    real_t x_out[NVars];
    returnValue ret_;
};

#endif