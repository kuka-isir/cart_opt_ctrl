#ifndef __CART_SOLVER_QPOASES_HPP__
#define __CART_SOLVER_QPOASES_HPP__
#include <cart_opt_ctrl/cart_solver.hpp>
#include <cart_opt_ctrl/cart_problem.hpp>
#include <qpOASES.hpp>


USING_NAMESPACE_QPOASES

class DynamicsConstraints: public ConstraintProduct{
public:
    /** Default constructor. */
    DynamicsConstraints(){}
    virtual int operator()( int constrIndex,
                            const real_t* const x,
                            real_t* const constrValue ) const
    {
        
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
};

template<unsigned int Ndof>
class CartSolverqpOASES: public CartOptSolver<Ndof>
{
public:
    CartSolverqpOASES():
    cart_prob(2*Ndof,Ndof+2*Ndof)
    {
        Options options;
        options.enableEqualities = BT_TRUE;
        cart_prob.setOptions( options );
        cart_prob.setConstraintProduct(&dynamics_contrains);
        cart_prob.setPrintLevel(qpOASES::PL_DEBUG_ITER);
        
        //cart_prob.init( Q_.data(),q_.data(),A,lb,ub,lbA,ubA, nWSR,0 );
        
    }
    void update(const Eigen::Matrix<double,6,Ndof>& jacobian,
                  const Eigen::Matrix<double,Ndof,Ndof>& mass,
                  const Eigen::Matrix<double,6,1>& jdot_qdot,
                  const Eigen::Matrix<double,Ndof,1>& coriolis,
                  const Eigen::Matrix<double,Ndof,1>& gravity,
                  const Eigen::Matrix<double,6,1>& xdd_des
                  )
    {
    }

    bool optimize(){}
    SQProblem cart_prob;
    DynamicsConstraints dynamics_contrains;
};

#endif