#ifndef __SOLVERS_HPP__
#define __SOLVERS_HPP__

#ifdef USE_GUROBI
#include <cart_opt_ctrl/cart_solver_gurobi.hpp>
#endif

#ifdef USE_QPOASES
#include <cart_opt_ctrl/cart_solver_qpoases.hpp>
#endif

#ifdef USE_CERES
#include <cart_opt_ctrl/cart_solver_ceres.hpp>
#endif

#endif