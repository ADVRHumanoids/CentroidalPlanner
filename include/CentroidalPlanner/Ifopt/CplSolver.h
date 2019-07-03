#ifndef __SOLVER__
#define __SOLVER__


#include <ifopt/ipopt_solver.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Ifopt/Variable3D.h>

namespace cpl { namespace solver {

/**
* @brief Solver for CentroidalPlanner problems
*/      
class CplSolver : public ifopt::IpoptSolver {
    
public:
    
    typedef std::shared_ptr<CplSolver> Ptr;
    
    struct ContactVars
    {
        cpl::solver::Variable3D::Ptr force_var;
        cpl::solver::Variable3D::Ptr position_var;
        cpl::solver::Variable3D::Ptr normal_var;            
    };
    
};
    
} }


#endif