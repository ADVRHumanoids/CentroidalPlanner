#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Environment/Environment.h>
#include <CentroidalPlanner/Ifopt/CPLSolver.h>

namespace cpl { namespace solver { namespace Environment {

/**
* @brief Environment normal constraint
*/    
class EnvironmentNormal : public ifopt::ConstraintSet {
    
public:
    
    EnvironmentNormal(std::string contact_name, CPLSolver::ContactVars contact_vars, cpl::env::EnvironmentClass::Ptr env);
    
private:
    
    cpl::env::EnvironmentClass::Ptr _env; // PTR -> POLYMORPHIC BEHAVIOR
    
    Eigen::VectorXd GetValues() const override;
    VecBound GetBounds() const override;
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;
  
    std::string _contact_name; 
    CPLSolver::ContactVars _contact_vars;
    
    Eigen::Vector3d _p, _n;
    
};

} } }