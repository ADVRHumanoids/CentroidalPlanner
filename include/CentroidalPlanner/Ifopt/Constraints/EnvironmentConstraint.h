#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Environment/Environment.h>
#include <CentroidalPlanner/Ifopt/CplSolver.h>

namespace cpl { namespace solver {

/**
* @brief Environment continuous function constraint
*/
class EnvironmentConstraint : public ifopt::ConstraintSet {
    
public:  

    EnvironmentConstraint(std::string contact_name, CplSolver::ContactVars contact_vars, cpl::env::EnvironmentClass::Ptr env);
    
private:
    
    cpl::env::EnvironmentClass::Ptr _env; // PTR -> POLYMORPHIC BEHAVIOR
    
    Eigen::VectorXd GetValues() const override;
    VecBound GetBounds() const override;
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;
  
    std::string _contact_name; 
    CplSolver::ContactVars _contact_vars;    
    
};

} } 