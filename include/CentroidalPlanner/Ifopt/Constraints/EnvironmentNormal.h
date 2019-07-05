#ifndef __ENVIRONMENT_NORMAL__
#define __ENVIRONMENT_NORMAL__


#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Environment/Environment.h>
#include <CentroidalPlanner/Ifopt/Types.h>

namespace cpl { namespace solver {

/**
* @brief Environment normal constraint
*/    
class EnvironmentNormal : public ifopt::ConstraintSet {
    
public:
    
    EnvironmentNormal(std::string contact_name, ContactVars contact_vars, env::EnvironmentClass::Ptr env);
    
private:
    
    env::EnvironmentClass::Ptr _env; // PTR -> POLYMORPHIC BEHAVIOR
    
    Eigen::VectorXd GetValues() const override;
    VecBound GetBounds() const override;
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;
  
    std::string _contact_name; 
    ContactVars _contact_vars;
    
};

} } 


#endif