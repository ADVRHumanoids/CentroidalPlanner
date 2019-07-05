#ifndef __ENVIRONMENT_CONSTRAINT__
#define __ENVIRONMENT_CONSTRAINT__


#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Environment/Environment.h>
#include <CentroidalPlanner/Ifopt/Types.h>

namespace cpl { namespace solver {

/**
* @brief Environment continuous function constraint
*/
class EnvironmentConstraint : public ifopt::ConstraintSet {
    
public:  

    EnvironmentConstraint(std::string contact_name, 
                          ContactVars contact_vars, 
                          env::EnvironmentClass::Ptr env);
    
private:
    
    Eigen::VectorXd GetValues() const override;
    
    VecBound GetBounds() const override;
    
    void FillJacobianBlock (std::string var_set,
                            Jacobian& jac_block) const override;
      
    std::string _contact_name; 
    ContactVars _contact_vars;  
    env::EnvironmentClass::Ptr _env; 
    
};

} } 


#endif