#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Environment/Environment.h>

namespace cpl { namespace solver { namespace Environment {

class EnvironmentConstraint : public ifopt::ConstraintSet {
    
public:

    
private:
    
    cpl::env::EnvironmentClass::Ptr _env; // PTR -> POLYMORPHIC BEHAVIOR
};

} } }