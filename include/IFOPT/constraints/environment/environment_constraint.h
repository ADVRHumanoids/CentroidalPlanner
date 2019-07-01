#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include<Environment/Environment.h>


namespace ifopt { namespace Constraints { namespace Environment {

class Environment_Constraint : public ConstraintSet {
    
public:

    
private:
    
    Environment::EnvironmentClass::Ptr _env; // PTR -> POLYMORPHIC BEHAVIOR
};

}
}
}