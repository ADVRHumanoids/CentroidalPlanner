#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 

namespace cpl { namespace ifopt {
   
class Solver 
{
    
public:
    
    typedef std::shared_ptr<Solver> Ptr;
    
    struct ContactVars
    {
        ifopt::Component::Ptr force_var;
        ifopt::Component::Ptr position_var;
        ifopt::Component::Ptr normal_var;            
    };

    
};
    
} }