#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Environment/Environment.h>
#include <CentroidalPlanner/Ifopt/CplSolver.h>

namespace cpl { namespace solver {

/**
* @brief Non linear friction cone constraint
*/      
class FrictionCone : public ifopt::ConstraintSet {
    
public:
    
    typedef std::shared_ptr<FrictionCone> Ptr;
  
    FrictionCone(std::string contact_name, CplSolver::ContactVars contact_vars, cpl::env::EnvironmentClass::Ptr env);

    void SetForceThreshold(const double& force_thr);

  
private:
    
    cpl::env::EnvironmentClass::Ptr _env; // PTR -> POLYMORPHIC BEHAVIOR  

    Eigen::VectorXd GetValues() const override;
    VecBound GetBounds() const override;
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;

    std::string _contact_name; 
    CplSolver::ContactVars _contact_vars;
    double _mu, _force_thr;
    
};

} }