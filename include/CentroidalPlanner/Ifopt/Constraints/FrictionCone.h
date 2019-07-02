#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Environment/Environment.h>
#include <CentroidalPlanner/Ifopt/CPLSolver.h>

namespace cpl { namespace solver {

/**
* @brief Non linear friction cone constraint
*/      
class FrictionCone : public ifopt::ConstraintSet {
    
public:
  
    FrictionCone(std::string contact_name, CPLSolver::ContactVars contact_vars, cpl::env::EnvironmentClass::Ptr env);

    void SetMu(const double& mu);

    void SetForceThreshold(const double& force_thr);

  
private:
    
    cpl::env::EnvironmentClass::Ptr _env; // PTR -> POLYMORPHIC BEHAVIOR  

    Eigen::VectorXd GetValues() const override;
    VecBound GetBounds() const override;
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;

    std::string _contact_name; 
    CPLSolver::ContactVars _contact_vars;
    double _mu, _force_thr;
    Eigen::Vector3d _F, _n;
    
};

} }