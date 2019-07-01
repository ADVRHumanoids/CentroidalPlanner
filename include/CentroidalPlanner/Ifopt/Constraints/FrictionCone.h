#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Environment/Environment.h>

namespace cpl { namespace solver {

class FrictionCone : public ifopt::ConstraintSet {
    
public:
  
    struct ContactVarName
    {
        std::string force_name;
        std::string position_name;
        std::string normal_name;            
    };

    FrictionCone(const ContactVarName& contact_var_name, cpl::env::EnvironmentClass::Ptr& env);

    void SetMu(const double& mu);

    void SetForceThreshold(const double& force_thr);

  
private:
    
    cpl::env::EnvironmentClass::Ptr _env; // PTR -> POLYMORPHIC BEHAVIOR  

    Eigen::VectorXd GetValues() const override;
    VecBound GetBounds() const override;
    void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;

    ContactVarName _contact_var_name;
    double _mu, _force_thr;
    Eigen::Vector3d _F, _n;
    
};

} }