#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 

namespace cpl { namespace solver {

class FrictionCone : public ifopt::ConstraintSet {
    
public:
  
  struct ContactVarName
  {
    std::string force_name;
    std::string position_name;
    std::string normal_name;            
  };
  
  FrictionCone(const ContactVarName& contact_var_name);
  
  void setMu(const double& mu);
  
  void setForceThreshold(const double& force_thr);

    
private:
  
  Eigen::VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;
  
  ContactVarName _contact_var_name;
  double _mu, _force_thr;
  Eigen::Vector3d _F, _n;
      
};

} }