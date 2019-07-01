#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 

namespace ifopt { namespace Constraints {

class FrictionCone : public ConstraintSet {
    
public:
  
  struct Contact_Var_Name
  {
    std::string force_name;
    std::string position_name;
    std::string normal_name;            
  };
  
  FrictionCone(const Contact_Var_Name& contact_var_name);
  
  void setMu(const double& mu);
  
  void setForceThreshold(const double& force_thr);

    
private:
  
  Eigen::VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;
  
  Contact_Var_Name _contact_var_name;
  double _mu, _force_thr;
  Eigen::Vector3d _F, _n;
      
};

}
    
}