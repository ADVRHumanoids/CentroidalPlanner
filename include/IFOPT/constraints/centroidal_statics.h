#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 

namespace ifopt { namespace Constraints {

class CentroidalStatics : public ConstraintSet {
    
public:
  
  struct Contact_Var_Name
  {
    std::string force_name;
    std::string position_name;
    std::string normal_name;            
  };
  
  CentroidalStatics(const std::map<std::string, Contact_Var_Name>& contacts_map);
  
  void setMass(const double& m);
  void setManipulationWrench(const Eigen::VectorXd& wrench_manip);
    
private:
  
  Eigen::VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;
  
  std::map<std::string, Contact_Var_Name> _contacts_map;
  
  Eigen::Vector3d _CoM, _mg;
  Eigen::VectorXd _wrench_manip;
  
 
};

}
    
}