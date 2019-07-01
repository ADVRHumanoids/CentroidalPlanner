#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 

namespace cpl { namespace solver {

class MinimizeCentroidalVariables : public ifopt::CostTerm {
    
public:
    
  struct ContactVarName
  {
    std::string force_name;
    std::string position_name;
    std::string normal_name;            
  };
  
  MinimizeCentroidalVariables(const std::map<std::string, ContactVarName>& contacts_map);
  
  void SetCoMRef(const Eigen::Vector3d& CoM_ref);
  void SetCoMWeight(const double& W_CoM);
  void SetPositionWeight(const double& W_p);
  void SetForceWeight(const double& W_F);
  
    
private:
    
  double GetCost() const override;
  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override;
  
  std::map<std::string, ContactVarName> _contacts_map;
  
  Eigen::Vector3d _CoM, _CoM_ref;
  double _W_p, _W_F, _W_CoM;
      
};

} }
    