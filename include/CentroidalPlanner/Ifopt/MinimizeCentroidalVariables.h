#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Ifopt/CPLSolver.h>

namespace cpl { namespace solver {

/**
* @brief Miniminization of: CoM position, contact positions and contact forces
*/  
class MinimizeCentroidalVariables : public ifopt::CostTerm {
    
public:

  MinimizeCentroidalVariables(std::map<std::string, CPLSolver::ContactVars> contact_vars_map);
  
  void SetCoMRef(const Eigen::Vector3d& CoM_ref);
  void SetCoMWeight(const double& W_CoM);
  void SetPositionWeight(const double& W_p);
  void SetForceWeight(const double& W_F);
  
    
private:
    
  double GetCost() const override;
  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override;
  
  std::map<std::string, CPLSolver::ContactVars> _contact_vars_map;
  
  Eigen::Vector3d _CoM, _CoM_ref;
  double _W_p, _W_F, _W_CoM;
      
};

} }
    