#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Ifopt/CplSolver.h>
#include <CentroidalPlanner/Ifopt/Variable3D.h>

namespace cpl { namespace solver {

/**
* @brief Miniminization of: CoM position, contact positions and contact forces
*/  
class MinimizeCentroidalVariables : public ifopt::CostTerm {
    
public:
    
    typedef std::shared_ptr<MinimizeCentroidalVariables> Ptr;

    MinimizeCentroidalVariables(std::map<std::string, CplSolver::ContactVars> contact_vars_map,
                                Variable3D::Ptr com_var);

    void SetCoMRef(const Eigen::Vector3d& CoM_ref);
    void SetCoMWeight(const double& W_CoM);
    void SetPositionWeight(const double& W_p);
    void SetForceWeight(const double& W_F);
  
private:
    
    double GetCost() const override;
    void FillJacobianBlock (std::string var_set, Jacobian& jac) const override;

    std::map<std::string, CplSolver::ContactVars> _contact_vars_map;

    Eigen::Vector3d _CoM_ref;
    double _W_p, _W_F, _W_CoM;
    
    Variable3D::Ptr _com_var;
    
};

} }
    