#include <ifopt/problem.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Ifopt/Variable3D.h>
#include <CentroidalPlanner/Ifopt/Constraints/CentroidalStatics.h>
#include <CentroidalPlanner/Ifopt/Constraints/EnvironmentConstraint.h>
#include <CentroidalPlanner/Ifopt/Constraints/EnvironmentNormal.h>
#include <CentroidalPlanner/Ifopt/Constraints/FrictionCone.h>
#include <CentroidalPlanner/Ifopt/MinimizeCentroidalVariables.h>
#include <CentroidalPlanner/Ifopt/CplSolver.h>
#include <CentroidalPlanner/Environment/Environment.h>

namespace cpl { namespace solver {

/**
* @brief Centroidal statics problem with environment
*/      
class CplProblem : public ifopt::Problem {
    
public:
    
    typedef std::shared_ptr<CplProblem> Ptr;
       
    CplProblem(std::vector<std::string> contact_names,
                  double robot_mass,
                  cpl::env::EnvironmentClass::Ptr env);
    
private:
    
    std::map<std::string, CplSolver::ContactVars> _contact_vars_map;  
    std::vector<std::string> _contact_names;
    double _robot_mass;
    
    cpl::env::EnvironmentClass::Ptr _env;
    
    Variable3D::Ptr _com_var;
    CentroidalStatics::Ptr _centroidal_statics;
    EnvironmentConstraint::Ptr _env_const;
    EnvironmentNormal::Ptr _env_normal;
    FrictionCone::Ptr _friction_cone;
    MinimizeCentroidalVariables::Ptr _cost;
    
};
    
} }
