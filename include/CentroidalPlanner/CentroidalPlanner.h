#include <CentroidalPlanner/Ifopt/CplSolver.h>
#include <CentroidalPlanner/Ifopt/CplProblem.h>
#include <CentroidalPlanner/Environment/Superquadric.h>
#include <CentroidalPlanner/Environment/Ground.h>
#include <iostream>

namespace cpl { 
 
/**
* @brief The CentroidalPlanner class provides a generic way 
* to perform non linear optimization over centroidal statics.
*/    
class CentroidalPlanner
{
    
public:
    
    typedef std::shared_ptr<CentroidalPlanner> Ptr;
       
    struct WorkspaceBounds
    {       
        Eigen::Vector3d workspace_lb;
        Eigen::Vector3d workspace_ub;         
    };
    

    CentroidalPlanner(std::vector<std::string> contact_names,
                      double robot_mass,
                      cpl::env::EnvironmentClass::Ptr env);
    
    solver::CplProblem::Solution Solve();
    void SetPosBounds(const std::map<std::string, WorkspaceBounds>& pos_bounds_map);
    void SetPosRef(const std::map<std::string, Eigen::Vector3d>& pos_ref_map);
    void SetComRef(const Eigen::Vector3d& com_ref);
    void SetCoMWeight(const double& W_CoM);
    void SetPosWeight(const double& W_p);
    void SetForceWeight(const double& W_F);
    
private:
      
    cpl::solver::CplProblem::Ptr _cpl_problem;
    cpl::solver::CplSolver _cpl_solver;
    cpl::env::EnvironmentClass::Ptr _env;   
    std::vector<std::string> _contact_names;
    double _robot_mass;   
    cpl::solver::CplProblem::Solution _sol;
    
};
    
}