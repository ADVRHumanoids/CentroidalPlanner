#include <CentroidalPlanner/Environment/Environment.h>
#include <CentroidalPlanner/Ifopt/CPLSolver.h>
#include <CentroidalPlanner/Ifopt/SimpleProblem.h>

namespace cpl { 
 
/**
* @brief The CentroidalPlanner class provides a generic way 
* to perform non linear optimization over centroidal statics.
*/    
class CentroidalPlanner
{
    
public:
       
    struct WorkspaceBounds
    {       
        Eigen::Vector3d workspace_lb;
        Eigen::Vector3d workspace_ub;         
    };
    
    CentroidalPlanner(std::vector<std::string> contact_names,
                      double robot_mass);
    
    void setContactPosition(std::string, Eigen::Vector3d);
    
private:
    
    ifopt::Solver::Ptr _lcm_problem;
    ifopt::Solver::Ptr _ctr_problem;
    
    cpl::solver::SimpleProblem::Ptr _simple_problem;
    
    std::vector<std::string> _contact_names;
    double _robot_mass;   
    
};
    
}