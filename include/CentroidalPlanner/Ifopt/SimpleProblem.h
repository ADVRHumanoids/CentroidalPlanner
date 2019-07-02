#include <ifopt/problem.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Ifopt/Variable3D.h>
#include <CentroidalPlanner/Ifopt/Constraints/CentroidalStatics.h>
#include <CentroidalPlanner/Ifopt/CPLSolver.h>

namespace cpl { namespace solver {

/**
* @brief Simple balancing problem
*/      
class SimpleProblem : public ifopt::Problem {
    
public:
    
    typedef std::shared_ptr<SimpleProblem> Ptr;
       
    SimpleProblem(std::vector<std::string> contact_names,
                  double robot_mass);
    
private:
    
    std::map<std::string, CPLSolver::ContactVars> _contact_vars_map;  
    std::vector<std::string> _contact_names;
    double _robot_mass;
    
    ifopt::Problem _nlp;
    Variable3D::Ptr _com_var;
    
};
    
} }
