#include <CentroidalPlanner/CentroidalPlanner.h>

using namespace cpl;

CentroidalPlanner::CentroidalPlanner(std::vector< std::string > contact_names, double robot_mass, cpl::env::EnvironmentClass::Ptr env) :
    _contact_names(contact_names),
    _robot_mass(robot_mass),
    _env(env)
{
    
    _cpl_problem = std::make_shared<solver::CplProblem> (_contact_names, _robot_mass, _env);
    
    _cpl_solver.SetOption("derivative_test", "first-order");
    _cpl_solver.Solve(*_cpl_problem); 
    
     Eigen::VectorXd solution = _cpl_problem->GetOptVariables()->GetValues();     
     std::cout << "solution: " << solution << std::endl;
    
}
