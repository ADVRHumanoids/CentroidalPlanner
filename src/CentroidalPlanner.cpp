#include <CentroidalPlanner/CentroidalPlanner.h>

using namespace cpl;

CentroidalPlanner::CentroidalPlanner(std::vector< std::string > contact_names, double robot_mass) :
    _contact_names(contact_names),
    _robot_mass(robot_mass)
{
    
    _simple_problem = std::make_shared<solver::SimpleProblem> (_contact_names, _robot_mass);

}
