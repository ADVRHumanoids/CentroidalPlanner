#include <CentroidalPlanner/Ifopt/SimpleProblem.h>

using namespace cpl::solver;

SimpleProblem::SimpleProblem(std::vector< std::string > contact_names, double robot_mass) : 
    _contact_names(contact_names),
    _robot_mass(robot_mass)
{
    
    _com_var = std::make_shared<Variable3D> ("CoM");

    _nlp.AddVariableSet(_com_var);
    
    for(auto& elem: _contact_names)
    {
        
         CPLSolver::ContactVars _struct;
        
        _struct.force_var = std::make_shared<Variable3D> ("F_" + elem);
        _struct.position_var = std::make_shared<Variable3D> ("p_" + elem);
        _struct.normal_var = std::make_shared<Variable3D> ("n_" + elem);
        
        _contact_vars_map[elem] = _struct;
        
        _nlp.AddVariableSet(_struct.force_var);
        _nlp.AddVariableSet(_struct.position_var);
        _nlp.AddVariableSet(_struct.normal_var);
 
    }
    
    auto static_constr = std::make_shared<CentroidalStatics>(_contact_vars_map);    

}
