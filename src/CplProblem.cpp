#include <CentroidalPlanner/Ifopt/CplProblem.h>
#include <iostream>

using namespace cpl::solver;

CplProblem::CplProblem(std::vector< std::string > contact_names, double robot_mass, cpl::env::EnvironmentClass::Ptr env) : 
    _contact_names(contact_names),
    _robot_mass(robot_mass),
    _env(env)
{
    
    /* Variables */
    _com_var = std::make_shared<Variable3D> ("CoM");

    AddVariableSet(_com_var);
    
    for(auto& elem: _contact_names)
    {
        
         CplSolver::ContactVars _struct;
        
        _struct.force_var = std::make_shared<Variable3D> ("F_" + elem);
        _struct.position_var = std::make_shared<Variable3D> ("p_" + elem);
        _struct.normal_var = std::make_shared<Variable3D> ("n_" + elem);
        
        _contact_vars_map[elem] = _struct;
        
        AddVariableSet(_struct.force_var);
        AddVariableSet(_struct.position_var);
        AddVariableSet(_struct.normal_var);
 
    }
    
    /* Constraints */
    _centroidal_statics = std::make_shared<CentroidalStatics>(_contact_vars_map, _com_var);  
    _centroidal_statics->SetMass(_robot_mass);
    AddConstraintSet(_centroidal_statics);
     
    for(auto& elem: _contact_vars_map)
    {
        
        _env_const = std::make_shared<EnvironmentConstraint> (elem.first, elem.second, _env);
        _env_normal = std::make_shared<EnvironmentNormal> (elem.first, elem.second, _env);
        _friction_cone = std::make_shared<FrictionCone> (elem.first, elem.second, _env);
        
        AddConstraintSet(_env_const);
        AddConstraintSet(_env_normal);
        AddConstraintSet(_friction_cone);
        
    }
    
    /* Cost */
    _cost = std::make_shared<MinimizeCentroidalVariables>(_contact_vars_map, _com_var);
    Eigen::Vector3d com_ref;
    com_ref << 0.0, 0.0 , 1.0;
    _cost->SetCoMRef(com_ref);
    
    AddCostSet(_cost);
      
}


void CplProblem::GetSolution(Solution& sol)
{
    
    sol.com_sol = _com_var->GetValues();
    
    for(auto& elem: _contact_vars_map)
    {
        
        CplSolver::ContactVars _tmp = elem.second;
        _tmp.force_var->GetValues();
        
        ContactVarsSol _struct;
  
        _struct.force_sol = _tmp.force_var->GetValues();
        _struct.position_sol = _tmp.position_var->GetValues();
        _struct.normal_sol = _tmp.normal_var->GetValues();

        sol.contact_vars_sol_map[elem.first] = _struct;

    }

}

namespace cpl { namespace solver {
    
    std::ostream& operator<<(std::ostream& os, const CplProblem::Solution& sol)
    {
        os << "CoM: "<< sol.com_sol.transpose() << "\n";
    
        for(auto& elem: sol.contact_vars_sol_map)
        {      
            auto _struct = elem.second; 
            os << "F_" + elem.first + ": " << _struct.force_sol.transpose() << "\n";       
        }
        
        for(auto& elem: sol.contact_vars_sol_map)
        {       
            auto _struct = elem.second;    
            os << "p_" + elem.first + ": " << _struct.position_sol.transpose() << "\n";
        }
        
        for(auto& elem: sol.contact_vars_sol_map)
        {        
            auto _struct = elem.second;     
            os << "n_" + elem.first + ": " << _struct.normal_sol.transpose() << "\n";        
        }
        
        return os;
    }

}  }