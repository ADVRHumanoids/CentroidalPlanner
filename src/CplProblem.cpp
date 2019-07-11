#include <CentroidalPlanner/Ifopt/CplProblem.h>
#include <iostream>

using namespace cpl::solver;

CplProblem::CplProblem(std::vector< std::string > contact_names, 
                       double robot_mass, 
                       cpl::env::EnvironmentClass::Ptr env) : 
    _contact_names(contact_names),
    _robot_mass(robot_mass),
    _env(env)
{
    
    _ground_fake = std::make_shared<cpl::env::Ground>();
    
    /* Variables */
    _com_var = std::make_shared<Variable3D> ("CoM");

    AddVariableSet(_com_var);
    
    for(auto& elem: _contact_names)
    {
        
         ContactVars _struct;
        
        _struct.force_var = std::make_shared<Variable3D> ("F_" + elem);
        _struct.position_var = std::make_shared<Variable3D> ("p_" + elem);
        _struct.normal_var = std::make_shared<Variable3D> ("n_" + elem);
        
        _contact_vars_map[elem] = _struct;
        
        AddVariableSet(_struct.force_var);
        AddVariableSet(_struct.position_var);
        AddVariableSet(_struct.normal_var);
 
    }
    
    /* Constraints */
    _centroidal_statics = std::make_shared<CentroidalStatics>(_contact_vars_map, 
                                                              _com_var);  
    _centroidal_statics->SetMass(_robot_mass);
    AddConstraintSet(_centroidal_statics);
     
    for(auto& elem: _contact_vars_map)
    {
        if(_env)
        {
 
            _env_const = std::make_shared<EnvironmentConstraint> (elem.first, 
                                                                  elem.second, 
                                                                  _env);
            AddConstraintSet(_env_const);
             
            _env_normal = std::make_shared<EnvironmentNormal> (elem.first, 
                                                               elem.second, 
                                                               _env); 
            AddConstraintSet(_env_normal);
            
            _friction_cone = std::make_shared<FrictionCone> (elem.first, 
                                                             elem.second, 
                                                             _env);
            AddConstraintSet(_friction_cone);
             
        }
        else
        {
            
            _friction_cone = std::make_shared<FrictionCone> (elem.first, 
                                                             elem.second, 
                                                             _ground_fake);            
            AddConstraintSet(_friction_cone);
            
        }
        
        _friction_cone_map[elem.first] = _friction_cone;
        
    }
    
    /* Cost */
    _cost = std::make_shared<MinimizeCentroidalVariables>(_contact_vars_map,
                                                          _com_var);   
    AddCostSet(_cost);
      
}


void CplProblem::GetSolution(Solution& sol)
{
    
    sol.com_sol = _com_var->GetValues();
    
    for(auto& elem: _contact_vars_map)
    {
        
        ContactVars _tmp = elem.second;
        _tmp.force_var->GetValues();
        
        ContactValues _struct;
  
        _struct.force_value = _tmp.force_var->GetValues();
        _struct.position_value = _tmp.position_var->GetValues();
        _struct.normal_value = _tmp.normal_var->GetValues();

        sol.contact_values_map[elem.first] = _struct;

    }

}


void CplProblem::SetForceBounds(std::string contact_name, 
                                const Eigen::Vector3d& force_lb, 
                                const Eigen::Vector3d& force_ub)
{   
    _contact_vars_map.at(contact_name).force_var->SetBounds(force_lb, force_ub);
}


void CplProblem::GetForceBounds(std::string contact_name, 
                                Eigen::Vector3d& force_lb, 
                                Eigen::Vector3d& force_ub) const
{   
    auto bounds = _contact_vars_map.at(contact_name).force_var->GetBounds();
    
    for(int i = 0; i < 3; i++)
    {
        force_lb[i] = bounds.at(i).lower_;
        force_ub[i] = bounds.at(i).upper_;
    }
}


void CplProblem::SetPosBounds(std::string contact_name, 
                              const Eigen::Vector3d& pos_lb, 
                              const Eigen::Vector3d& pos_ub)
{   
    _contact_vars_map.at(contact_name).position_var->SetBounds(pos_lb, pos_ub);
}


void CplProblem::GetPosBounds(std::string contact_name, 
                              Eigen::Vector3d& pos_lb, 
                              Eigen::Vector3d& pos_ub) const
{   
    auto bounds = _contact_vars_map.at(contact_name).position_var->GetBounds();
    
    for(int i = 0; i < 3; i++)
    {
        pos_lb[i] = bounds.at(i).lower_;
        pos_ub[i] = bounds.at(i).upper_;
    }
}


void CplProblem::SetNormalBounds(std::string contact_name, 
                                 const Eigen::Vector3d& normal_lb,
                                 const Eigen::Vector3d& normal_ub)
{   
    _contact_vars_map.at(contact_name).normal_var->SetBounds(normal_lb, normal_ub);
}


void CplProblem::GetNormalBounds(std::string contact_name, 
                                 Eigen::Vector3d& normal_lb, 
                                 Eigen::Vector3d& normal_ub) const
{   
    auto bounds = _contact_vars_map.at(contact_name).normal_var->GetBounds();
    
    for(int i = 0; i < 3; i++)
    {
        normal_lb[i] = bounds.at(i).lower_;
        normal_ub[i] = bounds.at(i).upper_;
    }
}


void CplProblem::SetPosRef(std::string contact_name, 
                           const Eigen::Vector3d& pos_ref)
{
    _cost->SetPosRef(contact_name, pos_ref);   
}


void CplProblem::SetCoMRef(const Eigen::Vector3d& com_ref)
{    
    _cost->SetCoMRef(com_ref);
}


void CplProblem::SetCoMWeight(double W_CoM)
{   
    _cost->SetCoMWeight(W_CoM);
}


void CplProblem::SetPosWeight(double W_p)
{   
    _cost->SetPosWeight(W_p);
}


void CplProblem::SetContactPosWeight(std::string contact_name,
                                     double W_p)
{   
    _cost->SetContactPosWeight(contact_name, W_p);
}


void CplProblem::SetForceWeight(double W_F)
{
    _cost->SetForceWeight(W_F);
}


void CplProblem::SetManipulationWrench(const Eigen::VectorXd& wrench_manip)
{
    _centroidal_statics->SetManipulationWrench(wrench_manip);   
}


void CplProblem::SetMu(double mu)
{
    
    if ( _env )
    {
       _env->SetMu(mu);
    }
    else
    {
        _ground_fake->SetMu(mu);
    }

}


void CplProblem::SetForceThreshold(std::string contact_name, 
                                   double F_thr)
{
    _friction_cone_map.at(contact_name)->SetForceThreshold(F_thr);    
}


namespace cpl { namespace solver {
    
    std::ostream& operator<<(std::ostream& os, const Solution& sol)
    {
        os << "CoM: "<< sol.com_sol.transpose() << "\n";
    
        for(auto& elem: sol.contact_values_map)
        {      
            auto _struct = elem.second; 
            os << "F_" + elem.first + ": " << _struct.force_value.transpose() << "\n";       
        }
        
        for(auto& elem: sol.contact_values_map)
        {       
            auto _struct = elem.second;    
            os << "p_" + elem.first + ": " << _struct.position_value.transpose() << "\n";
        }
        
        for(auto& elem: sol.contact_values_map)
        {        
            auto _struct = elem.second;     
            os << "n_" + elem.first + ": " << _struct.normal_value.transpose() << "\n";        
        }
        
        return os;
    }

}  }
