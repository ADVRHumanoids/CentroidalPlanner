#include <CentroidalPlanner/Ifopt/MinimizeCentroidalVariables.h>

using namespace cpl::solver;

MinimizeCentroidalVariables::MinimizeCentroidalVariables(std::map<std::string, ContactVars> contact_vars_map, Variable3D::Ptr com_var):
      CostTerm("Minimize centroidal variables"),
      _contact_vars_map(contact_vars_map),
      _com_var(com_var)
{
    
  _CoM_ref << 0.0, 0.0, 1.0;
  
  _W_p   = 1.0;   
  _W_CoM = 1.0;
  _W_F   = 1.0;   
  
    for(auto& elem: _contact_vars_map)
    {
        ContactValues _struct;
        
        _struct.force_value.setZero();
        _struct.position_value.setZero();
        _struct.normal_value.setZero();
        
        _contact_vars_ref_map[elem.first] = _struct;
        
    }
  
}


void MinimizeCentroidalVariables::SetPosRef(std::string contact_name, const Eigen::Vector3d& pos_ref)
{
    
    _contact_vars_ref_map.at(contact_name).position_value = pos_ref;
    
}


void MinimizeCentroidalVariables::SetCoMRef(const Eigen::Vector3d& CoM_ref)
{    
    _CoM_ref = CoM_ref;
}

void MinimizeCentroidalVariables::SetCoMWeight(double W_CoM)
{    
    _W_CoM = W_CoM;
}


void MinimizeCentroidalVariables::SetPosWeight(double W_p)
{
    _W_p = W_p;
}


void MinimizeCentroidalVariables::SetForceWeight(double W_F)
{
    _W_F = W_F;
}


double MinimizeCentroidalVariables::GetCost() const 
{    
    double value = 0;
    
    Eigen::Vector3d CoM = _com_var->GetValues();
   
    for(auto& elem: _contact_vars_map)
    {
        
        ContactVars _struct = elem.second;
        
        Eigen::Vector3d _Fi = _struct.force_var->GetValues();     
        Eigen::Vector3d _pi = _struct.position_var->GetValues();
        Eigen::Vector3d _pi_ref = _contact_vars_ref_map.at(elem.first).position_value;
        
        value += 0.5*_W_p*(_pi - _pi_ref).squaredNorm() + 0.5*_W_F*_Fi.squaredNorm();
    }  
        
    value += 0.5*_W_CoM*(CoM - _CoM_ref).squaredNorm();
            
    return value;
    
};

void MinimizeCentroidalVariables::FillJacobianBlock (std::string var_set, Jacobian& jac) const 
{
    jac.setZero();
    
    Eigen::Vector3d CoM = _com_var->GetValues();
       
    for(auto& elem: _contact_vars_map)
    {

        ContactVars _struct = elem.second;
        
        if(var_set == "F_" + elem.first)
        {
            Eigen::Vector3d _Fi = _struct.force_var->GetValues(); 
            
            jac.coeffRef(0, 0) = _W_F * _Fi.x();
            jac.coeffRef(0, 1) = _W_F * _Fi.y();
            jac.coeffRef(0, 2) = _W_F * _Fi.z();
        }
        
        if(var_set == "p_" + elem.first)
        {
            Eigen::Vector3d _pi = _struct.position_var->GetValues(); 
            Eigen::Vector3d _pi_ref = _contact_vars_ref_map.at(elem.first).position_value;
            
            jac.coeffRef(0, 0) = _W_p * (_pi.x() - _pi_ref.x());
            jac.coeffRef(0, 1) = _W_p * (_pi.y() - _pi_ref.y());
            jac.coeffRef(0, 2) = _W_p * (_pi.z() - _pi_ref.z());
            
        }
    }  
    
        
    if(var_set == "CoM")
    { 
        jac.coeffRef(0, 0) = _W_CoM * (CoM.x() - _CoM_ref.x());
        jac.coeffRef(0, 1) = _W_CoM * (CoM.y() - _CoM_ref.y());
        jac.coeffRef(0, 2) = _W_CoM * (CoM.z() - _CoM_ref.z());
    }       
   
}