#include <CentroidalPlanner/Ifopt/MinimizeCentroidalVariables.h>

using namespace cpl::solver;

MinimizeCentroidalVariables::MinimizeCentroidalVariables(std::map<std::string, ContactVars> contact_vars_map, 
                                                         Variable3D::Ptr com_var):
      CostTerm("Minimize centroidal variables"),
      _contact_vars_map(contact_vars_map),
      _com_var(com_var)
{   
  _CoM_ref << 0.0, 0.0, 1.0;
    
  _W_CoM = 1.0;  
  
    for(auto& elem: _contact_vars_map)
    {
        ContactValues _struct;
        
        _struct.force_value.setZero();
        _struct.position_value.setZero();
        _struct.normal_value.setZero();
        
        _contact_vars_ref_map[elem.first] = _struct;
        _pos_weight_map[elem.first] = 1.0; 
        _force_weight_map[elem.first] = 1.0;
    } 
}


void MinimizeCentroidalVariables::SetPosRef(std::string contact_name, 
                                            const Eigen::Vector3d& pos_ref)
{
    _contact_vars_ref_map.at(contact_name).position_value = pos_ref;  
}


Eigen::Vector3d MinimizeCentroidalVariables::GetPosRef(std::string contact_name) const
{
    return _contact_vars_ref_map.at(contact_name).position_value;
}


void MinimizeCentroidalVariables::SetForceRef(std::string contact_name, 
                                              const Eigen::Vector3d& force_ref)
{
    _contact_vars_ref_map.at(contact_name).force_value = force_ref;  
}


Eigen::Vector3d MinimizeCentroidalVariables::GetForceRef(std::string contact_name) const
{
    return _contact_vars_ref_map.at(contact_name).force_value;
}


void MinimizeCentroidalVariables::SetCoMRef(const Eigen::Vector3d& CoM_ref)
{    
    _CoM_ref = CoM_ref;
}


Eigen::Vector3d MinimizeCentroidalVariables::GetCoMRef() const
{
    return _CoM_ref;
}


void MinimizeCentroidalVariables::SetCoMWeight(double W_CoM)
{    
    _W_CoM = W_CoM;
}


double MinimizeCentroidalVariables::GetCoMWeight() const
{
    return _W_CoM;
}


void MinimizeCentroidalVariables::SetPosWeight(double W_p)
{
    for(auto& elem: _contact_vars_map)
    {
         _pos_weight_map[elem.first] = W_p;
    }
}


void MinimizeCentroidalVariables::SetContactPosWeight(std::string contact_name, 
                                                      double W_p)
{
    _pos_weight_map.at(contact_name) = W_p;
}


double MinimizeCentroidalVariables::GetContactPosWeight(std::string contact_name) const
{
    return _pos_weight_map.at(contact_name);
}


void MinimizeCentroidalVariables::SetForceWeight(double W_F)
{
    for(auto& elem: _contact_vars_map)
    {
        _force_weight_map[elem.first] = W_F;
    }
}


void MinimizeCentroidalVariables::SetContactForceWeight(std::string contact_name, 
                                                        double W_F)
{
    _force_weight_map.at(contact_name) = W_F;
}


double MinimizeCentroidalVariables::GetContactForceWeight(std::string contact_name) const
{
    return _force_weight_map.at(contact_name);
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
        Eigen::Vector3d _Fi_ref = _contact_vars_ref_map.at(elem.first).force_value;
        double W_p_i = _pos_weight_map.at(elem.first);
        double W_F_i = _force_weight_map.at(elem.first);
        
        value += 0.5*W_p_i*(_pi - _pi_ref).squaredNorm() + 0.5*W_F_i*(_Fi - _Fi_ref).squaredNorm();
    }  
        
    value += 0.5*_W_CoM*(CoM - _CoM_ref).squaredNorm();
            
    return value; 
};


void MinimizeCentroidalVariables::FillJacobianBlock (std::string var_set, 
                                                     Jacobian& jac) const 
{
    jac.setZero();
    
    Eigen::Vector3d CoM = _com_var->GetValues();
       
    for(auto& elem: _contact_vars_map)
    {

        ContactVars _struct = elem.second;
        
        if(var_set == "F_" + elem.first)
        {
            Eigen::Vector3d _Fi = _struct.force_var->GetValues(); 
            Eigen::Vector3d _Fi_ref = _contact_vars_ref_map.at(elem.first).force_value;
            double W_F_i = _force_weight_map.at(elem.first);
            
            jac.coeffRef(0, 0) = W_F_i * (_Fi.x() - _Fi_ref.x());
            jac.coeffRef(0, 1) = W_F_i * (_Fi.y() - _Fi_ref.y());
            jac.coeffRef(0, 2) = W_F_i * (_Fi.z() - _Fi_ref.z());
        }
        
        if(var_set == "p_" + elem.first)
        {
            Eigen::Vector3d _pi = _struct.position_var->GetValues(); 
            Eigen::Vector3d _pi_ref = _contact_vars_ref_map.at(elem.first).position_value;
            double W_p_i = _pos_weight_map.at(elem.first);
            
            jac.coeffRef(0, 0) = W_p_i * (_pi.x() - _pi_ref.x());
            jac.coeffRef(0, 1) = W_p_i * (_pi.y() - _pi_ref.y());
            jac.coeffRef(0, 2) = W_p_i * (_pi.z() - _pi_ref.z());
            
        }
    }  
        
    if(var_set == "CoM")
    { 
        jac.coeffRef(0, 0) = _W_CoM * (CoM.x() - _CoM_ref.x());
        jac.coeffRef(0, 1) = _W_CoM * (CoM.y() - _CoM_ref.y());
        jac.coeffRef(0, 2) = _W_CoM * (CoM.z() - _CoM_ref.z());
    }        
}