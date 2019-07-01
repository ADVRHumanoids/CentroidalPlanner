#include<IFOPT/minimize_centroidal_variables.h>

using namespace ifopt;

MinimizeCentroidalVariables::MinimizeCentroidalVariables(const std::map<std::string, Contact_Var_Name>& contacts_map):
      CostTerm("minimize_variables_cost"),
      _contacts_map(contacts_map)
{
    
  _CoM = GetVariables()->GetComponent("CoM")->GetValues();  
  _CoM_ref.setZero();
  
  _W_p = 0.0;   
  _W_CoM = 0.0;
  _W_F = 1.0;    
}

void MinimizeCentroidalVariables::SetCoMRef(const Eigen::Vector3d& CoM_ref)
{    
    _CoM_ref = CoM_ref;
}

void MinimizeCentroidalVariables::SetCoMWeight(const double& W_CoM)
{    
    _W_CoM = W_CoM;
}

void MinimizeCentroidalVariables::SetPositionWeight(const double& W_p)
{
    _W_p = W_p;
}

void MinimizeCentroidalVariables::SetForceWeight(const double& W_F)
{
    _W_F = W_F;
}

double MinimizeCentroidalVariables::GetCost() const 
{    
    double value = 0;
   
    for(auto& elem: _contacts_map)
    {
        std::string key_ = elem.first;
        Contact_Var_Name _struct = elem.second;
        
        Eigen::Vector3d _Fi = GetVariables()->GetComponent(_struct.force_name)->GetValues();      
        Eigen::Vector3d _pi = GetVariables()->GetComponent(_struct.position_name)->GetValues(); 
        
        value += 0.5*_W_p*(_pi).squaredNorm() + 0.5*_W_F*_Fi.squaredNorm();
    }  
        
    value += 0.5*_W_CoM*(_CoM - _CoM_ref).squaredNorm();
            
    return value;
    
};

void MinimizeCentroidalVariables::FillJacobianBlock (std::string var_set, Jacobian& jac) const 
{
    jac.setZero();
    
    
    for(auto& elem: _contacts_map)
    {
        std::string key_ = elem.first;
        Contact_Var_Name _struct = elem.second;
        
        if(var_set == _struct.force_name)
        {
            Eigen::Vector3d _Fi = GetVariables()->GetComponent(_struct.force_name)->GetValues();
            
            jac.coeffRef(0, 0) = _W_F*_Fi.x();
            jac.coeffRef(0, 1) = _W_F*_Fi.y();
            jac.coeffRef(0, 2) = _W_F*_Fi.z();
        }
        
        if(var_set == _struct.position_name)
        {
            Eigen::Vector3d _pi = GetVariables()->GetComponent(_struct.position_name)->GetValues();
            
            jac.coeffRef(0, 0) = _W_p*(_pi.x());
            jac.coeffRef(0, 1) = _W_p*(_pi.y());
            jac.coeffRef(0, 2) = _W_p*(_pi.z());
            
        }
    }  
    
        
    if(var_set == "CoM")
    { 
        jac.coeffRef(0, 0) = _W_CoM*(_CoM.x() - _CoM_ref.x());
        jac.coeffRef(0, 1) = _W_CoM*(_CoM.y() - _CoM_ref.y());
        jac.coeffRef(0, 2) = _W_CoM*(_CoM.z() - _CoM_ref.z());
    }       
   
}