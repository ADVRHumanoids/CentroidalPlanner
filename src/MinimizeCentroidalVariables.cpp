#include <CentroidalPlanner/Ifopt/MinimizeCentroidalVariables.h>

using namespace cpl::solver;

MinimizeCentroidalVariables::MinimizeCentroidalVariables(std::map<std::string, CplSolver::ContactVars> contact_vars_map, Variable3D::Ptr com_var):
      CostTerm("Minimize centroidal variables"),
      _contact_vars_map(contact_vars_map),
      _com_var(com_var)
{
    
  _CoM_ref.setZero();
  
  _W_p   = 1.0;   
  _W_CoM = 1.0;
  _W_F   = 1.0;    
  
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
    
    Eigen::Vector3d CoM = _com_var->GetValues();
   
    for(auto& elem: _contact_vars_map)
    {
        std::string key_ = elem.first;
        CplSolver::ContactVars _struct = elem.second;
        
        Eigen::Vector3d _Fi = _struct.force_var->GetValues();     
        Eigen::Vector3d _pi = _struct.position_var->GetValues();
        
        value += 0.5*_W_p*(_pi).squaredNorm() + 0.5*_W_F*_Fi.squaredNorm();
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
        std::string key_ = elem.first;
        CplSolver::ContactVars _struct = elem.second;
        
        if(var_set == "F_" + key_)
        {
            Eigen::Vector3d _Fi = _struct.force_var->GetValues(); 
            
            jac.coeffRef(0, 0) = _W_F * _Fi.x();
            jac.coeffRef(0, 1) = _W_F * _Fi.y();
            jac.coeffRef(0, 2) = _W_F * _Fi.z();
        }
        
        if(var_set == "p_" + key_)
        {
            Eigen::Vector3d _pi = _struct.position_var->GetValues(); 
            
            jac.coeffRef(0, 0) = _W_p * (_pi.x());
            jac.coeffRef(0, 1) = _W_p * (_pi.y());
            jac.coeffRef(0, 2) = _W_p * (_pi.z());
            
        }
    }  
    
        
    if(var_set == "CoM")
    { 
        jac.coeffRef(0, 0) = _W_CoM * (CoM.x() - _CoM_ref.x());
        jac.coeffRef(0, 1) = _W_CoM * (CoM.y() - _CoM_ref.y());
        jac.coeffRef(0, 2) = _W_CoM * (CoM.z() - _CoM_ref.z());
    }       
   
}