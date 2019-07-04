#include <CentroidalPlanner/Ifopt/Constraints/CentroidalStatics.h>

using namespace cpl::solver;

CentroidalStatics::CentroidalStatics(std::map<std::string, ContactVars> contact_vars_map, Variable3D::Ptr com_var):
    ConstraintSet(6, "Centroidal statics constraint"),
    _contact_vars_map(contact_vars_map),
    _com_var(com_var)
{
     
     _wrench_manip.setZero(6);
     
     _m = 100.0;
     _g << 0.0, 0.0, -9.81;
    
}


void CentroidalStatics::SetMass(const double& m)
{
    _m = m;
}

void CentroidalStatics::SetManipulationWrench(const Eigen::VectorXd& wrench_manip)
{
     _wrench_manip =  wrench_manip;
}


Eigen::VectorXd CentroidalStatics::GetValues() const 
{
    Eigen::VectorXd value(6);
    value.setZero();
    
    Eigen::Vector3d CoM = _com_var->GetValues();
    
    for(auto& elem: _contact_vars_map)
    {

        ContactVars _struct = elem.second;
        
        Eigen::Vector3d _Fi = _struct.force_var->GetValues();   
        Eigen::Vector3d _pi = _struct.position_var->GetValues(); 
        
        value.head<3>() += _Fi;
        value.tail<3>() += (_pi-CoM).cross(_Fi);
    }  
    
    value -= _wrench_manip;
    value.head<3>() += _m*_g;
  
    return value;
    
}


ifopt::Composite::VecBound CentroidalStatics::GetBounds() const 
{
    VecBound b(GetRows());
    for(int i = 0; i < 6; i++)
    {            
        b.at(i) = ifopt::Bounds(.0, .0);  
    }
       
    return b;
}

void CentroidalStatics::FillJacobianBlock (std::string var_set, ifopt::Composite::Jacobian& jac_block) const 
{
    
    jac_block.setZero();
    
    Eigen::Vector3d CoM = _com_var->GetValues();
      
    for(auto& elem: _contact_vars_map)
    {

        ContactVars _struct = elem.second;
        Eigen::Vector3d _pi = _struct.position_var->GetValues();
        Eigen::Vector3d _Fi = _struct.force_var->GetValues();
        
        if(var_set == "F_" + elem.first)
        {
            
            jac_block.coeffRef(0, 0) = 1.0;
            jac_block.coeffRef(1, 1) = 1.0;
            jac_block.coeffRef(2, 2) = 1.0;       
            jac_block.coeffRef(3, 1) = -(_pi.z() - CoM.z());
            jac_block.coeffRef(3, 2) =   _pi.y() - CoM.y();
            jac_block.coeffRef(4, 0) =   _pi.z() - CoM.z();
            jac_block.coeffRef(4, 2) = -(_pi.x() - CoM.x());
            jac_block.coeffRef(5, 0) = -(_pi.y() - CoM.y());
            jac_block.coeffRef(5, 1) =   _pi.x() - CoM.x();
            
        }
        
        if(var_set == "p_" + elem.first)
        {
            
            jac_block.coeffRef(3, 1) =  _Fi.z();
            jac_block.coeffRef(3, 2) = -_Fi.y();
            jac_block.coeffRef(4, 0) = -_Fi.z();
            jac_block.coeffRef(4, 2) =  _Fi.x();
            jac_block.coeffRef(5, 0) =  _Fi.y();
            jac_block.coeffRef(5, 1) = -_Fi.x();
            
        }   
    }  
    

    if(var_set == "CoM")
    { 
        for(auto& elem: _contact_vars_map)
        {
           
            ContactVars _struct = elem.second;
                
            Eigen::Vector3d _Fi = _struct.force_var->GetValues(); 
    
            jac_block.coeffRef(3, 1) -=  _Fi.z();
            jac_block.coeffRef(3, 2) -= -_Fi.y();
            jac_block.coeffRef(4, 0) -= -_Fi.z();
            jac_block.coeffRef(4, 2) -=  _Fi.x();
            jac_block.coeffRef(5, 0) -=  _Fi.y();
            jac_block.coeffRef(5, 1) -= -_Fi.x();
    
        }
     }
    
}