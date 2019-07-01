#include <IFOPT/constraints/centroidal_statics.h>

using namespace ifopt::Constraints;

CentroidalStatics::CentroidalStatics(const std::map<std::string, Contact_Var_Name>& contacts_map):
    ConstraintSet(6, "CentroidalStatics"),
    _contacts_map(contacts_map)
{
  
     _CoM = GetVariables()->GetComponent("CoM")->GetValues();  
     
     _wrench_manip.setZero(6);
     _mg << 0.0, 0.0, -1000;
    
}


void CentroidalStatics::setMass(const double& m)
{
    _mg[2] = -m*9.81;
}

void CentroidalStatics::setManipulationWrench(const Eigen::VectorXd& wrench_manip)
{
     _wrench_manip =  wrench_manip;
}


Eigen::VectorXd CentroidalStatics::GetValues() const 
{
    Eigen::VectorXd value(6);
    value.setZero();
    
    for(auto& elem: _contacts_map)
    {
        std::string key_ = elem.first;
        Contact_Var_Name _struct = elem.second;
        
        Eigen::Vector3d _Fi = GetVariables()->GetComponent(_struct.force_name)->GetValues();      
        Eigen::Vector3d _pi = GetVariables()->GetComponent(_struct.position_name)->GetValues(); 
        
        value.head<3>() += _Fi;
        value.tail<3>() += (_pi-_CoM).cross(_Fi);
    }  
    
    value -= _wrench_manip;
    value.head<3>() += _mg;
  
    return value;
    
}


ifopt::Composite::VecBound CentroidalStatics::GetBounds() const 
{
    VecBound b(GetRows());
    for(int i = 0; i < 6; i++)
    {            
        b.at(i) = Bounds(.0, .0);  
    }
       
    return b;
}

void CentroidalStatics::FillJacobianBlock (std::string var_set, ifopt::Composite::Jacobian& jac_block) const 
{
    
    jac_block.setZero();
    
    for(auto& elem: _contacts_map)
    {
        std::string key_ = elem.first;
        Contact_Var_Name _struct = elem.second;
        
        if(var_set == _struct.force_name)
        {
            
            jac_block.coeffRef(0, 0) = 1.0;
            jac_block.coeffRef(1, 1) = 1.0;
            jac_block.coeffRef(2, 2) = 1.0;
            
            Eigen::Vector3d _pi = GetVariables()->GetComponent(_struct.position_name)->GetValues(); 
            
            jac_block.coeffRef(3, 1) = -(_pi.z()-_CoM.z());
            jac_block.coeffRef(3, 2) =   _pi.y()-_CoM.y();
            jac_block.coeffRef(4, 0) =   _pi.z()-_CoM.z();
            jac_block.coeffRef(4, 2) = -(_pi.x()-_CoM.x());
            jac_block.coeffRef(5, 0) = -(_pi.y()-_CoM.y());
            jac_block.coeffRef(5, 1) =   _pi.x()-_CoM.x();
        }
        
        if(var_set == _struct.position_name)
        {
            
            Eigen::Vector3d _Fi = GetVariables()->GetComponent(_struct.force_name)->GetValues();      
            
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
        for(auto& elem: _contacts_map)
        {
            
            std::string key_ = elem.first;
            Contact_Var_Name _struct = elem.second;
                
            Eigen::Vector3d _Fi = GetVariables()->GetComponent(_struct.force_name)->GetValues(); 
    
            jac_block.coeffRef(3, 1) -=  _Fi.z();
            jac_block.coeffRef(3, 2) -= -_Fi.y();
            jac_block.coeffRef(4, 0) -= -_Fi.z();
            jac_block.coeffRef(4, 2) -=  _Fi.x();
            jac_block.coeffRef(5, 0) -=  _Fi.y();
            jac_block.coeffRef(5, 1) -= -_Fi.x();
    
        }
     }
    
}