#include <CentroidalPlanner/Environment/Ground.h>

using namespace cpl::env;

Ground::Ground()
{ 
    _ground_z = 0.0;
}


void Ground::SetGroundZ(const double& ground_z)
{
   _ground_z = ground_z;   
}


double Ground::GetGroundZ() const
{
   return _ground_z;   
}


void Ground::GetEnvironmentValue(const Eigen::Vector3d& p, 
                                 double& environment_Value)
{             
    environment_Value = p.z() - _ground_z;
}


void Ground::GetEnvironmentJacobian(const Eigen::Vector3d& p, 
                                    Eigen::Vector3d& environment_Jacobian)
{  
    environment_Jacobian.setZero();
    environment_Jacobian.z() = 1.0;
}


void Ground::GetNormalValue(const Eigen::Vector3d& p, 
                            Eigen::Vector3d& normal_Value)
{
    normal_Value.setZero();
    normal_Value.z() = 1.0;  
}


void Ground::GetNormalJacobian(const Eigen::Vector3d& p, 
                               Eigen::MatrixXd& normal_Jacobian)
{  
    normal_Jacobian.setZero(3,3);
}