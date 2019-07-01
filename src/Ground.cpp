#include <CentroidalPlanner/Environment/Ground.h>

using namespace cpl::env;

void Ground::getEnvironmentValue(const Eigen::Vector3d& p, double& environment_Value)
{
              
    environment_Value = p.z();

}

void Ground::getEnvironmentJacobian(const Eigen::Vector3d& p, Eigen::Vector3d& environment_Jacobian)
{
    
    environment_Jacobian.setZero();
    environment_Jacobian.z() = 1.0;

}


void Ground::getNormalValue(const Eigen::Vector3d& p, Eigen::Vector3d& normal_Value)
{
    
    normal_Value.setZero();
    normal_Value.z() = 1.0;  

}


void Ground::getNormalJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& normal_Jacobian)
{
    
    normal_Jacobian.setZero(3,3);

}