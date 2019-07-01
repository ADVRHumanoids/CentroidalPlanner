#include <CentroidalPlanner/Environment/Superquadric.h>

using namespace cpl::env;

void Superquadric::getEnvironmentValue(const Eigen::Vector3d& p, double& environment_Value)
{
    
    for(int i = 0; i < 3; i++)
    {            
        environment_Value += pow((p(i)-_C(i))/_R(i),_P(i));
    }

}

void Superquadric::getEnvironmentJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& environment_Jacobian)
{

}


void Superquadric::getNormalValue(const Eigen::Vector3d& p, Eigen::Vector3d& normal_Value)
{
    
    normal_Value.x() = -_P.x()/pow(_R.x(),_P.x()) * pow(p.x()-_C.x(),_P.x()-1);
    normal_Value.y() = -_P.y()/pow(_R.y(),_P.y()) * pow(p.y()-_C.y(),_P.y()-1);
    normal_Value.z() = -_P.z()/pow(_R.z(),_P.z()) * pow(p.z()-_C.z(),_P.z()-1);  

}


void Superquadric::getNormalJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& normal_Jacobian)
{

}

