#include<Environment/Environment.h>
#include <Eigen/Geometry>

namespace Environment { namespace environments {

class Superquadric : public EnvironmentClass {
    
public:    
    
    Superquadric(const Eigen::Vector3d& C, const Eigen::Vector3d& R, const Eigen::Vector3d& P);
    
    virtual void getNormalValue(const Eigen::Vector3d& p, Eigen::Vector3d& normal_Value);
    virtual void getNormalJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& normal_Jacobian);
    virtual void getEnvironmentJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& environment_Jacobian);
    virtual void getEnvironmentValue(const Eigen::Vector3d& p, Eigen::Vector3d& environment_Value);
    
};

}
}
