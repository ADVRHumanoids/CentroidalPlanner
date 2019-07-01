#include <CentroidalPlanner/Environment/Environment.h>
#include <Eigen/Geometry>

namespace cpl { namespace env {

class Ground : public EnvironmentClass {
    
public:    
    
    virtual void getEnvironmentValue(const Eigen::Vector3d& p, double& environment_Value) = 0; 
    virtual void getEnvironmentJacobian(const Eigen::Vector3d& p, Eigen::Vector3d& environment_Jacobian) = 0; 
    virtual void getNormalValue(const Eigen::Vector3d& p, Eigen::Vector3d& normal_Value) = 0; 
    virtual void getNormalJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& normal_Jacobian) = 0;    
    
};

} }
