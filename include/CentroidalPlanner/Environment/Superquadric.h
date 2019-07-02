#include <CentroidalPlanner/Environment/Environment.h>
#include <Eigen/Geometry>

namespace cpl { namespace env {

/**
* @brief Superquadric function
*/
class Superquadric : public EnvironmentClass {
    
public:    
    
    Superquadric(const Eigen::Vector3d& C, const Eigen::Vector3d& R, const Eigen::Vector3d& P);
    
    virtual void getEnvironmentValue(const Eigen::Vector3d& p, double& environment_Value) = 0; 
    virtual void getEnvironmentJacobian(const Eigen::Vector3d& p, Eigen::Vector3d& environment_Jacobian) = 0; 
    virtual void getNormalValue(const Eigen::Vector3d& p, Eigen::Vector3d& normal_Value) = 0; 
    virtual void getNormalJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& normal_Jacobian) = 0; 

private:

Eigen::Vector3d _C, _P, _R;    
    
};

} }
