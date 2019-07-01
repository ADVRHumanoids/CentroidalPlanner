#include <Eigen/Geometry>
#include <memory>

namespace Environment {

class EnvironmentClass {

    
public:
    
    typedef std::shared_ptr<EnvironmentClass> Ptr;

    virtual void getNormalValue(const Eigen::Vector3d& p, Eigen::Vector3d& normal_Value) = 0; 
    virtual void getNormalJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& normal_Jacobian) = 0; 
    virtual void getEnvironmentValue(const Eigen::Vector3d& p, Eigen::Vector3d& environment_Value) = 0; 
    virtual void getEnvironmentJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& environment_Jacobian) = 0; 
    
    virtual ~EnvironmentClass() = default; // polymorphic classes must have virtual destructor (good practice)
    
};

}
