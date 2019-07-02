#include <Eigen/Geometry>
#include <memory>

namespace cpl { namespace env {

/**
* @brief Environment class
*/
class EnvironmentClass {
    
protected:

    double _mu = 1.0;
   
public:
    
    typedef std::shared_ptr<EnvironmentClass> Ptr;
    
    void SetMu(const double& mu) { _mu = mu; }; 
    const double GetMu() { return _mu; }; 

    virtual void getEnvironmentValue(const Eigen::Vector3d& p, double& environment_Value) = 0; 
    virtual void getEnvironmentJacobian(const Eigen::Vector3d& p, Eigen::Vector3d& environment_Jacobian) = 0; 
    virtual void getNormalValue(const Eigen::Vector3d& p, Eigen::Vector3d& normal_Value) = 0; 
    virtual void getNormalJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& normal_Jacobian) = 0; 
    
    virtual ~EnvironmentClass() = default; // polymorphic classes must have virtual destructor (good practice)
    
    
};

} }

