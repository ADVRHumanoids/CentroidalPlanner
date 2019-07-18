#ifndef __GROUND__
#define __GROUND__


#include <CentroidalPlanner/Environment/Environment.h>
#include <Eigen/Geometry>

namespace cpl { namespace env {

/**
* @brief Environment modelled as the (x,y) plane with settable ground level.
*/
class Ground : public EnvironmentClass {
    
public:   
    
    typedef std::shared_ptr<Ground> Ptr;
    
    Ground();
    
    void SetGroundZ(const double& ground_z);  
    
    double GetGroundZ() const;
    
    virtual void GetEnvironmentValue(const Eigen::Vector3d& p, 
                                     double& environment_Value); 
    
    virtual void GetEnvironmentJacobian(const Eigen::Vector3d& p, 
                                        Eigen::Vector3d& environment_Jacobian); 
    
    virtual void GetNormalValue(const Eigen::Vector3d& p, 
                                Eigen::Vector3d& normal_Value); 
    
    virtual void GetNormalJacobian(const Eigen::Vector3d& p, 
                                   Eigen::MatrixXd& normal_Jacobian);   
      
private:

    double _ground_z; 
};

} }


#endif
