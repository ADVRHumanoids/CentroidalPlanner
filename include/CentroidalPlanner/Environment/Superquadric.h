#ifndef __SUPERQUADRIC__
#define __SUPERQUADRIC__


#include <CentroidalPlanner/Environment/Environment.h>
#include <Eigen/Geometry>

namespace cpl { namespace env {

/**
* @brief Superquadric function
*/
class Superquadric : public EnvironmentClass {
    
public:  
    
    typedef std::shared_ptr<Superquadric> Ptr;
      
    Superquadric();
    
    /**
    * @brief Set superquadric parameters. C: center, R: axial radii, P axial curvatures
    */
    void SetParameters(const Eigen::Vector3d& C, const Eigen::Vector3d& R, const Eigen::Vector3d& P);
    
    virtual void GetEnvironmentValue(const Eigen::Vector3d& p, 
                                     double& environment_Value); 
    
    virtual void GetEnvironmentJacobian(const Eigen::Vector3d& p, 
                                        Eigen::Vector3d& environment_Jacobian); 
    
    virtual void GetNormalValue(const Eigen::Vector3d& p, 
                                Eigen::Vector3d& normal_Value); 
    
    virtual void GetNormalJacobian(const Eigen::Vector3d& p, 
                                   Eigen::MatrixXd& normal_Jacobian); 
    

private:

    Eigen::Vector3d _C, _P, _R;    
    
};

} }


#endif
