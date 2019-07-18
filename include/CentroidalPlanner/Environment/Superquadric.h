#ifndef __SUPERQUADRIC__
#define __SUPERQUADRIC__


#include <CentroidalPlanner/Environment/Environment.h>
#include <Eigen/Geometry>

namespace cpl { namespace env {

/**
* @brief Environment modelled as a superquadric function with settable parameters.
*/
class Superquadric : public EnvironmentClass {
    
public:  
    
    typedef std::shared_ptr<Superquadric> Ptr;
      
    Superquadric();
    
    /**
    * @brief Set superquadric parameters. 
    * @param C: center.
    * @param R: axial radii.
    * @param P: axial curvatures.
    * @throw exception if axial radii <= 0.0
    * @throw exception if axial curvatures < 2.0
    */
    void SetParameters(const Eigen::Vector3d& C, const Eigen::Vector3d& R, const Eigen::Vector3d& P);

    void GetParameters(Eigen::Vector3d& C, Eigen::Vector3d& R, Eigen::Vector3d& P);
    
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
