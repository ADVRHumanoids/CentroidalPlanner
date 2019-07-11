#ifndef __UTILS__
#define __UTILS__


#include <ros/ros.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/ros/RosImpl.h>
#include <Eigen/Geometry> 

namespace cpl { namespace utils { 
     
Eigen::Affine3d GetAffineFromNormal(const Eigen::Vector3d& n);
    
class SurfaceReacher
{
    
public:

    typedef std::shared_ptr<SurfaceReacher> Ptr;
    
    SurfaceReacher(std::vector<std::string> contact_names); 
    
    bool ReachSurface(XBot::Cartesian::RosImpl& ci,
                      std::string contact_name, 
                      const Eigen::Vector3d contact_lin_vel, 
                      double F_thr);
    
private:
    
    std::map<std::string, ros::Subscriber> _sub_force_map;
    std::map<std::string, Eigen::Vector6d> _f_est_map;
    
};
    
    
} }


#endif