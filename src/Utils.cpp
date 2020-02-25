#include <CentroidalPlanner/Utils.h>

Eigen::Affine3d cpl::utils::GetAffineFromNormal(const Eigen::Vector3d& n)
{
    Eigen::Matrix3d R;
    R.setZero();

    R.coeffRef(0, 0) =  n.y() / ((n.head(2)).norm());
    R.coeffRef(0, 1) = -n.x() / ((n.head(2)).norm());

    R.coeffRef(1, 0) = (n.x() * n.z()) / ((n.head(2)).norm());
    R.coeffRef(1, 1) = (n.y() * n.z()) / ((n.head(2)).norm());
    R.coeffRef(1, 2) = - (n.head(2)).norm();

    R.coeffRef(2, 0) = n.x();
    R.coeffRef(2, 1) = n.y();
    R.coeffRef(2, 2) = n.z();

    Eigen::Affine3d pose;
    pose.translation() = n;
    pose.linear() =  R.transpose();
    
    return pose;
}


std::map<std::string, Eigen::Vector6d> * g_fmap_ptr;

void on_force_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_ptr->at(l));
}


cpl::utils::SurfaceReacher::SurfaceReacher(std::vector<std::string> contact_names)                                    
{   
    for(auto& elem: contact_names)
    {
        auto sub_force = ros::NodeHandle("cartesian").subscribe<geometry_msgs::WrenchStamped>("force_estimation/" + elem, 1, boost::bind(on_force_recv, _1, elem));   
        _sub_force_map[elem] = sub_force;
        _f_est_map[elem] = Eigen::Vector6d::Zero();
    
        ROS_INFO("Subscribed to topic '%s'", sub_force.getTopic().c_str());
    }
    
    g_fmap_ptr = &_f_est_map; 
}


bool cpl::utils::SurfaceReacher::ReachSurface(XBot::Cartesian::RosImpl& ci,
                                              std::string contact_name, 
                                              const Eigen::Vector3d contact_lin_vel, 
                                              double F_thr)
{    
    if (F_thr < 0.0)
    {
        throw std::invalid_argument("Invalid force threshold");
    }
    
    ros::Rate rate(100.0);
    
    bool surface_reached = false;
    
    Eigen::Vector3d f_est = _f_est_map[contact_name].head(3);
    
    Eigen::Vector6d contact_twist;
    contact_twist.setZero();
    contact_twist.head(3) = contact_lin_vel;
    
    ci.setControlMode(contact_name, XBot::Cartesian::ControlType::Velocity);
    
    while ( !surface_reached )
    {
        ros::spinOnce();
        
        if( f_est.dot(-contact_lin_vel) >= F_thr )
            surface_reached = true;
        
        ci.setVelocityReference(contact_name, contact_twist);          
    }
    
    ci.setControlMode(contact_name, XBot::Cartesian::ControlType::Position); 
  
    return surface_reached;
}