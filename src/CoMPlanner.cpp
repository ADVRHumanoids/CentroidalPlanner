#include <CentroidalPlanner/CoMPlanner.h>

using namespace cpl;

CoMPlanner::CoMPlanner(std::vector< std::string > contact_names, 
                         double robot_mass): 
                         CentroidalPlanner(contact_names, robot_mass, nullptr)
{
     SetPosWeight(0.0);
     SetForceWeight(0.0);
     
     Eigen::Vector3d normal_init;
     normal_init << 0.0, 0.0, 1.0;
     
     for(auto& elem: contact_names)
     {        
          SetContactNormal(elem,
                           normal_init);        
     }   
}


void CoMPlanner::SetLiftingContact(std::string contact_name)
{    
    SetForceBounds(contact_name,
                   Eigen::Vector3d::Zero(), 
                   Eigen::Vector3d::Zero());
    
    SetForceThreshold(contact_name, 
                      0.0);      
}


void CoMPlanner::SetContactPosition(std::string contact_name, 
                                    const Eigen::Vector3d& pos_ref)
{
    SetPosBounds(contact_name,
                 pos_ref, 
                 pos_ref);
}


Eigen::Vector3d CoMPlanner::GetContactPosition(std::string contact_name) const
{
    Eigen::Vector3d lb, ub;
    GetPosBounds(contact_name, lb, ub);

    if(lb != ub)
    {
        throw std::runtime_error("Contact position for '" + contact_name + "' not set");
    }

    return lb;
}


void CoMPlanner::SetContactNormal(std::string contact_name, const Eigen::Vector3d& n_ref)
{
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    if ( n_ref.norm() != 1.0)
    {
        throw std::invalid_argument("Invalid contact normal");
    }
    
    SetNormalBounds(contact_name, 
                    n_ref, 
                    n_ref);
}


Eigen::Vector3d CoMPlanner::GetContactNormal(std::string contact_name) const
{
    Eigen::Vector3d lb, ub;
    GetNormalBounds(contact_name, lb, ub);

    if(lb != ub)
    {
        throw std::runtime_error("Contact normal for '" + contact_name + "' not set");
    }

    return lb;
}