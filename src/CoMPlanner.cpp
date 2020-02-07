#include <CentroidalPlanner/CoMPlanner.h>

using namespace cpl;

CoMPlanner::CoMPlanner(std::vector< std::string > contact_names, 
                         double robot_mass): 
                         CentroidalPlanner(contact_names, robot_mass, nullptr),
                         _contact_names(contact_names)
{   
     SetPosWeight(0.0);
     SetForceWeight(0.0);
     
     Eigen::Vector3d normal_init;
     normal_init << 0.0, 0.0, 1.0;
     
     for(auto& elem: contact_names)
     {        
          SetContactNormal(elem,
                           normal_init);  
           
          _F_thr_map[elem] = GetForceThreshold(elem);

     }   
}


void CoMPlanner::SetLiftingContact(std::string contact_name)
{     
    _F_thr_map[contact_name] = GetForceThreshold(contact_name);
     
    SetForceThreshold(contact_name, 
                      0.0);  
    
    SetForceBounds(contact_name,
                   Eigen::Vector3d::Zero(), 
                   Eigen::Vector3d::Zero());       
}


std::vector<std::string> CoMPlanner::GetLiftingContacts() const
{  
    std::vector<std::string> lifting_contacts;
    
    for (auto& elem : _contact_names)
    {     
        Eigen::Vector3d force_lb, force_ub;
        GetForceBounds(elem, force_lb, force_ub);
    
        if ( force_lb == Eigen::Vector3d::Zero() && force_ub == Eigen::Vector3d::Zero())
        {          
            lifting_contacts.push_back(elem);
        }      
    }
    
    return lifting_contacts;
}


bool CoMPlanner::IsLiftingContact(const std::string& contact_name) const
{  
    bool lifting = false;
    auto lifting_contacts = GetLiftingContacts();
    
    for (auto& elem: lifting_contacts)
    {        
        if(elem == contact_name)
            lifting = true;
    }
    
    return lifting;
}


void CoMPlanner::ResetLiftingContact(std::string contact_name)
{
    if (!IsLiftingContact(contact_name))
    {
        throw std::runtime_error("'" + contact_name + "' is not a lifting contact.");
    }
    else
    {    
        SetForceBounds(contact_name,
                       -1e3*Eigen::Vector3d::Ones(), 
                        1e3*Eigen::Vector3d::Ones());  
        
        SetForceThreshold(contact_name, _F_thr_map[contact_name]); 
    }
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
    
//    if ( n_ref.norm() != 1.0)
//    {
//        throw std::invalid_argument("Invalid contact normal");
//    }
    
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


void CoMPlanner::SetMu(double mu)
{
    if (mu <= 0.0)
    {
        throw std::invalid_argument("Invalid friction coefficient");
    }
    
    auto cpl_problem = GetCplProblem();
    cpl_problem->SetMu(mu);
    
}
