#include <CentroidalPlanner/CoMPlanner.h>

using namespace cpl;

CoMPlanner::CoMPlanner(std::vector< std::string > contact_names, 
                       double robot_mass) :
    _contact_names(contact_names),
    _robot_mass(robot_mass)
{   
     env::EnvironmentClass::Ptr env = nullptr;
    
     _cpl = std::make_shared<cpl::CentroidalPlanner>(_contact_names,
                                                     _robot_mass, 
                                                     env);
     
     _cpl->SetPosWeight(0.0);
     _cpl->SetForceWeight(0.0);
     
     Eigen::Vector3d normal_init;
     normal_init << 0.0, 0.0, 1.0;
     
     for (auto& elem: contact_names)
     {        
          _cpl->SetContactNormal(elem,
                                 normal_init);        
     }   
}


solver::Solution CoMPlanner::Solve()
{
    return _cpl->Solve();
}


void CoMPlanner::ResetForceBounds(std::string contact_name)
{   
    _cpl->SetForceBounds(contact_name,
                         -1e3*Eigen::Vector3d::Ones(), 
                          1e3*Eigen::Vector3d::Ones());  
}


void CoMPlanner::SetLiftingContact(std::string contact_name)
{    
    _cpl->SetForceBounds(contact_name,
                         Eigen::Vector3d::Zero(), 
                         Eigen::Vector3d::Zero());
    
    _cpl->SetForceThreshold(contact_name, 0.0);      
}


void CoMPlanner::SetContactPosition(std::string contact_name, 
                                    const Eigen::Vector3d& pos_ref)
{
    _cpl->SetPosBounds(contact_name,
                       pos_ref, 
                       pos_ref);
}


Eigen::Vector3d CoMPlanner::GetContactPosition(std::string contact_name) const
{
    Eigen::Vector3d lb, ub;
    _cpl->GetPosBounds(contact_name, lb, ub);

    if(lb != ub)
    {
        throw std::runtime_error("Contact position for '" + contact_name + "' not set");
    }

    return lb;
}


void CoMPlanner::SetContactNormal(std::string contact_name, 
                                  const Eigen::Vector3d& normal_ref)
{
    _cpl->SetContactNormal(contact_name,
                           normal_ref);
}


void CoMPlanner::SetCoMRef(const Eigen::Vector3d& com_ref)
{
    _cpl->SetCoMRef(com_ref);
}


void CoMPlanner::SetMu(double mu)
{
    _cpl->SetMu(mu);
}


void CoMPlanner::SetForceThreshold(std::string contact_name, 
                                   double F_thr)
{
    Eigen::Vector3d force_lb, force_ub;
    _cpl->GetForceBounds(contact_name, force_lb, force_ub);
    
    if ( force_lb != Eigen::Vector3d::Zero() && force_ub != Eigen::Vector3d::Zero())
    {
        _cpl->SetForceThreshold(contact_name, F_thr);
    }
}


void CoMPlanner::SetCoMWeight(double W_CoM)
{
    _cpl->SetCoMWeight(W_CoM);
}


void CoMPlanner::SetPosWeight(double W_p)
{
    _cpl->SetPosWeight(W_p);
}


void CoMPlanner::SetForceWeight(double W_F)
{
    _cpl->SetForceWeight(W_F);
}