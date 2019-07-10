#include <CentroidalPlanner/CoMPlanner.h>

using namespace cpl;

CoMPlanner::CoMPlanner(std::vector< std::string > contact_names, 
                       double robot_mass) :
    _contact_names(contact_names),
    _robot_mass(robot_mass)
{
    
     env::EnvironmentClass::Ptr env = nullptr;
    
     _cpl_problem = std::make_shared<cpl::CentroidalPlanner>(_contact_names, 
                                                             _robot_mass, 
                                                             env);
     
     _cpl_problem->SetPosWeight(0.0);
     _cpl_problem->SetForceWeight(0.0);  
     
     Eigen::Vector3d normal_init;
     normal_init << 0.0, 0.0, 1.0;
     
     for (auto& elem: contact_names)
     {        
          _cpl_problem->SetContactNormal(elem, 
                                         normal_init);        
     }
    
}


solver::Solution CoMPlanner::Solve()
{
    return _cpl_problem->Solve();
}


void CoMPlanner::ResetForceBounds(std::string contact_name)
{   
    _cpl_problem->SetForceBounds(contact_name, 
                                 -1e3*Eigen::Vector3d::Ones(), 
                                  1e3*Eigen::Vector3d::Ones());  
}


void CoMPlanner::SetLiftingContact(std::string contact_name)
{    
    _cpl_problem->SetForceBounds(contact_name, 
                                 Eigen::Vector3d::Zero(), 
                                 Eigen::Vector3d::Zero());
    
    _cpl_problem->SetForceThreshold(contact_name, 
                                    0.0);      
}


void CoMPlanner::SetContactPosition(std::string contact_name, 
                                    const Eigen::Vector3d& pos_ref)
{
    _cpl_problem->SetPosBounds(contact_name, 
                               pos_ref, 
                               pos_ref);
}


void CoMPlanner::SetContactNormal(std::string contact_name, 
                                  const Eigen::Vector3d& normal_ref)
{
    _cpl_problem->SetContactNormal(contact_name, 
                                   normal_ref);
}


void CoMPlanner::SetCoMRef(const Eigen::Vector3d& com_ref)
{
    _cpl_problem->SetCoMRef(com_ref);
}


void CoMPlanner::SetMu(double mu)
{
    _cpl_problem->SetMu(mu);
}


void CoMPlanner::SetForceThreshold(std::string contact_name, 
                                   double F_thr)
{
    Eigen::Vector3d force_lb, force_ub;
    _cpl_problem->GetForceBounds(contact_name, force_lb, force_ub);
    
    if ( force_lb != Eigen::Vector3d::Zero() && force_ub != Eigen::Vector3d::Zero())
    {
        _cpl_problem->SetForceThreshold(contact_name,
                                        F_thr);
    }
}