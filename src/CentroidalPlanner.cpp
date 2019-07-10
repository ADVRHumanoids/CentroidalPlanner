#include <CentroidalPlanner/CentroidalPlanner.h>

using namespace cpl;

CentroidalPlanner::CentroidalPlanner(std::vector< std::string > contact_names, 
                                     double robot_mass, 
                                     cpl::env::EnvironmentClass::Ptr env) :
    _contact_names(contact_names),
    _robot_mass(robot_mass),
    _env(env)
{
    if (robot_mass < 0.0)
    {
        throw std::invalid_argument("Invalid robot mass");
    }
    
    _cpl_problem = std::make_shared<solver::CplProblem> (_contact_names, 
                                                         _robot_mass, 
                                                         _env);
}

solver::Solution CentroidalPlanner::Solve()
{
    solver::Solution sol;
    
    _cpl_solver.SetOption("derivative_test", "first-order");
    _cpl_solver.Solve(*_cpl_problem); 
    
    _cpl_problem->GetSolution(sol);
    
    return sol;
     
}


void CentroidalPlanner::SetForceBounds(std::string contact_name,
                                       const Eigen::Vector3d& force_lb, 
                                       const Eigen::Vector3d& force_ub)
{    
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    _cpl_problem->SetForceBounds(contact_name, 
                                 force_lb, 
                                 force_ub);
}


void CentroidalPlanner::GetForceBounds(std::string contact_name,
                                       Eigen::Vector3d& force_lb, 
                                       Eigen::Vector3d& force_ub) const
{
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    _cpl_problem->GetForceBounds(contact_name, 
                                 force_lb, 
                                 force_ub);
}


void CentroidalPlanner::SetPosBounds(std::string contact_name, 
                                     const Eigen::Vector3d& pos_lb, 
                                     const Eigen::Vector3d& pos_ub)
{  
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    _cpl_problem->SetPosBounds(contact_name, 
                               pos_lb, 
                               pos_ub);
}

void CentroidalPlanner::GetPosBounds(std::string contact_name,
                                     Eigen::Vector3d& pos_lb, 
                                     Eigen::Vector3d& pos_ub) const
{
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    _cpl_problem->GetPosBounds(contact_name, 
                               pos_lb, 
                               pos_ub);
}


void CentroidalPlanner::SetNormalBounds(std::string contact_name, 
                                        const Eigen::Vector3d& normal_lb, 
                                        const Eigen::Vector3d& normal_ub)
{    
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    if ( normal_lb.norm() != 1.0 || normal_ub.norm() != 1.0)
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "' normal bound");
    }
    
    _cpl_problem->SetNormalBounds(contact_name, 
                                  normal_lb, 
                                  normal_ub);
}


void CentroidalPlanner::GetNormalBounds(std::string contact_name, 
                                        Eigen::Vector3d& normal_lb, 
                                        Eigen::Vector3d& normal_ub) const
{   
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    _cpl_problem->GetNormalBounds(contact_name, 
                                  normal_lb, 
                                  normal_ub);
}

void CentroidalPlanner::SetPosRef(std::string contact_name, 
                                  const Eigen::Vector3d& pos_ref)
{    
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    _cpl_problem->SetPosRef(contact_name, pos_ref);
}


void CentroidalPlanner::SetCoMRef(const Eigen::Vector3d& com_ref)
{    
    _cpl_problem->SetCoMRef(com_ref);
}


void CentroidalPlanner::SetCoMWeight(double W_CoM)
{
    if (W_CoM < 0.0)
    {
        throw std::invalid_argument("Invalid weight");
    }
    
    _cpl_problem->SetCoMWeight(W_CoM);   
}


void CentroidalPlanner::SetPosWeight(double W_p)
{   
    if (W_p < 0.0)
    {
        throw std::invalid_argument("Invalid weight");
    }
    
    _cpl_problem->SetPosWeight(W_p);
}


void CentroidalPlanner::SetForceWeight(double W_F)
{   
    if (W_F < 0.0)
    {
        throw std::invalid_argument("Invalid weight");
    }
    
    _cpl_problem->SetForceWeight(W_F);
}


void CentroidalPlanner::SetManipulationWrench(const Eigen::VectorXd& wrench_manip)
{  
   _cpl_problem->SetManipulationWrench(wrench_manip); 
}

void  CentroidalPlanner::SetMu(double mu)
{  
    if (mu <= 0.0)
    {
        throw std::invalid_argument("Invalid friction coefficient");
    }
    
    _cpl_problem->SetMu(mu);
}


void  CentroidalPlanner::SetForceThreshold(std::string contact_name, 
                                           double F_thr)
{
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    if (F_thr < 0.0)
    {
        throw std::invalid_argument("Invalid '" + contact_name + "' force threshold");
    }
    
    _cpl_problem->SetForceThreshold(contact_name, 
                                    F_thr);  
}


bool CentroidalPlanner::HasContact(const std::string& contact_name) const
{   
    bool valid_contact = false;
    
    for (auto& elem: _contact_names)
    {        
        if(elem == contact_name)
            valid_contact = true;
    }
    
    return valid_contact;
}

