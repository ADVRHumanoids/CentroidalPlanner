#include <CentroidalPlanner/CentroidalPlanner.h>

using namespace cpl;

CentroidalPlanner::CentroidalPlanner(std::vector< std::string > contact_names, 
                                     double robot_mass, 
                                     cpl::env::EnvironmentClass::Ptr env) :
    _contact_names(contact_names),
    _robot_mass(robot_mass),
    _env(env)
{
    if (robot_mass <= 0.0)
    {
        throw std::invalid_argument("Invalid robot mass");
    }
    
    _cpl_problem = std::make_shared<solver::CplProblem> (_contact_names, 
                                                         _robot_mass, 
                                                         _env);
}

int CentroidalPlanner::Solve(solver::Solution& solution)
{
    _cpl_solver.SetOption("derivative_test", "first-order");
    _cpl_solver.SetOption("print_timing_statistics", "no");
   
    _cpl_solver.Solve(*_cpl_problem);  
    _cpl_problem->GetSolution(solution);

    return _cpl_solver.GetReturnStatus();
     
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


Eigen::Vector3d CentroidalPlanner::GetPosRef(std::string contact_name) const
{
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    return _cpl_problem->GetPosRef(contact_name);
}


void CentroidalPlanner::SetForceRef(std::string contact_name, 
                                    const Eigen::Vector3d& force_ref)
{    
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    _cpl_problem->SetForceRef(contact_name, force_ref);
}


Eigen::Vector3d CentroidalPlanner::GetForceRef(std::string contact_name) const
{
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    return _cpl_problem->GetForceRef(contact_name);
}


void CentroidalPlanner::SetCoMRef(const Eigen::Vector3d& com_ref)
{    
    _cpl_problem->SetCoMRef(com_ref);
}


Eigen::Vector3d CentroidalPlanner::GetCoMRef() const
{
    return _cpl_problem->GetCoMRef();
}


void CentroidalPlanner::SetCoMWeight(double W_CoM)
{
    if (W_CoM < 0.0)
    {
        throw std::invalid_argument("Invalid weight");
    }
    
    _cpl_problem->SetCoMWeight(W_CoM);   
}


double CentroidalPlanner::GetCoMWeight() const
{
    return _cpl_problem->GetCoMWeight();
}


void CentroidalPlanner::SetPosWeight(double W_p)
{   
    if (W_p < 0.0)
    {
        throw std::invalid_argument("Invalid weight");
    }
    
    _cpl_problem->SetPosWeight(W_p);
}


std::map<std::string, double> CentroidalPlanner::GetPosWeight() const
{
    std::map< std::string, double > pos_weight_map;
    
    for ( auto& elem: _contact_names)
    {
        pos_weight_map[elem] = _cpl_problem->GetContactPosWeight(elem);
    }
    
    return pos_weight_map;
}


void CentroidalPlanner::SetContactPosWeight(std::string contact_name, 
                                            double W_p)
{   
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    if (W_p < 0.0)
    {
        throw std::invalid_argument("Invalid weight");
    }
    
    _cpl_problem->SetContactPosWeight(contact_name, W_p);
}


double CentroidalPlanner::GetContactPosWeight(std::string contact_name) const
{
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    return _cpl_problem->GetContactPosWeight(contact_name);
}



void CentroidalPlanner::SetForceWeight(double W_F)
{   
    if (W_F < 0.0)
    {
        throw std::invalid_argument("Invalid weight");
    }
    
    _cpl_problem->SetForceWeight(W_F);
}


std::map< std::string, double > CentroidalPlanner::GetForceWeight() const
{
    std::map< std::string, double > force_weight_map;
    
    for ( auto& elem: _contact_names)
    {
        force_weight_map[elem] = _cpl_problem->GetContactForceWeight(elem);
    }
    
    return force_weight_map;
}


void CentroidalPlanner::SetContactForceWeight(std::string contact_name, 
                                             double W_F)
{   
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    if (W_F < 0.0)
    {
        throw std::invalid_argument("Invalid weight");
    }
    
    _cpl_problem->SetContactForceWeight(contact_name, W_F);
}


double CentroidalPlanner::GetContactForceWeight(std::string contact_name) const
{
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    return _cpl_problem->GetContactForceWeight(contact_name);
}


void CentroidalPlanner::SetManipulationWrench(const Eigen::VectorXd& wrench_manip)
{    
    _cpl_problem->SetManipulationWrench(wrench_manip); 
}


Eigen::VectorXd CentroidalPlanner::GetManipulationWrench() const
{
    return _cpl_problem->GetManipulationWrench();
}


double CentroidalPlanner::GetMu() const
{
    return _cpl_problem->GetMu();
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
        throw std::invalid_argument("Invalid force threshold");
    }
    
    Eigen::Vector3d force_lb, force_ub;
    _cpl_problem->GetForceBounds(contact_name, force_lb, force_ub);
    
    if ( force_lb != Eigen::Vector3d::Zero() && force_ub != Eigen::Vector3d::Zero())
    {
        _cpl_problem->SetForceThreshold(contact_name, 
                                        F_thr);
    } 
}


double CentroidalPlanner::GetForceThreshold(std::string contact_name) const
{
    if (!HasContact(contact_name))
    {
        throw std::invalid_argument("Invalid contact name: '" + contact_name + "'");
    }
    
    return _cpl_problem->GetForceThreshold(contact_name);
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


solver::CplProblem::Ptr CentroidalPlanner::GetCplProblem() const
{
    return _cpl_problem;
}
