#include <CentroidalPlanner/CentroidalPlanner.h>

using namespace cpl;

CentroidalPlanner::CentroidalPlanner(std::vector< std::string > contact_names, 
                                     double robot_mass, 
                                     cpl::env::EnvironmentClass::Ptr env) :
    _contact_names(contact_names),
    _robot_mass(robot_mass),
    _env(env)
{
    
    _cpl_problem = std::make_shared<solver::CplProblem> (_contact_names, _robot_mass, _env);
    
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
    
    _cpl_problem->SetForceBounds(contact_name, force_lb, force_ub);

}


void CentroidalPlanner::SetPosBounds(std::string contact_name, 
                                     const Eigen::Vector3d& pos_lb, 
                                     const Eigen::Vector3d& pos_ub)
{
    
    _cpl_problem->SetPosBounds(contact_name, pos_lb, pos_ub);

}


void CentroidalPlanner::SetNormalBounds(std::string contact_name, 
                                        const Eigen::Vector3d& normal_lb, 
                                        const Eigen::Vector3d& normal_ub)
{
    
    _cpl_problem->SetNormalBounds(contact_name, normal_lb, normal_ub);

}


void CentroidalPlanner::SetPosRef(std::string contact_name, 
                                  const Eigen::Vector3d& pos_ref)
{
    
    _cpl_problem->SetPosRef(contact_name, pos_ref);

}


void CentroidalPlanner::SetCoMRef(const Eigen::Vector3d& com_ref)
{
    
    _cpl_problem->SetCoMRef(com_ref);

}


void CentroidalPlanner::SetCoMWeight(double W_CoM)
{

    _cpl_problem->SetCoMWeight(W_CoM);
    
}


void CentroidalPlanner::SetPosWeight(double W_p)
{
    
    _cpl_problem->SetPosWeight(W_p);

}


void CentroidalPlanner::SetForceWeight(double W_F)
{
    
    _cpl_problem->SetForceWeight(W_F);

}


void CentroidalPlanner::SetManipulationWrench(const Eigen::VectorXd& wrench_manip)
{
    
   _cpl_problem->SetManipulationWrench(wrench_manip); 

}

void  CentroidalPlanner::SetMu(double mu)
{
    
    _cpl_problem->SetMu(mu);

}


void  CentroidalPlanner::SetForceThreshold(std::string contact_name, double F_thr)
{

    _cpl_problem->SetForceThreshold(contact_name, F_thr);
    
}
