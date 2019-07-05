#include <CentroidalPlanner/Ifopt/Types.h>
#include <CentroidalPlanner/Ifopt/CplProblem.h>
#include <CentroidalPlanner/Environment/Superquadric.h>
#include <CentroidalPlanner/Environment/Ground.h>
#include <ifopt/ipopt_solver.h>
#include <iostream>

namespace cpl { 
 
/**
* @brief The CentroidalPlanner class provides a generic way 
* to perform non linear optimization over centroidal statics.
*/    
class CentroidalPlanner
{
    
public:
    
    typedef std::shared_ptr<CentroidalPlanner> Ptr;

    CentroidalPlanner(std::vector<std::string> contact_names,
                      double robot_mass,
                      env::EnvironmentClass::Ptr env);
    
    solver::Solution Solve();
    
    void SetManipulationWrench(const Eigen::VectorXd& wrench_manip);
    
    void SetForceBounds(std::string contact_name, const Eigen::Vector3d& force_lb, const Eigen::Vector3d& force_ub);
    void SetPosBounds(std::string contact_name, const Eigen::Vector3d& pos_lb, const Eigen::Vector3d& pos_ub);
    void SetNormalBounds(std::string contact_name, const Eigen::Vector3d& normal_lb, const Eigen::Vector3d& normal_ub);
    
    void SetPosRef(std::string contact_name, const Eigen::Vector3d& pos_ref);
    void SetCoMRef(const Eigen::Vector3d& com_ref);
    
    void SetCoMWeight(double W_CoM);
    void SetPosWeight(double W_p);
    void SetForceWeight(double W_F);
    
    void SetMu(double mu);
    void SetForceThreshold(double F_thr);
    
private:
    
    solver::CplProblem::Ptr _cpl_problem;
    ifopt::IpoptSolver _cpl_solver;
    env::EnvironmentClass::Ptr _env;   
    std::vector<std::string> _contact_names;
    double _robot_mass;   
    
};
    
}