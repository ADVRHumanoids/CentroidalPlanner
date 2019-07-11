#ifndef __CPL_PROBLEM__
#define __CPL_PROBLEM__


#include <ifopt/problem.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Ifopt/Variable3D.h>
#include <CentroidalPlanner/Ifopt/Constraints/CentroidalStatics.h>
#include <CentroidalPlanner/Ifopt/Constraints/EnvironmentConstraint.h>
#include <CentroidalPlanner/Ifopt/Constraints/EnvironmentNormal.h>
#include <CentroidalPlanner/Ifopt/Constraints/FrictionCone.h>
#include <CentroidalPlanner/Ifopt/MinimizeCentroidalVariables.h>
#include <CentroidalPlanner/Ifopt/Types.h>
#include <CentroidalPlanner/Environment/Environment.h>
#include <CentroidalPlanner/Environment/Ground.h>

namespace cpl { namespace solver {

/**
* @brief Centroidal statics problem with environment
*/      
class CplProblem : public ifopt::Problem {
    
public:
    
    typedef std::shared_ptr<CplProblem> Ptr;
      
    CplProblem(std::vector<std::string> contact_names,
               double robot_mass,
               env::EnvironmentClass::Ptr env);
    
    void GetSolution(Solution& sol);
    
    void SetManipulationWrench(const Eigen::VectorXd& wrench_manip);
    
    void SetForceBounds(std::string contact_name, 
                        const Eigen::Vector3d& force_lb, 
                        const Eigen::Vector3d& force_ub);
    
    void GetForceBounds(std::string contact_name, 
                        Eigen::Vector3d& force_lb, 
                        Eigen::Vector3d& force_ub) const;
    
    void SetPosBounds(std::string contact_name, 
                      const Eigen::Vector3d& pos_lb, 
                      const Eigen::Vector3d& pos_ub);
    
    void GetPosBounds(std::string contact_name, 
                      Eigen::Vector3d& pos_lb, 
                      Eigen::Vector3d& pos_ub) const;
    
    void SetNormalBounds(std::string contact_name, 
                         const Eigen::Vector3d& normal_lb, 
                         const Eigen::Vector3d& normal_ub);
    
    void GetNormalBounds(std::string contact_name, 
                         Eigen::Vector3d& normal_lb, 
                         Eigen::Vector3d& normal_ub) const;
    
    void SetPosRef(std::string contact_name, 
                   const Eigen::Vector3d& pos_ref);
    
    void SetCoMRef(const Eigen::Vector3d& com_ref);
    
    void SetCoMWeight(double W_CoM);
    
    void SetPosWeight(double W_p);
    
    void SetContactPosWeight(std::string contact_name, 
                             double W_p);
    
    void SetForceWeight(double W_F);
    
    void SetMu(double mu);
    
    void SetForceThreshold(std::string contact_name, 
                           double F_thr);
    
private:
    
    std::map<std::string, ContactVars> _contact_vars_map;  
    std::vector<std::string> _contact_names;
    double _robot_mass;
    env::EnvironmentClass::Ptr _env;
    env::Ground::Ptr _ground_fake;
    Variable3D::Ptr _com_var;
    CentroidalStatics::Ptr _centroidal_statics;
    EnvironmentConstraint::Ptr _env_const;
    EnvironmentNormal::Ptr _env_normal;
    FrictionCone::Ptr _friction_cone;
    MinimizeCentroidalVariables::Ptr _cost;
    std::map<std::string, FrictionCone::Ptr> _friction_cone_map;  
     
};


    
} }


#endif
