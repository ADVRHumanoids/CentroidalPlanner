#ifndef __CENTROIDAL_PLANNER__
#define __CENTROIDAL_PLANNER__


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
* @throw exception if robot mass <= 0
*/    
class CentroidalPlanner
{
    
public:
    
    typedef std::shared_ptr<CentroidalPlanner> Ptr;
    
    CentroidalPlanner(std::vector<std::string> contact_names,
                      double robot_mass,
                      env::EnvironmentClass::Ptr env);
    
    solver::Solution Solve();
    
    /**
    * @brief Set the manipulation wrench at the Center of Mass.
    */
    void SetManipulationWrench(const Eigen::VectorXd& wrench_manip);
    
    Eigen::VectorXd GetManipulationWrench() const; 
    
    /**
    * @throw exception if invalid contact name
    * @throw exception if inconsistent bounds
    */
    void SetForceBounds(std::string contact_name, 
                        const Eigen::Vector3d& force_lb, 
                        const Eigen::Vector3d& force_ub);
    
    /**
    * @throw exception if invalid contact name
    */
    void GetForceBounds(std::string contact_name, 
                        Eigen::Vector3d& force_lb, 
                        Eigen::Vector3d& force_ub) const;
                        
    /**
    * @throw exception if invalid contact name
    * @throw exception if inconsistent bounds
    */                      
    void SetPosBounds(std::string contact_name, 
                      const Eigen::Vector3d& pos_lb, 
                      const Eigen::Vector3d& pos_ub);
    
    /**
    * @throw exception if invalid contact name
    */ 
    void GetPosBounds(std::string contact_name, 
                      Eigen::Vector3d& pos_lb, 
                      Eigen::Vector3d& pos_ub) const;
    
    /**
    * @throw exception if invalid contact name
    */                   
    void GetNormalBounds(std::string contact_name, 
                         Eigen::Vector3d& normal_lb, 
                         Eigen::Vector3d& normal_ub) const;
                         
    /**
    * @throw exception if invalid contact name
    */  
    void SetPosRef(std::string contact_name,
                   const Eigen::Vector3d& pos_ref);
    
    /**
    * @throw exception if invalid contact name
    */  
    Eigen::Vector3d GetPosRef(std::string contact_name) const; 
    
    /**
    * @throw exception if invalid contact name
    */  
    void SetForceRef(std::string contact_name,
                    const Eigen::Vector3d& force_ref);
    
    /**
    * @throw exception if invalid contact name
    */  
    Eigen::Vector3d GetForceRef(std::string contact_name) const; 
      
    void SetCoMRef(const Eigen::Vector3d& com_ref);
    
    Eigen::Vector3d GetCoMRef() const;
    
    /**
    * @throw exception if weight < 0.0
    */  
    void SetCoMWeight(double W_CoM);
    
    double GetCoMWeight() const; 
    
    /**
    * @brief Set the same position weight for every contact.
    * @throw exception if weight < 0.0
    */
    void SetPosWeight(double W_p);
    
    std::map<std::string, double> GetPosWeight() const;
    
    /**
    * @brief Set the position weight for a specific contact.
    * @throw exception if invalid contact name
    * @throw exception if weight < 0.0
    */
    void SetContactPosWeight(std::string contact_name,
                             double W_p);
    
    /**
    * @throw exception if invalid contact name
    */  
    double GetContactPosWeight(std::string contact_name) const; 
      
    /**
    * @brief Set the same force weight for every contact.
    * @throw exception if weight < 0.0
    */
    void SetForceWeight(double W_F);
    
    std::map<std::string, double> GetForceWeight() const;
    
    /**
    * @brief Set the force weight for a specific contact.
    * @throw exception if invalid contact name
    * @throw exception if weight < 0.0
    */
    void SetContactForceWeight(std::string contact_name,
                               double W_F);
    
    /**
    * @throw exception if invalid contact name
    */  
    double GetContactForceWeight(std::string contact_name) const; 
    
    double GetMu() const; 
    
    /**
    * @brief Set a lower bound on the contact force along
    * the direction constrained by the environment. 
    * @throw exception if invalid contact name
    * @throw exception if force threshold < 0.0
    */
    void SetForceThreshold(std::string contact_name, 
                           double F_thr);
        
    /**
    * @brief Set the lower bound on the contact force along
    * the direction constrained by the environment. 
    * @throw exception if invalid contact name
    */  
    double GetForceThreshold(std::string contact_name) const; 
    
    virtual ~CentroidalPlanner() = default; // polymorphic classes must have virtual destructor (good practice)
       
protected:  
    
    void SetNormalBounds(std::string contact_name, 
                         const Eigen::Vector3d& normal_lb,
                         const Eigen::Vector3d& normal_ub);
    
    bool HasContact ( const std::string& contact_name ) const;
    
    solver::CplProblem::Ptr GetCplProblem() const;

private:
    
    std::vector<std::string> _contact_names;
    double _robot_mass;
    env::EnvironmentClass::Ptr _env;   
    ifopt::IpoptSolver _cpl_solver;
    solver::CplProblem::Ptr _cpl_problem;
    
};
    
}


#endif
