#ifndef __MINIMIZE_CENTROIDAL_VARIABLES__
#define __MINIMIZE_CENTROIDAL_VARIABLES__


#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Ifopt/Types.h>

namespace cpl { namespace solver {

/**
* @brief Miniminization of: CoM position, contact positions and contact forces
*/  
class MinimizeCentroidalVariables : public ifopt::CostTerm {
    
public:
    
    typedef std::shared_ptr<MinimizeCentroidalVariables> Ptr;

    MinimizeCentroidalVariables(std::map<std::string, ContactVars> contact_vars_map,
                                Variable3D::Ptr com_var);
  
    void SetPosRef(std::string contact_name, 
                   const Eigen::Vector3d& pos_ref);
    
    Eigen::Vector3d GetPosRef(std::string contact_name) const;
    
    void SetForceRef(std::string contact_name, 
                     const Eigen::Vector3d& force_ref);
    
    Eigen::Vector3d GetForceRef(std::string contact_name) const;
    
    void SetCoMRef(const Eigen::Vector3d& CoM_ref);
    
    Eigen::Vector3d GetCoMRef() const;
    
    void SetCoMWeight(double W_CoM);
    
    double GetCoMWeight () const;
    
    void SetPosWeight(double W_p);
    
    void SetContactPosWeight(std::string contact_name,
                             double W_p);
    
    double GetContactPosWeight(std::string contact_name) const; 
    
    void SetForceWeight(double W_F);
    
    void SetContactForceWeight(std::string contact_name,
                               double W_F);
    
    double GetContactForceWeight(std::string contact_name) const; 
  
private:
    
    double GetCost() const override;
    void FillJacobianBlock (std::string var_set, Jacobian& jac) const override;

    std::map<std::string, ContactVars> _contact_vars_map;
    std::map<std::string, ContactValues> _contact_vars_ref_map;
    std::map<std::string, double> _pos_weight_map, _force_weight_map;

    Eigen::Vector3d _CoM_ref;
    double _W_p, _W_F, _W_CoM;
    
    Variable3D::Ptr _com_var;
    
};

} }


#endif