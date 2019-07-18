#ifndef __CENTROIDAL_STATICS__
#define __CENTROIDAL_STATICS__


#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 
#include <CentroidalPlanner/Ifopt/Types.h>

namespace cpl { namespace solver {
  
class CentroidalStatics : public ifopt::ConstraintSet {
    
public:
    
    typedef std::shared_ptr<CentroidalStatics> Ptr;

    CentroidalStatics(std::map<std::string, ContactVars> contact_vars_map,
                      Variable3D::Ptr com_var);

    void SetMass(const double& m);
    
    void SetManipulationWrench(const Eigen::VectorXd& wrench_manip);
    
    Eigen::VectorXd GetManipulationWrench() const; 
    
private:

    Eigen::VectorXd GetValues() const override;
    
    VecBound GetBounds() const override;
    
    void FillJacobianBlock (std::string var_set, 
                            Jacobian& jac_block) const override;

    std::map<std::string, ContactVars> _contact_vars_map;  
    Variable3D::Ptr _com_var;
    double _m;
    Eigen::Vector3d _g;
    Eigen::VectorXd _wrench_manip;

};

} }


#endif