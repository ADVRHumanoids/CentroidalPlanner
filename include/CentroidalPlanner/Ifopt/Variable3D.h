#ifndef __VARIABLE_3D__
#define __VARIABLE_3D__


#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 

namespace cpl { namespace solver {
    
/**
* @brief 3D optimization variable
*/
class Variable3D : public ifopt::VariableSet {
    
public:
    
    typedef std::shared_ptr<Variable3D> Ptr;
     
    Variable3D(const std::string& var_name);
    
    void SetVariables(const VectorXd& x) override;
    
    void SetBounds(const Eigen::Vector3d& lower, 
                   const Eigen::Vector3d& upper);
    
    VecBound GetBounds() const override;
    
    Eigen::VectorXd GetValues() const override;
        
private:
    
    double _x0, _x1, _x2;
    Eigen::Vector3d _lb, _ub;
  
};

} }
    
    
#endif