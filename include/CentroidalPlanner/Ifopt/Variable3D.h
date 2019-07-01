#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <Eigen/Geometry> 

namespace cpl { namespace solver {

class Variable3D : public ifopt::VariableSet {
    
public:
    
    Variable3D(const std::string& var_name);
    
    void SetBounds(const Eigen::Vector3d& lower, const Eigen::Vector3d& upper);
    
        
private:
        
    void SetVariables(const VectorXd& x) override;
    VecBound GetBounds() const override;
    
    double _x0, _x1, _x2;
    Eigen::Vector3d _lb, _ub;
  
};

} }
    