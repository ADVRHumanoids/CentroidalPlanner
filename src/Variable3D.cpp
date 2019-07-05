#include <CentroidalPlanner/Ifopt/Variable3D.h>

using namespace cpl::solver;

Variable3D::Variable3D (const std::string& var_name) : VariableSet(3, var_name)
{
    
    _x0 = 0.0;
    _x1 = 0.0;
    _x2 = 0.0;

    _lb.setConstant(-1000.0);
    _ub.setConstant(1000.0);
    
}


void Variable3D::SetVariables(const Eigen::VectorXd& x) 
{
    
    _x0 = x(0);
    _x1 = x(1);
    _x2 = x(2);

};


void Variable3D::SetBounds(const Eigen::Vector3d& lower, 
                           const Eigen::Vector3d& upper)
{
 
    _lb = lower;
    _ub = upper;
   
    if( ((_ub - _lb).array() < 0).any() )
    {
        throw std::invalid_argument("Inconsistent bounds");
    }
    
}

Eigen::VectorXd Variable3D::GetValues() const
{

    Eigen::VectorXd temp; 
    temp.setZero(3);
    temp << _x0, _x1, _x2;
        
    return temp;
    
}


ifopt::Composite::VecBound Variable3D::GetBounds() const
{
    
    VecBound bounds(GetRows());
    
    bounds.at(0) = ifopt::Bounds(_lb(0), _ub(0));
    bounds.at(1) = ifopt::Bounds(_lb(1), _ub(1));
    bounds.at(2) = ifopt::Bounds(_lb(2), _ub(2));
    
    return bounds;
    
}