#include <CentroidalPlanner/Ifopt/Constraints/EnvironmentConstraint.h>

using namespace cpl::solver::Environment;

EnvironmentConstraint::EnvironmentConstraint(const ContactVarName& contact_var_name, cpl::env::EnvironmentClass::Ptr& env):
    ConstraintSet(1, "EnvironmentConstraint"),
    _contact_var_name(contact_var_name),
    _env(env)
{
    
    _p = GetVariables()->GetComponent(_contact_var_name.position_name)->GetValues();
    
}

Eigen::VectorXd EnvironmentConstraint::GetValues() const 
{
    
    Eigen::VectorXd value; value.setZero(1);    
    _env->getEnvironmentValue(_p, value(0));   
    value(0) -= 1.0;
    
    return value;
 
}


ifopt::Composite::VecBound EnvironmentConstraint::GetBounds() const 
{
    
    VecBound b(GetRows());
            
    b.at(0) = ifopt::Bounds(0.0, 0.0);  
            
    return b;

}

void EnvironmentConstraint::FillJacobianBlock (std::string var_set, ifopt::Composite::Jacobian& jac_block) const 
{
    
    jac_block.setZero();
    
    Eigen::Vector3d jac;
    _env->getEnvironmentJacobian(_p, jac);   
    
    if(var_set == _contact_var_name.position_name)
    {
        
        jac_block.coeffRef(0, 0) = jac.x();
        jac_block.coeffRef(0, 1) = jac.y();
        jac_block.coeffRef(0, 2) = jac.z();
        
    }
      
}