#include <CentroidalPlanner/Ifopt/Constraints/EnvironmentNormal.h>

using namespace cpl::solver::Environment;

EnvironmentNormal::EnvironmentNormal(std::string contact_name, CPLSolver::ContactVars contact_vars, cpl::env::EnvironmentClass::Ptr env):
    ConstraintSet(3, "EnvironmentNormal" + contact_name),
    _contact_name(contact_name),
    _contact_vars(contact_vars),
    _env(env)
{
    
    _p = _contact_vars.position_var->GetValues();
    _n = _contact_vars.normal_var->GetValues();
    
}

Eigen::VectorXd EnvironmentNormal::GetValues() const 
{
    
    Eigen::Vector3d value;    
    _env->getNormalValue(_p, value); 
    
    value += _n;
    
    return value;
 
}


ifopt::Composite::VecBound EnvironmentNormal::GetBounds() const 
{
    
    VecBound b(GetRows());
            
    for(int i = 0; i < 3; i++)
    { 
        
        b.at(i) = ifopt::Bounds(.0, .0);  
        
    }

    return b; 

}

void EnvironmentNormal::FillJacobianBlock (std::string var_set, ifopt::Composite::Jacobian& jac_block) const 
{
    
    jac_block.setZero();
    
    Eigen::MatrixXd jac;
    _env->getNormalJacobian(_p, jac);   
    
    if(var_set == "n_" + _contact_name)
    {
        
        jac_block.coeffRef(0, 0) = 1.0;
        jac_block.coeffRef(0, 1) = 1.0;
        jac_block.coeffRef(0, 2) = 1.0;
        
    }
    
    if(var_set == "p_" + _contact_name)
    {
        
        jac_block.coeffRef(0, 0) = jac(0, 0);
        jac_block.coeffRef(0, 1) = jac(0, 1);
        jac_block.coeffRef(0, 2) = jac(0, 2);
        jac_block.coeffRef(1, 0) = jac(1, 0);
        jac_block.coeffRef(1, 1) = jac(1, 1);
        jac_block.coeffRef(1, 2) = jac(1, 2);
        jac_block.coeffRef(2, 0) = jac(2, 0);
        jac_block.coeffRef(2, 1) = jac(2, 1);
        jac_block.coeffRef(2, 2) = jac(2, 2);
        
    }
      
}

