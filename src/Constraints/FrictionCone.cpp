#include <CentroidalPlanner/Ifopt/Constraints/FrictionCone.h>
#include <iostream>

using namespace cpl::solver;

FrictionCone::FrictionCone(std::string contact_name, 
                           ContactVars contact_vars, 
                           cpl::env::EnvironmentClass::Ptr env):
    ConstraintSet(2, "Friction cone: " + contact_name),
    _contact_name(contact_name),
    _contact_vars(contact_vars),
    _env(env)
{
    _F_thr = 0.0; 
}
 
 
void FrictionCone::SetForceThreshold(double F_thr)
{
    _F_thr = F_thr;
}


Eigen::VectorXd FrictionCone::GetValues() const 
{
   
    Eigen::VectorXd value;           
    value.setZero(2);
    
    Eigen::Vector3d F = _contact_vars.force_var->GetValues();    
    Eigen::Vector3d n = _contact_vars.normal_var->GetValues();
     
    value(0) = -F.dot(n) + _F_thr; 
    value(1) = (F-(n.dot(F))*n).norm() - _env->GetMu()*(F.dot(n)); 
    

    return value;
    
};


ifopt::Composite::VecBound FrictionCone::GetBounds() const 
{
    VecBound b(GetRows());
    
    for(int i = 0; i < 2; i++)
    {            
         b.at(i) = ifopt::BoundSmallerZero;               
    }        
                        
    return b;
}

void FrictionCone::FillJacobianBlock (std::string var_set, 
                                      ifopt::Composite::Jacobian& jac_block) const 
{
    
    double mu = _env->GetMu();
    
    jac_block.setZero();
    
    Eigen::Vector3d F = _contact_vars.force_var->GetValues();    
    Eigen::Vector3d n = _contact_vars.normal_var->GetValues();

    double t1 = F.dot(n);
    double t2 = F.x()-n.x()*t1;
    double t3 = F.y()-n.y()*t1;
    double t4 = F.z()-n.z()*t1;
    double t5 = F.x()*n.x();
    double t6 = F.y()*n.y();
    double t7 = F.z()*n.z();

    if(var_set == "F_" + _contact_name)
    {     
        
        jac_block.coeffRef(0, 0) = -n.x();
        jac_block.coeffRef(0, 1) = -n.y();
        jac_block.coeffRef(0, 2) = -n.z();
        jac_block.coeffRef(1, 0) = (t2*(n.x()*n.x()-1.0)*2.0+n.x()*n.y()*t3*2.0+n.x()*n.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu*n.x();
        jac_block.coeffRef(1, 1) = (t3*(n.y()*n.y()-1.0)*2.0+n.x()*n.y()*t2*2.0+n.y()*n.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu*n.y();
        jac_block.coeffRef(1, 2) = (t4*(n.z()*n.z()-1.0)*2.0+n.x()*n.z()*t2*2.0+n.y()*n.z()*t3*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu*n.z();

    }

    if(var_set == "n_" + _contact_name)
    {     
        jac_block.coeffRef(0, 0) = -F.x();
        jac_block.coeffRef(0, 1) = -F.y();
        jac_block.coeffRef(0, 2) = -F.z();
        
        jac_block.coeffRef(1, 0) = (t2*(t6+t7+t5*2.0)*2.0+F.x()*n.y()*t3*2.0+F.x()*n.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu*F.x();
        jac_block.coeffRef(1, 1) = (t3*(t5+t7+t6*2.0)*2.0+F.y()*n.x()*t2*2.0+F.y()*n.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu*F.y();
        jac_block.coeffRef(1, 2) = (t4*(t5+t6+t7*2.0)*2.0+F.z()*n.x()*t2*2.0+F.z()*n.y()*t3*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - mu*F.z();
                
    }
 
}
