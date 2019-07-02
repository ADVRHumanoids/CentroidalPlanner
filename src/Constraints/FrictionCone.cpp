#include <CentroidalPlanner/Ifopt/Constraints/FrictionCone.h>

using namespace cpl::solver;

FrictionCone::FrictionCone(std::string contact_name, CPLSolver::ContactVars contact_vars, cpl::env::EnvironmentClass::Ptr env):
    ConstraintSet(2, "FrictionCone" + contact_name),
    _contact_name(contact_name),
    _contact_vars(contact_vars),
    _env(env)
{
   
  _F = _contact_vars.force_var->GetValues();    
  _n = _contact_vars.normal_var->GetValues();
 
  _mu = _env->GetMu(); 
  _force_thr = 0.0; 
  
}
    
void FrictionCone::SetMu(const double& mu)
{
    _mu = mu;
}

void FrictionCone::SetForceThreshold(const double& force_thr)
{
    _force_thr = force_thr;
}

Eigen::VectorXd FrictionCone::GetValues() const 
{
   
    Eigen::VectorXd value;           
    value.setZero(2);
     
    value(0) = -_F.dot(_n) + _force_thr; 
    value(1) = (_F-(_n.dot(_F))*_n).norm() - _mu*(_F.dot(_n));                 

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

void FrictionCone::FillJacobianBlock (std::string var_set, ifopt::Composite::Jacobian& jac_block) const 
{
    
  jac_block.setZero();


   double t1 = _F.dot(_n);
   
   double t2 = _F.x()-_n.x()*t1;
   double t3 = _F.y()-_n.y()*t1;
   double t4 = _F.z()-_n.z()*t1;
   
   double t5 = _F.x()*_n.x();
   double t6 = _F.y()*_n.y();
   double t7 = _F.z()*_n.z();
   
   if(var_set == "F_" + _contact_name)
   {     
       
        jac_block.coeffRef(0, 0) = -_n.x();
        jac_block.coeffRef(0, 1) = -_n.y();
        jac_block.coeffRef(0, 2) = -_n.z();

        jac_block.coeffRef(1, 0) = (t2*(_n.x()*_n.x()-1.0)*2.0+_n.x()*_n.y()*t3*2.0+_n.x()*_n.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - _mu*_n.x();
        jac_block.coeffRef(1, 1) = (t3*(_n.y()*_n.y()-1.0)*2.0+_n.x()*_n.y()*t2*2.0+_n.y()*_n.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - _mu*_n.y();
        jac_block.coeffRef(1, 2) = (t4*(_n.z()*_n.z()-1.0)*2.0+_n.x()*_n.z()*t2*2.0+_n.y()*_n.z()*t3*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - _mu*_n.z();

   }
   

   if(var_set == "n_" + _contact_name)
   {     
        jac_block.coeffRef(0, 0) = -_F.x();
        jac_block.coeffRef(0, 1) = -_F.y();
        jac_block.coeffRef(0, 2) = -_F.z();
        
        jac_block.coeffRef(1, 0) = (t2*(t6+t7+t5*2.0)*2.0+_F.x()*_n.y()*t3*2.0+_F.x()*_n.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - _mu*_F.x();
        jac_block.coeffRef(1, 1) = (t3*(t5+t7+t6*2.0)*2.0+_F.y()*_n.x()*t2*2.0+_F.y()*_n.z()*t4*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - _mu*_F.y();
        jac_block.coeffRef(1, 2) = (t4*(t5+t6+t7*2.0)*2.0+_F.z()*_n.x()*t2*2.0+_F.z()*_n.y()*t3*2.0)*1.0/sqrt(t2*t2+t3*t3+t4*t4)*(-1.0/2.0) - _mu*_F.z();
                 
   }
    
}