#include <CentroidalPlanner/Environment/Superquadric.h>

using namespace cpl::env;

Superquadric::Superquadric(const Eigen::Vector3d& C, const Eigen::Vector3d& R, const Eigen::Vector3d& P)
{

    _C = C;
    _R = R;
    _P = P;
    
}


void Superquadric::getEnvironmentValue(const Eigen::Vector3d& p, double& environment_Value)
{
    
    for(int i = 0; i < 3; i++)
    {            
        environment_Value += pow((p(i)-_C(i))/_R(i),_P(i));
    }

}

void Superquadric::getEnvironmentJacobian(const Eigen::Vector3d& p, Eigen::Vector3d& environment_Jacobian)
{
    
    environment_Jacobian.x() = _P.x()/pow(_R.x(),_P.x()) * pow(p.x()-_C.x(),_P.x()-1);
    environment_Jacobian.y() = _P.y()/pow(_R.y(),_P.y()) * pow(p.y()-_C.y(),_P.y()-1);
    environment_Jacobian.z() = _P.z()/pow(_R.z(),_P.z()) * pow(p.z()-_C.z(),_P.z()-1);

}


void Superquadric::getNormalValue(const Eigen::Vector3d& p, Eigen::Vector3d& normal_Value)
{
    
    normal_Value.x() = _P.x()/pow(_R.x(),_P.x()) * pow(p.x()-_C.x(),_P.x()-1);
    normal_Value.y() = _P.y()/pow(_R.y(),_P.y()) * pow(p.y()-_C.y(),_P.y()-1);
    normal_Value.z() = _P.z()/pow(_R.z(),_P.z()) * pow(p.z()-_C.z(),_P.z()-1);  

}


void Superquadric::getNormalJacobian(const Eigen::Vector3d& p, Eigen::MatrixXd& normal_Jacobian)
{
    
    normal_Jacobian.setZero(3,3);
    
    double t2 = -_C.x()+p.x();
    double t3 = _C.x()-p.x();
    double t4 = 1.0/(t3*t3);
    double t6 = _P.y()*2.0;
    double t5 = pow(_R.y(),-t6);
    double t7 = _C.y()-p.y();
    double t8 = 1.0/(t7*t7);
    double t10 = _P.z()*2.0;
    double t9 = pow(_R.z(),-t10);
    double t11 = _C.z()-p.z();
    double t12 = 1.0/(t11*t11);
    double t13 = _P.y()*_P.y();
    double t14 = -_C.y()+p.y();
    double t15 = pow(t14,t6);
    double t16 = _P.z()*_P.z();
    double t17 = -_C.z()+p.z();
    double t18 = pow(t17,t10);
    double t19 = pow(_R.z(),t10);
    double t20 = pow(_R.y(),t6);
    
    normal_Jacobian(0,0) = _P.x()*pow(_R.x(),-_P.x())*pow(t2,_P.x())*t4*t5*t8*t9*t12*(_P.x()-1.0)*1.0/pow(t5*t8*t13*t15+t9*t12*t16*t18+(_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*
                           pow(t2,_P.x()*2.0)*t4,3.0/2.0)*((_C.y()*_C.y())*t16*t18*t20+(_C.z()*_C.z())*t13*t15*t19+(p.y()*p.y())*t16*t18*t20 +(p.z()*p.z())*t13*t15*t19-_C.y()*
                           p.y()*t16*t18*t20*2.0-_C.z()*p.z()*t13*t15*t19*2.0);
    
    t2 = _P.y()*2.0;
    t3 = -_C.x()+p.x();
    t4 = _P.y()*_P.y();
    t5 = -_C.y()+p.y();
    t6 = t2-2.0;
    t7 = pow(_R.y(),-t2);
        
    normal_Jacobian(0,1) = _P.x()*pow(_R.x(),-_P.x())*pow(t3,_P.x()-1.0)*t4*pow(t5,t2-3.0)*t6*t7*1.0/pow((_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*pow(-_C.z()+p.z(),_P.z()*2.0-2.0)
                           +(_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*pow(t3,_P.x()*2.0-2.0)+t4*pow(t5,t6)*t7,3.0/2.0)*(-1.0/2.0);
    
    t2 = _P.z()*2.0;   
    t3 = -_C.x()+p.x();
    t4 = _P.z()*_P.z();
    t5 = -_C.z()+p.z();   
    t6 = t2-2.0;   
    t7 = pow(_R.z(),-t2);  
    
    normal_Jacobian(0,2) =  _P.x()*pow(_R.x(),-_P.x())*pow(t3,_P.x()-1.0)*t4*pow(t5,t2-3.0)*t6*t7*1.0/pow((_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*pow(-_C.y()+p.y(),_P.y()*2.0-2.0)
                            +(_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*pow(t3,_P.x()*2.0-2.0)+t4*pow(t5,t6)*t7,3.0/2.0)*(-1.0/2.0);                              
    
    t2 = _P.x()*2.0;
    t3 = _P.x()*_P.x();
    t4 = -_C.x()+p.x();
    t5 = t2-2.0;
    t6 = -_C.y()+p.y();
    t7 = pow(_R.x(),-t2);
    
    normal_Jacobian(1,0) = _P.y()*pow(_R.y(),-_P.y())*t3*pow(t4,t2-3.0)*t5*pow(t6,_P.y()-1.0)*t7*1.0/pow((_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*pow(-_C.z()+p.z(),_P.z()*2.0-2.0)
                            +(_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*pow(t6,_P.y()*2.0-2.0)+t3*pow(t4,t5)*t7,3.0/2.0)*(-1.0/2.0);
    
    t3 = _P.x()*2.0;
    t2 = pow(_R.x(),-t3);
    t4 = _C.x()-p.x();
    t5 = 1.0/(t4*t4);
    t6 = -_C.y()+p.y();
    t7 = _C.y()-p.y();
    t8 = 1.0/(t7*t7);
    t10 = _P.z()*2.0;
    t9 = pow(_R.z(),-t10);
    t11 = _C.z()-p.z();
    t12 = 1.0/(t11*t11);
    t13 = _P.x()*_P.x();
    t14 = -_C.x()+p.x();
    t15 = pow(t14,t3);
    t16 = _P.z()*_P.z();
    t17 = -_C.z()+p.z();
    t18 = pow(t17,t10);
    t19 = pow(_R.z(),t10);
    t20 = pow(_R.x(),t3);
    
    normal_Jacobian(1,1) = _P.y()*pow(_R.y(),-_P.y())*t2*t5*pow(t6,_P.y())*t8*t9*t12*(_P.y()-1.0)*1.0/pow(t2*t5*t13*t15+t9*t12*t16*t18+(_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*
                           pow(t6,_P.y()*2.0)*t8,3.0/2.0)*((_C.x()*_C.x())*t16*t18*t20+(_C.z()*_C.z())*t13*t15*t19+(p.x()*p.x())*t16*t18*t20+(p.z()*p.z())*t13*t15*t19-_C.x()*p.x()
                           *t16*t18*t20*2.0-_C.z()*p.z()*t13*t15*t19*2.0);

    t2 = _P.z()*2.0;
    t3 = -_C.y()+p.y();
    t4 = _P.z()*_P.z();
    t5 = -_C.z()+p.z();
    t6 = t2-2.0;
    t7 = pow(_R.z(),-t2);
    
    normal_Jacobian(1,2) = _P.y()*pow(_R.y(),-_P.y())*pow(t3,_P.y()-1.0)*t4*pow(t5,t2-3.0)*t6*t7*1.0/pow((_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*pow(-_C.x()+p.x(),_P.x()*2.0-2.0)
                           +(_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*pow(t3,_P.y()*2.0-2.0)+t4*pow(t5,t6)*t7,3.0/2.0)*(-1.0/2.0);

    t2 = _P.x()*2.0;
    t3 = _P.x()*_P.x();
    t4 = -_C.x()+p.x();
    t5 = t2-2.0;
    t6 = -_C.z()+p.z();
    t7 = pow(_R.x(),-t2);
    
    normal_Jacobian(2,0) = _P.z()*pow(_R.z(),-_P.z())*t3*pow(t4,t2-3.0)*t5*pow(t6,_P.z()-1.0)*t7*1.0/pow((_P.y()*_P.y())*pow(_R.y(),_P.y()*-2.0)*pow(-_C.y()+p.y(),_P.y()*2.0-2.0)
                           +(_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*pow(t6,_P.z()*2.0-2.0)+t3*pow(t4,t5)*t7,3.0/2.0)*(-1.0/2.0);

    t2 = _P.y()*2.0;
    t3 = _P.y()*_P.y();
    t4 = -_C.y()+p.y();
    t5 = t2-2.0;
    t6 = -_C.z()+p.z();
    t7 = pow(_R.y(),-t2);
 
    normal_Jacobian(2,1) = _P.z()*pow(_R.z(),-_P.z())*t3*pow(t4,t2-3.0)*t5*pow(t6,_P.z()-1.0)*t7*1.0/pow((_P.x()*_P.x())*pow(_R.x(),_P.x()*-2.0)*pow(-_C.x()+p.x(),_P.x()*2.0-2.0)
                           +(_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*pow(t6,_P.z()*2.0-2.0)+t3*pow(t4,t5)*t7,3.0/2.0)*(-1.0/2.0);

    t3 = _P.x()*2.0;
    t2 = pow(_R.x(),-t3);
    t4 = _C.x()-p.x();
    t5 = 1.0/(t4*t4);
    t7 = _P.y()*2.0;
    t6 = pow(_R.y(),-t7);
    t8 = _C.y()-p.y();
    t9 = 1.0/(t8*t8);
    t10 = -_C.z()+p.z();
    t11 = _C.z()-p.z();
    t12 = 1.0/(t11*t11);
    t13 = _P.x()*_P.x();
    t14 = -_C.x()+p.x();
    t15 = pow(t14,t3);
    t16 = _P.y()*_P.y();
    t17 = -_C.y()+p.y();
    t18 = pow(t17,t7);
    t19 = pow(_R.y(),t7);
    t20 = pow(_R.x(),t3);
     
    normal_Jacobian(2,2) = _P.z()*pow(_R.z(),-_P.z())*t2*t5*t6*t9*pow(t10,_P.z())*t12*(_P.z()-1.0)*1.0/pow(t2*t5*t13*t15+t6*t9*t16*t18+(_P.z()*_P.z())*pow(_R.z(),_P.z()*-2.0)*
                           pow(t10,_P.z()*2.0)*t12,3.0/2.0)*((_C.x()*_C.x())*t16*t18*t20+(_C.y()*_C.y())*t13*t15*t19+(p.x()*p.x())*t16*t18*t20+(p.y()*p.y())*t13*t15*t19-_C.x()*
                           p.x()*t16*t18*t20*2.0-_C.y()*p.y()*t13*t15*t19*2.0);
    
}

