#ifndef __CPL_TYPES_H__
#define __CPL_TYPES_H__

#include <CentroidalPlanner/Ifopt/Variable3D.h>

namespace cpl { namespace solver {     

struct ContactValues
{
    Eigen::Vector3d force_value;
    Eigen::Vector3d position_value;
    Eigen::Vector3d normal_value;            
};    

struct Solution
{
    std::map<std::string, ContactValues> contact_values_map;
    Eigen::Vector3d com_sol;
    
    friend std::ostream& operator<<(std::ostream& os, const Solution& sol);
};

struct ContactVars
{
    cpl::solver::Variable3D::Ptr force_var;
    cpl::solver::Variable3D::Ptr position_var;
    cpl::solver::Variable3D::Ptr normal_var;            
};

} }

#endif