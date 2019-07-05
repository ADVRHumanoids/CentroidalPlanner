#include <CentroidalPlanner/CentroidalPlanner.h>

namespace cpl { 
 
/**
* @brief The CoMPlanner class provides CoM position and contact forces for balancing
* given contact positions, contact normals and the lifting contact/s.
*/    
class CoMPlanner
{
    
public:
    
    typedef std::shared_ptr<CoMPlanner> Ptr;

    CoMPlanner(std::vector<std::string> contact_names,
                      double robot_mass);   
    
    
    solver::Solution Solve();
    
    void SetLiftingContact(std::string contact_name);
      
    void SetContactPosition(std::string contact_name, const Eigen::Vector3d& pos_ref);
    void SetContactNormal(std::string contact_name, const Eigen::Vector3d& normal_ref);
    void SetCoMRef(const Eigen::Vector3d& com_ref);
    
    void ResetForceBounds(std::string contact_name);
    
    void SetMu(double mu);
    void SetForceThreshold(std::string contact_name, double F_thr);
   
    
private:
    
    cpl::CentroidalPlanner::Ptr _cpl_problem;
    std::vector<std::string> _contact_names;
    double _robot_mass;   
    
};
    
}