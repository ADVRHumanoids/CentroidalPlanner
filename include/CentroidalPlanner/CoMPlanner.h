#include <CentroidalPlanner/CentroidalPlanner.h>

namespace cpl { 
 
/**
* @brief The CoMPlanner class provides CoM position and contact forces for balancing
* given contact positions, contact normals and the lifting contact/s.
*/    
class CoMPlanner : public CentroidalPlanner {
    
public:
    
    typedef std::shared_ptr<CoMPlanner> Ptr;

    CoMPlanner(std::vector<std::string> contact_names,
               double robot_mass);   
    
    void SetLiftingContact(std::string contact_name);
      
    void SetContactPosition(std::string contact_name, 
                            const Eigen::Vector3d& pos_ref);

    Eigen::Vector3d GetContactPosition(std::string contact_name) const;
    
    void SetContactNormal(std::string contact_name, 
                          const Eigen::Vector3d& n_ref);

    Eigen::Vector3d GetContactNormal(std::string contact_name) const;
    
};

} 