#include <CentroidalPlanner/CentroidalPlanner.h>

namespace cpl { 
 
/**
* @brief The CoMPlanner class provides CoM position and contact forces for 
* static balancing given: contact positions, contact normals and lifting contact/s.
* @throw exception if robot mass <= 0
*/    
class CoMPlanner : private CentroidalPlanner {
    
public:
    
    typedef std::shared_ptr<CoMPlanner> Ptr;

    CoMPlanner(std::vector<std::string> contact_names,
               double robot_mass);   
    
    /**
    * @brief Set the lifting contact.
    * NOTE: the related contact force is equal to zero.
    * @throw exception if invalid contact name.
    */
    void SetLiftingContact(std::string contact_name);
    
    std::vector<std::string> GetLiftingContacts() const;
    
    /**
    * @brief Set the contact position as equality constraint. 
    * @throw exception if invalid contact name.
    */
    void SetContactPosition(std::string contact_name, 
                            const Eigen::Vector3d& pos_ref);
  
    /**
    * @throw exception if invalid contact name.
    */
    Eigen::Vector3d GetContactPosition(std::string contact_name) const;
    
    /**
    * @brief Set the contact normal as equality constraint. 
    * @throw exception if invalid contact name.
    * @throw exception if contact normal has not unitary norm.
    */
    void SetContactNormal(std::string contact_name, 
                          const Eigen::Vector3d& n_ref);

    /**
    * @throw exception if invalid contact name.
    */
    Eigen::Vector3d GetContactNormal(std::string contact_name) const;
    
    void SetMu(double mu);
    
    using CentroidalPlanner::Solve;
    using CentroidalPlanner::SetCoMWeight;
    using CentroidalPlanner::SetPosWeight;
    using CentroidalPlanner::SetForceWeight;
    using CentroidalPlanner::SetCoMRef;
    using CentroidalPlanner::ResetForceBounds;
    using CentroidalPlanner::SetForceThreshold;
    
private:

    std::vector<std::string> _contact_names; 
    
};

} 