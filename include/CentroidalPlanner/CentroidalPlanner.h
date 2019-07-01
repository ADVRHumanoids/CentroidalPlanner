#include <CentroidalPlanner/Environment/Environment.h>
#include <CentroidalPlanner/Ifopt/Solver.h>

namespace cpl { 
 
    class CentroidalPlanner
    {
        
    public:
        
        struct Paramaters
        {
          
        };
        
        CentroidalPlanner(std::vector<std::string> contact_names,
                          double robot_mass,
                          Paramaters params = Paramaters());
        
        void setContactPosition(std::string, Eigen::Vector3d);
        
    private:
        
        ifopt::Solver::Ptr _lcm_solver;
        ifopt::Solver::Ptr _ctr_solver;
        
    };
    
}