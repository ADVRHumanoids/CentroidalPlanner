#ifndef __CPL_UTILS__
#define __CPL_UTILS__

#include <ros/ros.h>
#include <Eigen/Geometry>

#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <OpenSoT/tasks/force/FloatingBase.h>
#include <OpenSoT/constraints/force/CoP.h>
#include <OpenSoT/tasks/force/Force.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/ros/RosImpl.h>


namespace cpl { namespace utils {

    Eigen::Affine3d GetAffineFromNormal(const Eigen::Vector3d& p, const Eigen::Vector3d& n);

    /**
     * @brief This class computes the solution of a force distribution problem,
     * in order to actuate a full N+6 component torque vector on a floating-fixed_base
     * robot, by using ground reaction forces exchanged with the environment.
     *
     */
    class Cpl_ForceOptimization
    {

    public:

        typedef boost::shared_ptr<Cpl_ForceOptimization> Ptr;

        constexpr static double DEFAULT_FRICTION_COEFF = 0.5;

        /**
         * @brief Constructor
         *
         * @param model ModelInterface object that is kept updated with the robot state
         * @param contact_links List of contact links
         * @param optimize_torque False if point contacts are assumed
         */
        Cpl_ForceOptimization(XBot::ModelInterface::Ptr model,
                              std::vector<std::string> contact_links,
                              bool optimize_torque = true,
                              double friction_coeff = DEFAULT_FRICTION_COEFF
                              );

        /**
         * @brief Translate a fixed-base torque vector to an under-actuated
         * torque vector + contact forces.
         */
        bool compute(const Eigen::VectorXd& fixed_base_torque,
                     std::vector<Eigen::Vector6d>& Fc,
                     Eigen::VectorXd& tau
                    );

        void setContactRotationMatrix(const std::string& contact_link,
                                      const Eigen::Matrix3d& w_R_c);

        void setReferenceContactForce(const std::string& contact_link,
                                      const Eigen::Vector3d& Fc_ref);

        void log(XBot::MatLogger::Ptr logger);


    private:

        XBot::ModelInterface::Ptr _model;
        std::vector<std::string> _contact_links;
        std::vector< OpenSoT::AffineHelper > _wrenches;

        OpenSoT::constraints::force::FrictionCones::Ptr _friction_cone;
        OpenSoT::tasks::force::FloatingBase::Ptr _forza_giusta;
        OpenSoT::tasks::force::Wrenches::Ptr _min_wrench_aggr;
        OpenSoT::solvers::iHQP::Ptr _solver;
        OpenSoT::AutoStack::Ptr _autostack;

        Eigen::VectorXd _x_value;
        Eigen::MatrixXd _JC;
        Eigen::VectorXd _fc_i;
        Eigen::VectorXd _Fci;

    };

    /**
     * @brief This class detects if contact with the environment has been established,
     * based on estimated contact forces.
     *
     */
    class SurfaceReacher
    {

    public:

        typedef std::shared_ptr<SurfaceReacher> Ptr;

        /**
         * @brief Constructor
         *
         * @param contact_names List of contact links
         */

        SurfaceReacher(std::vector<std::string> contact_names);

        bool ReachSurface(XBot::Cartesian::RosImpl& ci,
                          std::string contact_name,
                          const Eigen::Vector3d contact_lin_vel,
                          double F_thr);

    private:

        std::map<std::string, ros::Subscriber> _sub_force_map;
        std::map<std::string, Eigen::Vector6d> _f_est_map;

    };

} }

#endif
