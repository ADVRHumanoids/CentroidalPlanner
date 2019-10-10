#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <OpenSoT/tasks/force/Force.h>
#include <OpenSoT/constraints/force/WrenchLimits.h>

namespace forza_giusta {

class ForzaGiusta : public OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>
{

public:

    typedef boost::shared_ptr<ForzaGiusta> Ptr;

    ForzaGiusta(XBot::ModelInterface::Ptr model,
                const std::vector<OpenSoT::AffineHelper>& wrenches,
                std::vector<std::string> contact_links);

    void setFixedBaseTorque(const Eigen::VectorXd& fixed_base_torque);

private:

    virtual void _update(const Eigen::VectorXd& x);

    virtual void _log(XBot::MatLogger::Ptr logger);

    XBot::ModelInterface::Ptr _model;
    std::vector<std::string> _contact_links;
    std::vector<bool> _enabled_contacts;
    std::vector<OpenSoT::AffineHelper> _wrenches;
    OpenSoT::AffineHelper _task;
    Eigen::MatrixXd _J_i;
    Eigen::Matrix6d _Jfb_i;
    Eigen::Vector6d _x_ref_fb;
    std::vector<Eigen::Vector6d> _Fref_ifopt;


};

ForzaGiusta::ForzaGiusta(XBot::ModelInterface::Ptr model, 
                         const std::vector< OpenSoT::AffineHelper >& wrenches,
                         std::vector< std::string > contact_links):
    Task< Eigen::MatrixXd, Eigen::VectorXd >("FORZA_GIUSTA", wrenches[0].getInputSize()),
    _model(model),
    _contact_links(contact_links),
    _wrenches(wrenches),
    _enabled_contacts(wrenches.size(), true)
{
    _x_ref_fb.setZero();
    
    update(Eigen::VectorXd());
}

void ForzaGiusta::setFixedBaseTorque(const Eigen::VectorXd& fixed_base_torque)
{
    if(fixed_base_torque.size() != _model->getJointNum())
    {
        throw std::invalid_argument("fixed_base_torque != _model->getJointNum()");
    }
    
    _x_ref_fb = fixed_base_torque.head<6>();
}


void ForzaGiusta::_update(const Eigen::VectorXd& x)
{
    _task.setZero(_wrenches[0].getInputSize(), 6);
    
    for(int i = 0; i < _enabled_contacts.size(); i++)
    {
        if(!_enabled_contacts[i]){
            continue;
        }
        else {
            _model->getJacobian(_contact_links[i], _J_i);
            _Jfb_i = _J_i.block<6,6>(0,0).transpose();
            _task = _task + _Jfb_i * _wrenches[i];
        }
    }
    
    _A = _task.getM();
    _b = -_task.getq() + _x_ref_fb;
}

void ForzaGiusta::_log(XBot::MatLogger::Ptr logger)
{
    OpenSoT::Task< Eigen::MatrixXd, Eigen::VectorXd >::_log(logger);
}



class ForceOptimization
{

public:

    typedef boost::shared_ptr<ForceOptimization> Ptr;

    ForceOptimization(XBot::ModelInterface::Ptr model,
                      std::vector<std::string> contact_links,
                      double mu,
                      bool optimize_contact_torque,
                      std::map<std::string, Eigen::VectorXd> constraint_links
                      );

    bool compute(const Eigen::VectorXd& fixed_base_torque,
                 const std::map<std::string, Eigen::Vector6d>& Fref_ifopt_map,
                 const std::map<std::string, Eigen::Matrix3d>& RotM_ifopt,
                 std::map<std::string, Eigen::Vector6d>& Fc_map);

    void log(XBot::MatLogger::Ptr logger);

    bool setConstraints(std::map<std::string, Eigen::VectorXd> constraint_links);
    bool resetConstraints(std::vector<std::string> reset_links);


private:

    XBot::ModelInterface::Ptr _model;
    std::vector<std::string> _contact_links;
    std::vector< OpenSoT::AffineHelper > _wrenches;
    std::vector<OpenSoT::tasks::MinimizeVariable::Ptr> _min_wrench;
    std::map<std::string, Eigen::VectorXd> _map_lowerLims, _map_upperLims;

    OpenSoT::tasks::force::Wrenches::Ptr _Wrenches;
    OpenSoT::constraints::force::WrenchesLimits::Ptr _Wrenches_limits;


    OpenSoT::constraints::force::FrictionCones::Ptr _friction_cones;
    ForzaGiusta::Ptr _forza_giusta;
    OpenSoT::solvers::iHQP::Ptr _solver;
    OpenSoT::AutoStack::Ptr _autostack;

    Eigen::VectorXd _x_value;
    Eigen::MatrixXd _JC;

    std::list<OpenSoT::solvers::iHQP::TaskPtr> min_wrench_tasks;

    OpenSoT::tasks::Aggregated::Ptr min_force_aggr;


};

}




forza_giusta::ForceOptimization::ForceOptimization(XBot::ModelInterface::Ptr model, 
                                                   std::vector< std::string > contact_links,
                                                   double mu,
                                                   bool optimize_contact_torque = false,
                                                   std::map<std::string, Eigen::VectorXd> constraint_links = std::map<std::string, Eigen::VectorXd>()
                                                   ):
    _model(model),
    _contact_links(contact_links)
{
    /* Do we want to consider contact torques? */
    //const bool optimize_contact_torque = false;
    
    /* Define optimization vector by stacking all contact wrenches */
    OpenSoT::OptvarHelper::VariableVector vars;

    for(auto cl : _contact_links)
    {
        vars.emplace_back(cl, optimize_contact_torque ? 6 : 3); // put 6 for full wrench
    }

    OpenSoT::OptvarHelper opt(vars);
    

    /* Define affine mappings for all wrenches */
    for(auto cl : _contact_links)
    {

        _wrenches.emplace_back(opt.getVariable(cl) /
                               OpenSoT::AffineHelper::Zero(opt.getSize(), optimize_contact_torque ? 0 : 3)
                               );

    }
    
    std::cout << "_wrenches.getInputSize()" << _wrenches[0].getInputSize() << std::endl;
    std::cout << "_wrenches.getOutputSize()" << _wrenches[0].getOutputSize() << std::endl;
    
    _Wrenches = boost::make_shared<OpenSoT::tasks::force::Wrenches>(_contact_links,_wrenches);

    /* Set Wrenches limits */
    setConstraints(constraint_links);
    
    /* Define friction cones */
    OpenSoT::constraints::force::FrictionCones::friction_cones friction_cones;
    
    Eigen::Matrix3d R; R.setIdentity();

    for(auto cl : _contact_links)
    {
        friction_cones.push_back(std::pair<Eigen::Matrix3d,double> (R,mu));
    }
    
    
    _friction_cones = boost::make_shared<OpenSoT::constraints::force::FrictionCones>(_contact_links, _wrenches, *_model, friction_cones);

    /* Construct forza giusta task */
    _forza_giusta = boost::make_shared<ForzaGiusta>(_model, _wrenches, _contact_links);
    
    /* Define optimization problem */
    _autostack = boost::make_shared<OpenSoT::AutoStack>(_Wrenches);
    _autostack << boost::make_shared<OpenSoT::constraints::TaskToConstraint>(_forza_giusta);
    _autostack << _friction_cones;
    _autostack << _Wrenches_limits;
    
    _autostack->update(Eigen::VectorXd());
    
    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(_autostack->getStack(), _autostack->getBounds(), 1e6);
    
}


bool forza_giusta::ForceOptimization::compute(const Eigen::VectorXd& fixed_base_torque, 
                                              const std::map<std::string, Eigen::Vector6d>& Fref_ifopt_map,
                                              const std::map<std::string, Eigen::Matrix3d>& RotM_ifopt,
                                              std::map<std::string, Eigen::Vector6d>& Fc_map)
{    
    
    std::vector<Eigen::Vector6d> Fc;
    Fc.resize(_contact_links.size());

    _forza_giusta->setFixedBaseTorque(fixed_base_torque);

    bool all_inactive = true;
    
    Eigen::Matrix6d weight;
    weight.setIdentity();
    
    for(const auto& pair : Fref_ifopt_map)
    {
        auto _wrench = _Wrenches->getWrenchTask(pair.first);
        _wrench->setReference(pair.second);

        if(pair.second.norm() < 1e-12)
        {
            _wrench->setWeight(1e3*weight);

        }
        else
        {
            _wrench->setWeight(weight);
        }

    }
    
    if (all_inactive)
    {
        for(const auto& pair : Fref_ifopt_map)
        {
            auto _wrench = _Wrenches->getWrenchTask(pair.first);
            _wrench->setWeight(weight);
        }
    }


    for(int i = 0; i < _contact_links.size(); i++)
    {
        for(const auto& pair : RotM_ifopt)
        {
            if (_contact_links[i] == pair.first)
            {
                
                auto _friction_cone = _friction_cones->getFrictionCone(_contact_links[i]);
                _friction_cone->setContactRotationMatrix(pair.second);

            }
        }
    }

    _autostack->update(Eigen::VectorXd());
    
    if(!_solver->solve(_x_value))
    {
        std::cout << "SOLVER CAN NOT SOLVE" << std::endl;
        return false;
    }
    
    
    for(int i = 0; i < _contact_links.size(); i++)
    {
        _wrenches[i].getValue(_x_value, Fc_map[_contact_links[i]]);
        
    }
    
    return true;
    
}

bool forza_giusta::ForceOptimization::setConstraints(std::map<std::string, Eigen::VectorXd> constraint_links)
{
    std::vector<Eigen::VectorXd> vec_lowerLims, vec_upperLims;

    //     for (auto const& x : constraint_links)
    //     {
    //         std::cout << x.first
    //                   << ':'
    //                   << x.second.transpose()
    //                   << std::endl ;
    //     }

    for(auto cl : _contact_links)
    {
        Eigen::VectorXd lowerLims(6), upperLims(6);

        if (_map_lowerLims.find(cl) == _map_lowerLims.end())
        {
            lowerLims.setOnes(); lowerLims*= -1e3;
            _map_lowerLims[cl] = lowerLims;
        }

        if (_map_upperLims.find(cl) == _map_upperLims.end())
        {
            upperLims.setOnes(); upperLims*= 1e3;
            _map_upperLims[cl] = upperLims;
        }

        if ( constraint_links.find(cl) == constraint_links.end() )
        {
            /*do nothing*/
        }
        else
        {
            lowerLims = - constraint_links[cl];
            upperLims =   constraint_links[cl];

            _map_lowerLims[cl] = lowerLims;
            _map_upperLims[cl] = upperLims;

            std::cout << "Constraints of link '" << cl <<  "' set to: [" << _map_upperLims[cl].transpose() << "]" <<std::endl;
        }
    }


     for(auto elem : _contact_links)
     {
         vec_lowerLims.push_back(_map_lowerLims[elem]);
         vec_upperLims.push_back(_map_upperLims[elem]);
     }

     for (auto const& x : _map_upperLims)
     {
         std::cout << x.first
                   << ':'
                   << x.second.transpose()
                   << std::endl ;
     }

    _Wrenches_limits = boost::make_shared<OpenSoT::constraints::force::WrenchesLimits>(_contact_links, vec_lowerLims, vec_upperLims, _wrenches);

}


bool forza_giusta::ForceOptimization::resetConstraints(std::vector<std::string> reset_links)
{
    std::vector<Eigen::VectorXd> vec_lowerLims, vec_upperLims;

    for(auto cl : _contact_links)
    {
        Eigen::VectorXd lowerLims(6), upperLims(6);

        if (std::find(reset_links.begin(), reset_links.end(), cl) != reset_links.end())
        {
            lowerLims.setOnes(); lowerLims*= -1e3;
            _map_lowerLims[cl] = lowerLims;
            upperLims.setOnes(); upperLims*= 1e3;
            _map_upperLims[cl] = upperLims;

            std::cout << "Constraints of link '" << cl <<  "' reset to: [" << _map_upperLims[cl].transpose() << "]" <<std::endl;
        }
        else
        {
            /*do nothing*/
        }
    }


    for(auto elem : _contact_links)
    {
        vec_lowerLims.push_back(_map_lowerLims[elem]);
         vec_upperLims.push_back(_map_upperLims[elem]);
     }

     for (auto const& x : _map_upperLims)
     {
         std::cout << x.first
                   << ':'
                   << x.second.transpose()
                   << std::endl ;
     }

    _Wrenches_limits = boost::make_shared<OpenSoT::constraints::force::WrenchesLimits>(_contact_links, vec_lowerLims, vec_upperLims, _wrenches);

}
void forza_giusta::ForceOptimization::log(XBot::MatLogger::Ptr logger)
{
    _autostack->log(logger);
    _solver->log(logger);
}

