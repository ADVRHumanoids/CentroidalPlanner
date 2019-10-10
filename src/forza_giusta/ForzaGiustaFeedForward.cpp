#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <forza_giusta/ForzaGiustaOpt.h>
#include <sensor_msgs/JointState.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosImpl.h>
#include <centroidal_planner/SetLiftedContacts.h>

std::map<std::string, Eigen::Vector6d> * g_fmap_ptr;
std::map<std::string, Eigen::Vector6d> * g_fmap_est_ptr;
std::map<std::string, Eigen::Matrix3d> * g_Rmap_ptr;
Eigen::Vector6d* wrench_manip_ptr;
std::map<std::string, Eigen::Vector6d> * g_fmap_arms_ptr;
Eigen::Matrix3d imu_R_w0;

boost::shared_ptr<forza_giusta::ForceOptimization> force_opt;

void on_joint_pos_recv(const sensor_msgs::JointStateConstPtr& msg, XBot::JointNameMap * jmap)
{
    for(int i = 0; i < msg->name.size(); i++)
    {
        (*jmap)[msg->name.at(i)] = msg->position.at(i);
    }
}

void on_force_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_ptr->at(l));
}

void on_force_est_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_est_ptr->at(l));
}

void on_force_arms_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_arms_ptr->at(l));
}

void on_wrench_recv(const geometry_msgs::WrenchStampedConstPtr& msg)
{
    tf::wrenchMsgToEigen(msg->wrench, wrench_manip_ptr[0]);
}

void on_normal_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
   
    Eigen::Matrix3d R;    
    
    if (msg->wrench.force.x == 0 && msg->wrench.force.y == 0 && msg->wrench.force.z == 1) 
    {      
        R.setIdentity();             
    }     
    
    else
    {

        double _tmp = std::sqrt( (msg->wrench.force.x*msg->wrench.force.x) + (msg->wrench.force.y*msg->wrench.force.y) );

	Eigen::Matrix3d R_tmp;
	
        R_tmp.coeffRef(0, 0) =  msg->wrench.force.y/_tmp;
        R_tmp.coeffRef(0, 1) = -msg->wrench.force.x/_tmp;

        R_tmp.coeffRef(1, 0) = (msg->wrench.force.x * msg->wrench.force.z)/_tmp;
        R_tmp.coeffRef(1, 1) = (msg->wrench.force.y * msg->wrench.force.z)/_tmp;
        R_tmp.coeffRef(1, 2) = -_tmp;

        R_tmp.coeffRef(2, 0) =  msg->wrench.force.x;
        R_tmp.coeffRef(2, 1) =  msg->wrench.force.y;
        R_tmp.coeffRef(2, 2) =  msg->wrench.force.z;
	
	R = R_tmp.transpose();
    }
    
    g_Rmap_ptr->at(l) = R; 
        
}

class item_sender
{
public:
    item_sender() {_isSending = false;}

    bool send_wrenches(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
    {
        if (req.data)
        {
            _isSending = true;
            std::cout << "START received. Starting ..." << std::endl;
            res.message = "Sending wrenches";
            res.success = true;
        }
        else
        {
            _isSending = false;
            std::cout << "STOP received. Stopping ..." << std::endl;
            res.message = "Stopping wrenches";
            res.success = false;
        }
        return true;
    }

    bool isSending() {return _isSending;}

private :
    bool _isSending;
};


bool setLiftedContacts(centroidal_planner::SetLiftedContactsRequest& req, centroidal_planner::SetLiftedContactsResponse& res)
{

    Eigen::Vector6d default_constraints;
    default_constraints << 1000, 1000, 1000, 0, 0, 0;
    std::map<std::string, Eigen::VectorXd> constraint_links;

    /* generate constraint on selected link */
    for (auto const& value : req.link)
    {
        constraint_links[value] = default_constraints;
    }

    /* SET CONSTRAINTS */
    if (force_opt)
    {
        force_opt->setConstraints(constraint_links);

        std::string output = "";
        for (auto it = constraint_links.cbegin(); it != constraint_links.cend(); it++)
        {
            output += (it->first) + ", ";
        }
        res.message = "Successfully lifted link: " + output.substr(0, output.size() - 2 );
    }
    res.success = true;
    return true;
}

bool resetLiftedContacts(centroidal_planner::SetLiftedContactsRequest& req, centroidal_planner::SetLiftedContactsResponse& res)
{

    /* SET CONSTRAINTS */
    if (force_opt)
    {
        force_opt->resetConstraints(req.link);

        std::string output = "";
        for (auto it : req.link)
        {
            output += (it) + ", ";
        }
        res.message = "Successfully resetted link: " + output.substr(0, output.size() - 2 );
    }
    res.success = true;
    return true;
}

int main(int argc, char ** argv)
{
    
    ros::init(argc, argv, "forza_giusta_node");
    ros::NodeHandle nh("forza_giusta");
    ros::NodeHandle nh_priv("~");

    ros::NodeHandle nh_xbotcore("xbotcore");

    auto cfg = XBot::ConfigOptionsFromParamServer(nh_xbotcore);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    auto model = XBot::ModelInterface::getModel(cfg);

    auto imu = robot->getImu().begin()->second;
    robot->sense(false);
    Eigen::Matrix3d tmp;
    imu->getOrientation(tmp);
    imu_R_w0 = tmp.transpose();
    
    XBot::JointNameMap jmap;
    robot->getMotorPosition(jmap);
        
    XBot::Cartesian::RosImpl ci;
      
    auto j_sub = ros::NodeHandle("cartesian").subscribe<sensor_msgs::JointState>("solution", 1, 
                            std::bind(on_joint_pos_recv, std::placeholders::_1, &jmap)
                        );
    
    robot->setControlMode(XBot::ControlMode::Effort());
    
    double rate = nh_priv.param("rate", 100.0);
    double mu = nh_priv.param("mu", 0.5);
    auto links = nh_priv.param("links", std::vector<std::string>());

    /* BLACKLISTED JOINTS */    
    auto blacklist = nh_priv.param("blacklist", std::vector<std::string>());
    
    
    std::map<std::string, XBot::ControlMode> ctrl_map;
    for(auto j : blacklist)
    {
        if(!robot->hasJoint(j))
        {
            if(!robot->hasChain(j))
            {
                throw std::runtime_error("Joint or chain '" + j + "' undefined");
            }
            else
            {
                for(auto jc: robot->chain(j).getJointNames())
                {
                    ROS_INFO("Joint '%s' is blacklisted", jc.c_str());
                    ctrl_map[jc] = XBot::ControlMode::Idle();
                }
            }
            
        }
        
        ROS_INFO("Joint '%s' is blacklisted", j.c_str());
        ctrl_map[j] = XBot::ControlMode::Idle();
    }
    robot->setControlMode(ctrl_map);
    
    /* TORQUE OFFSET */  
    auto tau_off_map = nh_priv.param("torque_offset", std::map<std::string, double>());
    XBot::JointNameMap tau_off_map_xbot(tau_off_map.begin(), tau_off_map.end());
    Eigen::VectorXd tau_offset;
    tau_offset.setZero(model->getJointNum());
    model->mapToEigen(tau_off_map_xbot, tau_offset);
    std::cout << "Torque offset: " << tau_offset.transpose() << std::endl;
    
    std::map<std::string, ros::Subscriber> sub_wrench_map;
    std::map<std::string, ros::Subscriber> sub_n_map;
    std::map<std::string, Eigen::Vector6d> w_ref_map;
    std::map<std::string, Eigen::Matrix3d> RotM_map;
    std::map<std::string, Eigen::Vector6d> w_ForzaGiusta_map;

    item_sender sender;
    ros::ServiceServer switch_srv;
    switch_srv = nh.advertiseService("send_forces/", &item_sender::send_wrenches, &sender);

    ros::ServiceServer lifted_contacts_srv;
    lifted_contacts_srv = nh.advertiseService("set_lifted_contacts/", setLiftedContacts);

    ros::ServiceServer reset_contacts_srv;
    reset_contacts_srv = nh.advertiseService("reset_lifted_contacts/", resetLiftedContacts);

    for(auto l : links)
    {
        auto sub_force = nh.subscribe<geometry_msgs::WrenchStamped>("force_ref/" + l,
								    1, 
								    boost::bind(on_force_recv, _1, l));
        
        auto sub_n = nh.subscribe<geometry_msgs::WrenchStamped>("normal/" + l,
                                                                1, 
                                                                boost::bind(on_normal_recv, _1, l));
        
        sub_wrench_map[l] = sub_force;
        w_ref_map[l] = Eigen::Vector6d::Zero();
        w_ForzaGiusta_map[l] = Eigen::Vector6d::Zero();
        sub_n_map[l] = sub_n;
        RotM_map[l] =  Eigen::Matrix3d::Identity();
                
        ROS_INFO("Subscribed to topic '%s'", sub_force.getTopic().c_str());
        ROS_INFO("Subscribed to topic '%s'", sub_n.getTopic().c_str());
    }
    
       
    std::vector<std::string> arms  = {""};
    
    std::map<std::string, ros::Subscriber> sub_force_arms_map;
    std::map<std::string, Eigen::Vector6d> f_ref_arms_map;
    
    for(auto i : arms)
    {
	auto sub_force_arms = nh.subscribe<geometry_msgs::WrenchStamped>("force_arms/" + i,
									 1, 
									 boost::bind(on_force_arms_recv, _1, i));
	
	sub_force_arms_map[i] = sub_force_arms;
        f_ref_arms_map[i] = Eigen::Vector6d::Zero();	

    }
    
    std::vector<std::string> force_links  = {"l_sole", "r_sole", "l_ball_tip", "r_ball_tip"};
    
    std::map<std::string, ros::Subscriber> sub_wrench_est_map;
    std::map<std::string, Eigen::Vector6d> w_est_map;
    
    for(auto j : force_links)
    {
        auto sub_force_est = ros::NodeHandle("cartesian").subscribe<geometry_msgs::WrenchStamped>("force_estimation/" + j, 1, boost::bind(on_force_est_recv, _1, j));	
    sub_wrench_est_map[j] = sub_force_est;
    w_est_map[j] = Eigen::Vector6d::Zero();
	
	ROS_INFO("Subscribed to topic '%s'", sub_force_est.getTopic().c_str());
	
    }
    
    g_fmap_est_ptr = &w_est_map;
       
     
    auto sub_wrench_manip = nh.subscribe<geometry_msgs::WrenchStamped>("wrench_manip/",
                                                                       1, 
                                                                       boost::bind(on_wrench_recv, _1));
    
    Eigen::Vector6d wrench_manip; 
    wrench_manip.setZero();
    
    ROS_INFO("Subscribed to topic '%s'", sub_wrench_manip.getTopic().c_str());
    

    
    g_fmap_ptr = &w_ref_map;
    g_Rmap_ptr = &RotM_map;
    wrench_manip_ptr = &wrench_manip;
    g_fmap_arms_ptr = &f_ref_arms_map;
    

    Eigen::VectorXd lim_l_hand(6), lim_r_hand(6);

    lim_l_hand << 1000, 1000, 1000, 0, 0, 0;
    lim_r_hand << 1000, 1000, 1000, 0, 0, 0;

//    constraint_links["l_ball_tip"] = lim_l_hand;
//    constraint_links["r_ball_tip"] = lim_r_hand;

    force_opt = boost::make_shared<forza_giusta::ForceOptimization>(model, links, mu, true); // true is for optimize also wrenches
    
    Eigen::VectorXd tau;
    ros::Rate loop_rate(rate);
    
    bool log;
    nh_priv.param("log", log, false);
    
    XBot::MatLogger::Ptr logger;
    std::stringstream ss;
    ss << "/tmp/forza_giusta_node";

    if (log)
        logger = XBot::MatLogger::getLogger(ss.str());
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        /* Sense robot state and update model */
        robot->sense(false);
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        model->setFloatingBaseState(imu,imu_R_w0);
        model->update();
        
        Eigen::Affine3d fb_pose;
        model->getFloatingBasePose(fb_pose);

        /* Compute gcomp */
        model->computeGravityCompensation(tau);
        tau -= tau_offset;
        tau.head(6) += wrench_manip;

        force_opt->compute(tau, w_ref_map, RotM_map, w_ForzaGiusta_map);
        
        force_opt->log(logger);

        for(const auto& pair : w_ref_map)
        {
            Eigen::Vector6d f_world = pair.second;
            
            if (log)
            {
                logger->add("F_ifopt_" + pair.first, f_world);
                for (auto pair : RotM_map)
                {
                    logger->add("RotM_map_" + pair.first, pair.second);
                }
            }
        }
        


        for(const auto& pair : w_ForzaGiusta_map)
        {

            Eigen::Vector6d w_world = pair.second;

//            std::cout << "W_" + pair.first + ": " << w_world.head(6).transpose() << std::endl;

            if (log)
            {
                logger->add("W_" + pair.first, w_world);

            }
            
            Eigen::MatrixXd J;
            model->getJacobian(pair.first, J);

            tau -= J.transpose() * w_world;
            
        }
        

        //	for (const auto& pair : f_ref_arms_map)
        //	{
//        Eigen::Vector6d f_world = pair.second;
//	    f_world.tail(3).setZero();
	    
//	    if (log)
//            {
//                logger->add("F_" + pair.first, f_world);
                                      
//            }
                 
//        Eigen::MatrixXd J;
//        model->getJacobian(pair.first, J);

//        tau -= J.transpose() * f_world;
//	}
	
//    for(const auto& pair : w_est_map)
//        {
	   
//	   Eigen::Affine3d pose;
//	   model->getPose(pair.first, pose);
	   
//	   Eigen::Vector3d f_est_world;
//	   f_est_world = pose.linear() * pair.second.head(3);
	   
//	   logger->add("F_est_" + pair.first, f_est_world);
	   
//	}
        
        if (sender.isSending())
        {
            /* Send torque to joints */
            model->setJointEffort(tau);
            robot->setReferenceFrom(*model, XBot::Sync::Effort);
            robot->move();
        }
        
        loop_rate.sleep();
    }
    
    if (log)
        logger->flush();
    
    return 0;
    
}
