#include <ros/ros.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/problem/Com.h>
#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/ros/RosImpl.h>

using namespace XBot;
using namespace XBot::Cartesian;

XBot::MatLogger::Ptr logger;

int main(int argc, char **argv)
{

    /* Init ROS node */
    ros::init(argc, argv, "centauro_replay_node");
    ros::NodeHandle nh("centauro_replay_node");
    ros::NodeHandle nh_priv("~");

//    XBot::Cartesian::RosImpl ci;

    double rate = nh_priv.param("rate", 100.0);

    XmlRpc::XmlRpcValue param_list;
//    nh_priv.getParam("ext_w_init", param_list);

    ros::Rate loop_rate(rate);

    ConfigOptions config = XBot::ConfigOptionsFromParamServer();

//    auto robot = XBot::RobotInterface::getRobot(config);
//    robot->sense();

//    auto model = ModelInterface::getModel(config);
//    model->update();

    bool log;
    nh_priv.param("log", log, false);

    while (ros::ok()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    if (log)
      logger->flush();
    
    return 0;
}

