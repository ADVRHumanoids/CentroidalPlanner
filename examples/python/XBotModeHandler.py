#!/usr/bin/env python
import xbot_interface.config_options as xbot_opt
import xbot_interface.xbot_interface as xbot
import rospy
from centroidal_planner.srv import setStiffnessDamping
import xbot_stiffness as xbotstiff
import xbot_damping as xbotdamp
import lower_ankle_impedance as lower_ankle_impedance
import numpy as np
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

def is_empty(any_structure):
    if any_structure:
        return False
    else:
        return True

def executeCommand(req) :

    if (req.end_effector == 'l_sole' or req.end_effector == 'r_sole') :

        xbotstiff.set_leg_stiffness(robot, req.end_effector, req.stiffness)
        xbotdamp.set_leg_damping(robot, req.end_effector, req.damping)

    elif (req.end_effector == 'hands') :

        print 'entered set ', req.end_effector
        xbotstiff.set_arms_default_stiffness(robot, req.stiffness)
        xbotdamp.set_arms_default_damping(robot, req.damping)

    elif (req.end_effector == 'l_ankle') :
        lower_ankle_impedance.run(robot, 'l_sole')

    elif (req.end_effector == 'r_ankle') :
        lower_ankle_impedance.run(robot, 'r_sole')


    return True

def get_robot() :

    np.set_printoptions(precision=3, suppress=True)

    # INIT FOR XBOTCORE
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('xbotcore/robot_description')
    srdf = rospy.get_param('xbotcore/robot_description_semantic')

    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_string_parameter('model_type', 'RBDL')
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('framework', 'ROS')

    robot = xbot.RobotInterface(opt)
    model = xbot.ModelInterface(opt)

    return robot, model

if __name__ == '__main__':

    rospy.init_node('xbotcore_mode')
    roscpp_init('xbotcore_mode', [])

    robot, model = get_robot()
    robot.sense()
    model.syncFrom(robot)

    robot.setControlMode(xbot.ControlMode.Stiffness() + xbot.ControlMode.Damping())

    s = rospy.Service('xbotcore_impedance/set_stiffness_damping', setStiffnessDamping, executeCommand)

    rospy.spin()




