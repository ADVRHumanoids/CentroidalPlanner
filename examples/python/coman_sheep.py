#!/usr/bin/env python
from cartesian_interface.pyci_all import *
import centroidal_planner.pycpl as cpl
import numpy as np
import xbot_interface.config_options as xbot_opt
import xbot_interface.xbot_interface as xbot
import rospy
import centroidal_planner.pyforcepub as fp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

import xbot_stiffness as xbotstiff
import xbot_damping as xbotdamp
import homing as homing
import opt_sheep as opt_pos_sheep
import opt_wall as opt_pos_wall
import hand_positioning as hand_cmd
import foot_positioning_1 as foot_cmd
import surface_reacher as surface_reacher
import matlogger2.matlogger as matlog

from geometry_msgs.msg import *

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

def sensors_init(arm_estimation_flag, f_est) :


    ft_map = robot.getForceTorque()

    if (arm_estimation_flag) :
        # create force estimator
        indices_wrench = [0,1,2]
        ft_map['l_arm_ft'] = f_est.addLink('l_ball_tip', indices_wrench, ['left_arm'])
        ft_map['r_arm_ft'] = f_est.addLink('r_ball_tip', indices_wrench, ['right_arm'])
        f_est.update()

    return ft_map


def gen_com_planner(contacts, mass, mu) :

    com_pl = cpl.CoMPlanner(contacts, mass)

    # WEIGHTS
    com_pl.SetMu(mu)
    com_pl.SetCoMWeight(1000000000000)  # 100000.0
    com_pl.SetForceWeight(0.)  # 0.0000001
    return com_pl


def advance_com(ci, x_pos, reach_time) :

    print "advancing with com..."
    com_advance = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    com_advance[0] += x_pos
    com_ci = Affine3(pos=com_advance)
    ci.setTargetPose('com', com_ci, reach_time)
    ci.waitReachCompleted('com')
    ci.update()
    print "CoM at: ", ci.getPoseFromTf('ci/com', 'ci/world_odom').translation


def position_com(ci, x_pos, reach_time) :

    print "advancing with com..."
    com_advance = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    com_advance[0] = x_pos
    com_ci = Affine3(pos=com_advance)
    ci.setTargetPose('com', com_ci, reach_time)
    ci.waitReachCompleted('com')
    ci.update()
    print "CoM at: ", ci.getPoseFromTf('ci/com', 'ci/world_odom').translation

def lower_waist(ci, z_pos, reach_time) :

    print "lowering waist..."
    ci.setControlMode('Waist', pyci.ControlType.Position)
    waist_pos = ci.getPoseReference('Waist')[0].translation
    waist_pos[2] -= z_pos
    waist_ci = Affine3(pos=waist_pos)
    ci.setTargetPose('Waist', waist_ci, reach_time)
    ci.waitReachCompleted('Waist')
    ci.update()
    print "Waist at: ", ci.getPoseReference('Waist')[0].translation

if __name__ == '__main__':

    np.set_printoptions(precision=3, suppress=True)

    rospy.init_node('coman_sheep_commander')
    roscpp_init('coman_sheep', [])
    # define contacts for the ForcePublisher
    #select contacts of COMAN+
    feet_list = ['l_sole', 'r_sole']
    hands_list = ['l_ball_tip', 'r_ball_tip']
    contact_joints = feet_list + hands_list
    print contact_joints

    logger = matlog.MatLogger2('/tmp/feet_on_wall')
    logger.setBufferMode(matlog.BufferMode.CircularBuffer)

    robot, model = get_robot()
    robot.sense()
    model.syncFrom(robot)

    # get cartesio ros client
    ci = pyci.CartesianInterfaceRos()
    mass = 70.0

    #height of ground
    world_odom_T_world = (ci.getPoseFromTf('ci/com', 'ci/world_odom').translation)[2]
    print('world_odom_T_world is :', world_odom_T_world)
    # distance hands from feet
    dist_hands = np.array([0.5, 0.1])
    # size of the bound region for solver
    bound_size = 0.25

    f_est = pyest.ForceEstimation(model, 0.05) # 0.05 treshold

    ft_map = sensors_init(arm_estimation_flag=True, f_est=f_est)

    mu_com_pl = 0.5
    com_pl = gen_com_planner(contact_joints, mass, mu_com_pl)

    # SET HOMING POSIITION
    # homing.run(ci)
    # COMPUTE OPTIMAL POSE FOR SHEEP CONFIGURATION
    ctrl_pl_sheep, sol_centroidal_sheep = opt_pos_sheep.compute(ci, contact_joints, hands_list, feet_list, mass, dist_hands, bound_size)
    # REACH WITH HANDS AND MOVE COM

    # logger.add("forces_crounching", sol_centroidal_sheep)

    ci.setControlMode('Waist', pyci.ControlType.Disabled)
    ci.setControlMode('torso', pyci.ControlType.Disabled)

    # MOVE HANDS DOWN
    hand_cmd.run(robot, ft_map, ci, hands_list, sol_centroidal_sheep)

    ci.setControlMode('torso', pyci.ControlType.Position)

    # REACH WITH HANDS THE BRICKS
    surface_reacher.run_hand(ci, robot, model, ft_map, hands_list, f_est, logger)

    #MOVE COM
    position_com(ci, sol_centroidal_sheep.com[0] - 0.08, 15.)     #reach_time = 15.

    print "waiting for forza_giusta node ...."
    forcepub = fp.ForcePublisher(contact_joints)
    print "connected to forza_giusta."

    # SET FEET POINT CONTACT
    forcepub.setPointContacts(feet_list)
    # SET HANDS POINT CONTACT
    forcepub.setPointContacts(hands_list)


    forces_sheep = [sol_centroidal_sheep.contact_values_map[feet_list[0]].force[0],
                    sol_centroidal_sheep.contact_values_map[feet_list[0]].force[1],
                    sol_centroidal_sheep.contact_values_map[feet_list[0]].force[2],
                    sol_centroidal_sheep.contact_values_map[feet_list[1]].force[0],
                    sol_centroidal_sheep.contact_values_map[feet_list[1]].force[1],
                    sol_centroidal_sheep.contact_values_map[feet_list[1]].force[2],
                    sol_centroidal_sheep.contact_values_map[hands_list[0]].force[0],
                    sol_centroidal_sheep.contact_values_map[hands_list[0]].force[1],
                    sol_centroidal_sheep.contact_values_map[hands_list[0]].force[2],
                    sol_centroidal_sheep.contact_values_map[hands_list[1]].force[0],
                    sol_centroidal_sheep.contact_values_map[hands_list[1]].force[1],
                    sol_centroidal_sheep.contact_values_map[hands_list[1]].force[2]]

    normal_sheep = [sol_centroidal_sheep.contact_values_map[feet_list[0]].normal[0],
                    sol_centroidal_sheep.contact_values_map[feet_list[0]].normal[1],
                    sol_centroidal_sheep.contact_values_map[feet_list[0]].normal[2],
                    sol_centroidal_sheep.contact_values_map[feet_list[1]].normal[0],
                    sol_centroidal_sheep.contact_values_map[feet_list[1]].normal[1],
                    sol_centroidal_sheep.contact_values_map[feet_list[1]].normal[2],
                    sol_centroidal_sheep.contact_values_map[hands_list[0]].normal[0],
                    sol_centroidal_sheep.contact_values_map[hands_list[0]].normal[1],
                    sol_centroidal_sheep.contact_values_map[hands_list[0]].normal[2],
                    sol_centroidal_sheep.contact_values_map[hands_list[1]].normal[0],
                    sol_centroidal_sheep.contact_values_map[hands_list[1]].normal[1],
                    sol_centroidal_sheep.contact_values_map[hands_list[1]].normal[2]]

    print "contact_joints: ", contact_joints
    print "forces are: ", forces_sheep
    print "normals are: ", normal_sheep

    # sending forces ..
    forcepub.sendForce(contact_joints, forces_sheep)
    forcepub.sendNormal(contact_joints, normal_sheep)

    # SWITCH ON FORZA GIUSTA
    raw_input("Press Enter to switch ON forza giusta and lower the impedance.")

    forcepub.switchController(True)
    print "Switched ON forza_giusta"

    # PREPARE FOR FORCE CONTROL
    default_stiffness_leg = 1000 #1500
    default_damping_leg = 10

    default_stiffness_arm = 1000 #1500
    default_damping_arm = 10

    default_stiffness_wrist = 500

    # TO AVOID THE HIP TO REACH JOINT LIMITS
    # default_stiffness_hip_pitch = 800

    xbotstiff.set_legs_default_stiffness(robot, [default_stiffness_leg, default_stiffness_leg, default_stiffness_leg, default_stiffness_leg, default_stiffness_leg, default_stiffness_leg])
    xbotdamp.set_legs_default_damping(robot, [default_damping_leg, default_damping_leg, default_damping_leg, default_damping_leg, default_damping_leg, default_damping_leg])

    xbotstiff.set_arms_default_stiffness(robot, [default_stiffness_arm, default_stiffness_arm, default_stiffness_arm, default_stiffness_arm, default_stiffness_wrist, default_stiffness_wrist, default_stiffness_wrist])
    xbotdamp.set_arms_default_damping(robot, [default_damping_arm, default_damping_arm, default_damping_arm, default_damping_arm, default_damping_arm, default_damping_arm, default_damping_arm])

    raw_input("Press Enter to advance com.")
    advance_com(ci, 0.06, 5.)

    # raw_input("Press Enter to lower waist.")
    # lower_waist(ci, 0.05, 10)


    # COMPUTE OPTIMAL POSE FOR WALL CONFIGURATION
    mu_feet = 0.5
    ctrl_pl_wall, sol_centroidal_wall = opt_pos_wall.compute(ci, contact_joints, hands_list, feet_list, mass, mu_feet)

    # logger.add("forces_pos wall", sol_centroidal_wall)
    # REACH WALL WITH FEET
    foot_cmd.run(robot, ft_map, ci, ctrl_pl_wall, contact_joints, hands_list, feet_list, sol_centroidal_wall, com_pl, forcepub, world_odom_T_world, logger)

    # ci.setControlMode('Waist', pyci.ControlType.Position)
    # waist_pos = ci.getPoseReference('Waist')[0].translation
    # waist_pos[2] += 0.1
    # waist_ci = Affine3(pos=waist_pos)
    # ci.setTargetPose('Waist', waist_ci, 5)

    del logger

    exit(0)


