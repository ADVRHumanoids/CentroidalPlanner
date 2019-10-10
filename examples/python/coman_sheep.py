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
from geometry_msgs.msg import *

def get_robot() :

    np.set_printoptions(precision=3, suppress=True)

    # INIT FOR XBOTCORE
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('robot_description')
    srdf = rospy.get_param('robot_description_semantic')

    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_string_parameter('model_type', 'RBDL')
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('framework', 'ROS')

    robot = xbot.RobotInterface(opt)

    return robot

def sensors_init() :

    ft_map = robot.getForceTorque()

    return ft_map

def homing(ci) :

    foot_one = ci.getPoseReference(feet_list[0])[0].translation
    foot_two = ci.getPoseReference(feet_list[1])[0].translation
    foot_ci_one = Affine3(pos=foot_one)
    foot_ci_two = Affine3(pos=foot_two)

    ci.setTargetPose(feet_list[0], foot_ci_one, 5)
    ci.setTargetPose(feet_list[1], foot_ci_two, 5)

    ci.waitReachCompleted(feet_list[0])
    ci.waitReachCompleted(feet_list[1])
    ci.update()

    ci.setControlMode('Waist', pyci.ControlType.Disabled)

def rotation(theta):

    tx, ty, tz = theta

    Rx = np.array([[1, 0, 0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
    Ry = np.array([[np.cos(ty), 0, -np.sin(ty)], [0, 1, 0], [np.sin(ty), 0, np.cos(ty)]])
    Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0, 0, 1]])

    return np.dot(Rx, np.dot(Ry, Rz))

def init_param():

    # get cartesio ros client
    ci = pyci.CartesianInterfaceRos()
    #select contacts of COMAN+
    feet_list = ['l_sole', 'r_sole']
    hands_list = ['l_ball_tip', 'r_ball_tip']
    contacts = feet_list + hands_list

    mass = 70.0

    #height of ground
    world_odom_T_world = (ci.getPoseFromTf('ci/com', 'ci/world_odom').translation)[2]
    print('world_odom_T_world is :', world_odom_T_world)
    # distance hands from feet
    dist_hands = np.array([0.5, 0.1])
    # size of the bound region for solver
    bound_size = 0.25

    return ci, feet_list, hands_list, contacts, mass, world_odom_T_world, dist_hands, bound_size


def optimal_pos_sheep(ci, contacts, hands_list, feet_list, mass, dist_hands, bound_size) :

    env = cpl.Ground()
    env.SetGroundZ(ci.getPoseReference('l_sole')[0].translation[2])

    ctrl_pl = cpl.CentroidalPlanner(contacts, mass, env)

    # ctrl_pl.SetCoMWeight(100000)
    # ctrl_pl.SetPosWeight(100000000)
    # ctrl_pl.SetForceWeight(0)

    # set position reference and bounds for FEET
    for foot in feet_list :
        contact_foot = ci.getPoseReference(foot)[0].translation
        # contact_foot[2] = 0.
        ctrl_pl.SetPosRef(foot, contact_foot)
        print "Setting contact position for",  foot, " to ", ctrl_pl.GetPosRef(foot)
        lower_bound = ctrl_pl.GetPosRef(foot) - bound_size
        upper_bound = ctrl_pl.GetPosRef(foot) + bound_size
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound
        ctrl_pl.SetPositionBound(foot, lower_bound, upper_bound)

    # set position reference and bounds for HANDS
    i = 0
    for hand in hands_list :
        contact_hand = ci.getPoseReference(feet_list[i])[0].translation
        # contact_hand[2] = 0.
        contact_hand[0] += dist_hands[0]
        if hand == "l_ball_tip":
            contact_hand[1] += dist_hands[1]
        else:
            contact_hand[1] -= dist_hands[1]

        i = i+1
        ctrl_pl.SetPosRef(hand, contact_hand)
        print "Setting contact position for",  hand, " to ", ctrl_pl.GetPosRef(hand)
        lower_bound = ctrl_pl.GetPosRef(hand) - bound_size
        upper_bound = ctrl_pl.GetPosRef(hand) + bound_size
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound
        ctrl_pl.SetPositionBound(hand, lower_bound, upper_bound)

    sol_centroidal = ctrl_pl.Solve()

    print sol_centroidal

    return ctrl_pl, sol_centroidal

    # print list(sol.contact_values_map)

def gen_com_planner(contacts, mass) :

    com_pl = cpl.CoMPlanner(contacts, mass)

    # WEIGHTS
    mu = 0.5
    com_pl.SetMu(mu)
    com_pl.SetCoMWeight(1000000000000)  # 100000.0
    com_pl.SetForceWeight(0.)  # 0.0000001

    return com_pl

def foot_positioning(ci, ctrl_pl, contacts, hands_list, feet_list, mass, sol_centroidal, com_pl) :

    # HANDS WRT WAIST
    for hand_i in hands_list:
        ci.setBaseLink(hand_i, 'Waist')

    # WAIST DISABLED
    ci.setControlMode('Waist', pyci.ControlType.Disabled)

    # SET THRESHOLD FOR FEET
    for c in feet_list :
        com_pl.SetForceThreshold(c, 50.0)

    # SET FEET CONTACTS
    for c_f in feet_list :
        # contact_pos = ctrl_pl.GetPosRef(c_f) # should be the same
        contact_pos = ci.getPoseReference(c_f)[0].translation
        com_pl.SetContactPosition(c_f, contact_pos)
        print("Setting contact position for ", c_f, " to ", com_pl.GetContactPosition(c_f))

    # SET HANDS CONTACTS
    for c_h in hands_list :
        # contact_pos = ctrl_pl.GetPosRef(c_h)
        contact_pos = sol_centroidal.contact_values_map[c_h].position
        com_pl.SetContactPosition(c_h, contact_pos)
        print("Setting contact position for ", c_h, " to ", com_pl.GetContactPosition(c_h))

    # SET COM
    com_ref = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    print 'Setting com to: ', com_ref
    com_pl.SetCoMRef(com_ref)

    # lift hands, they are not in contact
    for hands_i in hands_list:
        com_pl.SetLiftingContact(hands_i)
        print 'LIFTING ', hands_i

    print("Starting foot placing...")

    # putting feet in the right position
    for foot_i in feet_list :

        com_pl.SetLiftingContact(foot_i)
        print 'LIFTING ', foot_i

        # find solution
        sol = com_pl.Solve()
        print(sol)

        com_pl.ResetLiftingContact(foot_i)
        print 'RESETTING LIFT ', foot_i
        # send commands to cartesio

        # move com
        com_disp = [sol.com[0], sol.com[1], world_odom_T_world]
        print(com_disp)
        com_ci = Affine3(pos=com_disp)
        reach_time = 2.0
        ci.setTargetPose('com', com_ci, reach_time)
        ci.waitReachCompleted('com')
        ci.update()

        # lift sole
        sole_ci = ci.getPoseReference(foot_i)[0]
        sole_ci.translation_ref()[2] += 0.05
        ci.setTargetPose(foot_i, sole_ci, reach_time/2.0)
        ci.waitReachCompleted(foot_i)
        ci.update()

        # move foot
        foot_ci = Affine3(pos=sol_centroidal.contact_values_map[foot_i].position)
        ci.setTargetPose(foot_i, foot_ci, reach_time/2.0)
        ci.waitReachCompleted(foot_i)
        ci.update()


        # get contacts references from cartesio and set them to the planner
        for c in feet_list:
            contact_pos = ci.getPoseReference(c)[0].translation
            com_pl.SetContactPosition(c, contact_pos)
            print("Setting contact position for ", c, " to ", com_pl.GetContactPosition(c))


    # PUT BACK COM IN THE MIDDLE
    for c_f in feet_list :
        contact_pos = ctrl_pl.GetPosRef(c_f)
        com_pl.SetContactPosition(c_f, contact_pos)
        print("Setting contact position for ", c_f, " to ", com_pl.GetContactPosition(c_f))

    sol = com_pl.Solve()

    com_disp = [sol.com[0], sol.com[1], world_odom_T_world]
    com_ci = Affine3(pos=com_disp)
    reach_time = 2.0
    ci.setTargetPose('com', com_ci, reach_time)
    ci.waitReachCompleted('com')
    ci.update()

def hand_positioning(robot, ft_map, ci, ctrl_pl, contacts, hands_list, feet_list, mass, sol_centroidal, com_pl, forcepub) :
    # ==================================================================================================================
    # SET HANDS WRT WORLD
    for hand_i in hands_list:
        ci.setBaseLink(hand_i, 'world')

    # for FEEDBACK CONTACT
    stop_before_contact = 0.05

    # PLACE HANDS ON THE GROUND
    reach_time_hand = 20
    dist_from_ground = 0.15 + stop_before_contact


    hand_one = [sol_centroidal.contact_values_map[hands_list[0]].position[0], sol_centroidal.contact_values_map[hands_list[0]].position[1], sol_centroidal.contact_values_map[hands_list[0]].position[2]+dist_from_ground]
    hand_two = [sol_centroidal.contact_values_map[hands_list[1]].position[0], sol_centroidal.contact_values_map[hands_list[1]].position[1], sol_centroidal.contact_values_map[hands_list[1]].position[2]+dist_from_ground]

    hand_one = Affine3(pos=hand_one)
    hand_two = Affine3(pos=hand_two)

    ci.setTargetPose(hands_list[0], hand_one, reach_time_hand / 2.0)
    ci.setTargetPose(hands_list[1], hand_two, reach_time_hand / 2.0)

    # SEND COMMAND WITH CONTACT DETECTION
    contact_flag_l = 0
    contact_flag_r = 0
    force_threshold = 50
    direction = 2
    while not contact_flag_l or not contact_flag_r :


        contact_flag_l = ci.waitReachCompleted(hands_list[0], 0.001)
        if force_detection(robot, ft_map['l_arm_ft'], direction, force_threshold) :
            print hands_list[0], ': contact detected, stopping task.'
            contact_flag_l = 1


        contact_flag_r = ci.waitReachCompleted(hands_list[1], 0.001)
        if force_detection(robot, ft_map['r_arm_ft'], direction, force_threshold) :
            print hands_list[1], ': contact detected, stopping task.'
            contact_flag_r = 1

        robot.sense()


    print hands_list[0], ': task finished.'
    print hands_list[1], ': task finished.'
    ci.update()


def surface_reacher(robot, ft_map) :

    # velocity desired
    vel_hands = [0, 0, -0.03, 0, 0, 0]


    ci.setControlMode(hands_list[0], pyci.ControlType.Velocity)
    ci.setControlMode(hands_list[1], pyci.ControlType.Velocity)


    contact_treshold = 70
    direction = 2

    while not force_detection(robot, ft_map['l_arm_ft'], direction, contact_treshold) or not force_detection(robot, ft_map['r_arm_ft'], direction, contact_treshold) :

        ci.setVelocityReference(hands_list[0], vel_hands)
        ci.setVelocityReference(hands_list[1], vel_hands)

        if force_detection(robot, ft_map['l_arm_ft'], direction, contact_treshold) :
            print 'l_arm contact'

        if force_detection(robot, ft_map['r_arm_ft'], direction, contact_treshold) :
            print 'r_arm contact'

        robot.sense()

    ci.setControlMode(hands_list[0], pyci.ControlType.Position)
    ci.setControlMode(hands_list[1], pyci.ControlType.Position)

    ci.update()

def surface_reacher_foot(robot, ft_map, end_effector) :

    print('starting surface reacher for: ', end_effector)
    # velocity desired
    vel_foot = [-0.03, 0, 0, 0, 0, 0]

    if end_effector == 'l_sole':
        ft = ft_map['l_leg_ft']
    elif end_effector == 'r_sole':
        ft = ft_map['r_leg_ft']
    else:
        print 'WRONG FOOT'

    ci.setControlMode(end_effector, pyci.ControlType.Velocity)


    contact_treshold = 200
    direction = 2

    contact_sensed = False
    n_cycle = 0

    while not contact_sensed :
        contact_sensed = False
        ci.setVelocityReference(end_effector, vel_foot)

        if force_detection(robot, ft, direction, contact_treshold) :
            print end_effector, ': waiting..'
            n_cycle += 1
        else :
            n_cycle = 0

        if n_cycle > 10 :
            contact_sensed = True
            print end_effector, ': Contact sensed.'
            n_cycle = 0



        robot.sense()

    ci.update()

    ci.setControlMode(end_effector, pyci.ControlType.Position)

def force_detection(robot, ft, direction, magnitude) :


    detect_bool = 0
    wrench = ft.getWrench()

    if (wrench[direction] >= magnitude) :
        detect_bool = 1

    return detect_bool


def advance_com(ci) :

    print "advancing with com..."
    com_advance = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    com_advance[0] = 0.1
    com_ci = Affine3(pos=com_advance)
    reach_time = 4.0
    ci.setTargetPose('com', com_ci, reach_time)
    ci.waitReachCompleted('com')
    ci.update()
    print "CoM at: ", ci.getPoseFromTf('ci/com', 'ci/world_odom').translation

def optimal_pos_wall(ci, contacts, hands_list, feet_list, mass) :


    print "starting foot positioning on wall .."

    # get x position of feet
    contact_foot_x = ci.getPoseReference(feet_list[0])[0].translation[0]
    # height of hand
    contact_hand_z = ci.getPoseReference(hands_list[0])[0].translation[2]

    # point from which the superquadric is computed
    p = [contact_foot_x, 0, contact_hand_z]   # point in between the two soles, at height of hands
    print "p: ", p

    # distance of wall from p
    wall_distance = 0.7 #x

    # distance of ground from p
    ground_distance = 0.0 #z

    # p w.r.t superquadric
    d = [wall_distance, 0.0,  ground_distance]
    print "d: ", d

    # height from superquadricd
    height_contact = 0.4

    # set bounds
    bound_size_wall = 0.1
    bound_size_hands = 0.1

    superquadric_env = cpl.Superquadric()
    mu = 0.5
    superquadric_env.SetMu(mu)

    C_translation = [10, 0, 10]

    radius = [10, 10, 10]

    C = np.array([p[0] + C_translation[0], p[1] + C_translation[1], p[2] + C_translation[2]])  # center of superquadric
    R = np.array([d[0] + radius[0], d[1] + radius[1], d[2] + radius[2]]) # radius of superquadric
    P = np.array([50.0, 50.0, 50.0])

    superquadric_env.SetParameters(C,R,P)

    print "C: ", C
    print "R: ", R

    # create a centroidal planner
    ctrl_pl = cpl.CentroidalPlanner(contacts, mass, superquadric_env)

    # set hands in world
    for hand_i in hands_list:
        ci.setBaseLink(hand_i, 'world')

    # ctrl_pl.SetCoMWeight(100000)
    # ctrl_pl.SetPosWeight(100000000)
    # ctrl_pl.SetForceWeight(0)

    # set position reference and bounds for HANDS
    for hand in hands_list:
        contact_hand = ci.getPoseReference(hand)[0].translation
        ctrl_pl.SetPosRef(hand, contact_hand)

        print "Setting contact position for", hand, " to ", ctrl_pl.GetPosRef(hand)
        lower_bound = ctrl_pl.GetPosRef(hand) - bound_size_hands
        upper_bound = ctrl_pl.GetPosRef(hand) + bound_size_hands
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound
        ctrl_pl.SetPositionBound(hand, lower_bound, upper_bound)

    print "current_foot_position: ", ci.getPoseReference("l_sole")[0].translation
    print "height contact: ", height_contact

    # # set position reference and bounds for FEET
    for foot in feet_list:
        contact_foot = [p[0] - d[0], p[1] - d[1] + ci.getPoseReference(foot)[0].translation[1], p[2] - d[2] + height_contact]

        p = [contact_foot_x, 0, contact_hand_z]
        ctrl_pl.SetPosRef(foot, contact_foot)
        print "Setting contact position for", foot, " to ", ctrl_pl.GetPosRef(foot)
        lower_bound = ctrl_pl.GetPosRef(foot) - bound_size_wall
        upper_bound = ctrl_pl.GetPosRef(foot) + bound_size_wall
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound
        ctrl_pl.SetPositionBound(foot, lower_bound, upper_bound)

    sol_centroidal = ctrl_pl.Solve()

    print sol_centroidal

    return ctrl_pl, sol_centroidal

def hand_positioning_wall(robot, ci, ctrl_pl, contacts, hands_list, feet_list, mass, sol_centroidal, com_pl) :

    # WEIGHTS
    mu = 0.5
    com_pl.SetMu(mu)
    com_pl.SetCoMWeight(1000000000000)  # 100000.0
    com_pl.SetForceWeight(0.)  # 0.0000001

    # SET THRESHOLD FOR HANDS
    # for c in hands_list:
    #     com_pl.SetForceThreshold(c, 50.0)

    # SET HANDS CONTACTS
    for c_h in hands_list:
        contact_pos = ci.getPoseReference(c_h)[0].translation
        com_pl.SetContactPosition(c_h, contact_pos)
        print("Setting contact position for ", c_h, " to ", com_pl.GetContactPosition(c_h))

    # SET FEET CONTACTS
    for c_f in feet_list:
        contact_pos = ci.getPoseReference(c_f)[0].translation
        com_pl.SetContactPosition(c_f, contact_pos)
        print("Setting contact position for ", c_f, " to ", com_pl.GetContactPosition(c_f))

    # SET COM
    com_ref = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    print 'Setting com to: ', com_ref
    com_pl.SetCoMRef(com_ref)


    print("Starting hands placing...")

    # putting feet in the right position
    for hand_i in hands_list :

        com_pl.SetLiftingContact(hand_i)
        print 'LIFTING ', hand_i

        # find solution
        sol = com_pl.Solve()
        print(sol)

        com_pl.ResetLiftingContact(hand_i)
        print 'RESETTING LIFT ', hand_i
        # send commands to cartesio

        # move com
        com_disp = [sol.com[0], sol.com[1], world_odom_T_world]
        print(com_disp)
        com_ci = Affine3(pos=com_disp)
        reach_time = 2.0
        ci.setTargetPose('com', com_ci, reach_time)
        ci.waitReachCompleted('com')
        ci.update()

        # lift hand
        hand_ci = ci.getPoseReference(hand_i)[0]
        hand_ci.translation_ref()[2] += 0.05
        ci.setTargetPose(hand_i, hand_ci, reach_time / 2.0)
        ci.waitReachCompleted(hand_i)
        ci.update()

        # move hand
        hand_ci = Affine3(pos=sol_centroidal.contact_values_map[hand_i].position)
        ci.setTargetPose(hand_i, hand_ci, reach_time / 2.0)
        ci.waitReachCompleted(hand_i)
        ci.update()

        # get contacts references from cartesio and set them to the planner
        for c in hands_list :
            contact_pos = ci.getPoseReference(c)[0].translation
            com_pl.SetContactPosition(c, contact_pos)
            print("Setting contact position for ", c, " to ", com_pl.GetContactPosition(c))

    # PUT BACK COM IN THE MIDDLE
    for c_f in feet_list:
        contact_pos = ctrl_pl.GetPosRef(c_f)
        com_pl.SetContactPosition(c_f, contact_pos)
        print("Setting contact position for ", c_f, " to ", com_pl.GetContactPosition(c_f))

    sol = com_pl.Solve()

    com_disp = [sol.com[0], sol.com[1], world_odom_T_world]
    com_ci = Affine3(pos=com_disp)
    reach_time = 2.0
    ci.setTargetPose('com', com_ci, reach_time)
    ci.waitReachCompleted('com')
    ci.update()

def foot_positioning_wall(robot, ft_map, ci, ctrl_pl, contacts, hands_list, feet_list, mass, sol_centroidal, com_pl, forcepub) :

    # SET THRESHOLD FOR FEET
    # for c in feet_list:
    #     com_pl.SetForceThreshold(c, 50.0)
    force_threshold = 300
    direction = 2

    distance_for_reaching = - 0.01

    move_time_com = 2
    lift_time = 5
    reach_time = 10

    lift_heigth = 0.05
    # SET FEET CONTACTS
    for c_f in feet_list:
        # contact_pos = ctrl_pl.GetPosRef(c_f) # should be the same
        contact_pos = ci.getPoseReference(c_f)[0].translation
        com_pl.SetContactPosition(c_f, contact_pos)
        print("Setting contact position for ", c_f, " to ", com_pl.GetContactPosition(c_f))

    # SET HANDS CONTACTS
    for c_h in hands_list:
        # contact_pos = ctrl_pl.GetPosRef(c_h)
        contact_pos = ci.getPoseReference(c_h)[0].translation
        com_pl.SetContactPosition(c_h, contact_pos)
        print("Setting contact position for ", c_h, " to ", com_pl.GetContactPosition(c_h))

    # SET COM
    com_ref = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    print 'Setting com to: ', com_ref
    com_pl.SetCoMRef(com_ref)

    # SEND FORCE
    forcepub.sendForce(sol_centroidal.contact_values_map[hands_list[0]].force[0],
                       sol_centroidal.contact_values_map[hands_list[1]].force[0],
                       sol_centroidal.contact_values_map[feet_list[0]].force[0],
                       sol_centroidal.contact_values_map[feet_list[1]].force[0])


    print("Starting foot placing...")

    # putting feet in the right position
    for foot_i in feet_list:

        com_pl.SetLiftingContact(foot_i)
        print 'LIFTING ', foot_i

        # find solution
        sol = com_pl.Solve()
        print(sol)

        com_pl.ResetLiftingContact(foot_i)
        print 'RESETTING LIFT ', foot_i
        # send commands to cartesio

        # move com
        com_disp = [sol.com[0], sol.com[1], world_odom_T_world]
        print(com_disp)
        com_ci = Affine3(pos=com_disp)
        ci.setTargetPose('com', com_ci, move_time_com)
        ci.waitReachCompleted('com')
        ci.update()


        # prepare ROTATION OF SOLE
        theta = [0, 0, 0]
        rot_mat = rotation(theta)

        # lift sole
        sole_ci = ci.getPoseReference(foot_i)[0]
        sole_ci.translation_ref()[2] += lift_heigth
        sole_ci.linear = rot_mat
        ci.setTargetPose(foot_i, sole_ci, lift_time / 2.0)
        ci.waitReachCompleted(foot_i)
        ci.update()


        theta = [0, - np.pi / 2, 0]
        rot_mat = rotation(theta)

        # move foot
        goal_wall = [sol_centroidal.contact_values_map[foot_i].position[0], sol_centroidal.contact_values_map[foot_i].position[1], sol_centroidal.contact_values_map[foot_i].position[2]]
        goal_wall[0] -= distance_for_reaching
        foot_ci = Affine3(pos=goal_wall)
        foot_ci.linear = rot_mat
        ci.setTargetPose(foot_i, foot_ci, reach_time / 2.0)
        ci.waitReachCompleted(foot_i)
        ci.update()
        print foot_i, ': task finished.'

        # contact_flag = 0
        #
        # if foot_i == 'l_sole' :
        #     ft = ft_map['l_leg_ft']
        # elif foot_i == 'r_sole' :
        #     ft = ft_map['r_leg_ft']
        # else :
        #     print 'WRONG FOOT'
        #
        # while not contact_flag :
        #
        #     contact_flag = ci.waitReachCompleted(foot_i, 0.001)
        #
        #     if force_detection(robot, ft, direction, force_threshold) :
        #         print foot_i, ': contact detected, stopping task.'
        #         contact_flag = 1
        #
        #     robot.sense()

        surface_reacher_foot(robot, ft_map, foot_i)

        # WAIST ENABLE
        ci.setControlMode('Waist', pyci.ControlType.Position)

        # get contacts references from cartesio and set them to the planner
        for c in feet_list:
            contact_pos = ci.getPoseReference(c)[0].translation
            com_pl.SetContactPosition(c, contact_pos)
            print("Setting contact position for ", c, " to ", com_pl.GetContactPosition(c))

    # PUT BACK COM IN THE MIDDLE
    for c_f in feet_list:
        contact_pos = ctrl_pl.GetPosRef(c_f)
        com_pl.SetContactPosition(c_f, contact_pos)
        print("Setting contact position for ", c_f, " to ", com_pl.GetContactPosition(c_f))

    sol = com_pl.Solve()

    com_disp = [sol.com[0], sol.com[1], world_odom_T_world]
    com_ci = Affine3(pos=com_disp)
    reach_time = 2.0
    ci.setTargetPose('com', com_ci, reach_time)
    ci.waitReachCompleted('com')
    ci.update()


if __name__ == '__main__':

    np.set_printoptions(precision=3, suppress=True)
    rospy.init_node('coman_sheep_commander')

    roscpp_init('coman_sheep', [])
    # define contacts for the ForcePublisher
    contact_joints = ['l_sole', 'r_sole', 'l_ball_tip', 'r_ball_tip']


    forcepub = fp.ForcePublisher(contact_joints)

    # set zero forces on the links NOT IN CONTACTS (l_ball_tip, r_ball_tip)
    zero_arm_forces = [0., 0., 400,
                       0., 0., 400,
                       0., 0., 0.,
                       0., 0., 0.]

    forcepub.sendForce(zero_arm_forces)

    robot = get_robot()
    robot.sense()

    xbotstiff.set_legs_default_stiffness(robot, [2000, 2000, 2000, 2000, 2000, 2000])
    xbotdamp.set_legs_default_damping(robot, [10, 10, 10, 10, 10, 10])

    exit(0)

    ft_map = sensors_init()

    ci, feet_list, hands_list, contacts, mass, world_odom_T_world, dist_hands, bound_size = init_param()


    homing(ci)

    com_pl = gen_com_planner(contacts, mass)

    ctrl_pl_sheep, sol_centroidal_sheep = optimal_pos_sheep(ci, contacts, hands_list, feet_list, mass, dist_hands, bound_size)

    # # foot_positioning(ci, ctrl_pl_sheep, contacts, hands_list, feet_list, mass, sol_centroidal_sheep, com_pl)

    hand_positioning(robot, ft_map, ci, ctrl_pl_sheep, contacts, hands_list, feet_list, mass, sol_centroidal_sheep, com_pl, forcepub)

    surface_reacher(robot, ft_map)

    # sending forces ..
    forcepub.sendForce(0.1, 0.1, 200,
                       0.1, 0.1, 200,
                       sol_centroidal_sheep.contact_values_map[hands_list[0]].force[0],
                       sol_centroidal_sheep.contact_values_map[hands_list[1]].force[0])

    advance_com(ci)

    ctrl_pl_wall, sol_centroidal_wall = optimal_pos_wall(ci, contacts, hands_list, feet_list, mass)

    # # hand_positioning_wall(ci, ctrl_pl_wall, contacts, hands_list, feet_list, mass, sol_centroidal_wall, com_pl)

    foot_positioning_wall(robot, ft_map, ci, ctrl_pl_wall, contacts, hands_list, feet_list, mass, sol_centroidal_wall, com_pl, forcepub)

















    # DISABLE COM
    # ci.setControlMode("com", pyci.ControlType.Disabled)

    # ENABLE WAIST
    # ci.setControlMode("Waist", pyci.ControlType.Position)

    # CROUCH WAIST
    # waist_ci = ci.getPoseReference("Waist")[0].translation
    # waist_ci[2] -= 0.3
    # waist_ci = Affine3(pos=waist_ci)
    # reach_time = 2.0
    # ci.setTargetPose('Waist', waist_ci, reach_time)
    # ci.waitReachCompleted('Waist')
    # ci.update()

    # CROUCH COM
    # com_disp = (ci.getPoseFromTf('ci/com', 'ci/world_odom').translation)
    # com_disp = [com_disp[0], com_disp[1], com_disp[2]-0.25]
    # com_ci = Affine3(pos=com_disp)
    # reach_time = 5.0
    # ci.setTargetPose('com', com_ci, reach_time)
    # ci.waitReachCompleted('com')
    # ci.update()

    # RELEASE WAIST CONSTRAINT
    # ci.setControlMode("torso", pyci.ControlType.Disabled)

    # RELEASE COM CONSTRAINT
    # ci.setControlMode("com", pyci.ControlType.Disabled)

    # # ==================================================================================================================
    # # ==================================================================================================================