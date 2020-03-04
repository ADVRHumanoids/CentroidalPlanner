from cartesian_interface.pyci_all import *
import impact_detector as impact_detector

def run_hand(ci, robot, model, ft_map, hands_list, f_est, logger) :

    print 'starting surface reacher for hands.'
    # velocity desired
    vel_hands = [0, 0, -0.03, 0, 0, 0]

    ci.setControlMode(hands_list[0], pyci.ControlType.Velocity)
    ci.setControlMode(hands_list[1], pyci.ControlType.Velocity)


    force_treshold = 15
    direction = 0

    # position treshold
    pos_treshold = 0.05 #0.05
    initial_pose_l_arm = ci.getPoseFromTf('ci/com', 'ci/l_ball_tip').translation[direction]
    initial_pose_r_arm = ci.getPoseFromTf('ci/com', 'ci/r_ball_tip').translation[direction]

    print "initial_pose_l_arm: ", initial_pose_l_arm
    print "initial_pose_r_arm: ", initial_pose_r_arm
    # booleans

    while not impact_detector.run_hand(robot, ft_map['l_arm_ft'], direction, force_treshold) or not impact_detector.run_hand(robot, ft_map['r_arm_ft'], direction, force_treshold) :

        if (ci.getPoseFromTf('ci/com', 'ci/l_ball_tip').translation[direction] < initial_pose_l_arm - pos_treshold) and (ci.getPoseFromTf('ci/com', 'ci/r_ball_tip').translation[direction] < initial_pose_r_arm - pos_treshold) :
            break

        if not impact_detector.run_hand(robot, ft_map['l_arm_ft'], direction, force_treshold) :
            ci.setVelocityReference(hands_list[0], vel_hands)

        if not impact_detector.run_hand(robot, ft_map['r_arm_ft'], direction, force_treshold) :
            ci.setVelocityReference(hands_list[1], vel_hands)

        robot.sense()
        model.syncFrom(robot)
        f_est.update()

    # ci.setControlMode(hands_list[0], pyci.ControlType.Position)
    # ci.setControlMode(hands_list[1], pyci.ControlType.Position)

    ci.update()

def run_foot(ci, robot, ft_map, end_effector):

    print 'starting surface reacher for: ', end_effector
    # velocity desired
    vel_foot = [-0.01, 0, 0, 0, 0, 0]



    if end_effector == 'l_sole':
        ft = ft_map['l_leg_ft']
    elif end_effector == 'r_sole':
        ft = ft_map['r_leg_ft']
    else:
        print 'WRONG FOOT'

    # SETTING VELOCITY
    ci.setControlMode(end_effector, pyci.ControlType.Velocity)


    force_treshold = 65 #250
    direction = 2



    if end_effector == "r_sole" :
        force_treshold = 35 #120

    contact_sensed = False
    n_cycle = 0

    if end_effector == 'r_sole' :
        ci_end_effector = 'ci/r_sole'
    elif end_effector == 'l_sole' :
        ci_end_effector = 'ci/l_sole'

    # for position treshold (overrides forces, when reach a treshold it stops)
    initial_pose_foot = ci.getPoseFromTf('ci/com', ci_end_effector).translation[direction]
    print "initial_pose_foot: ", initial_pose_foot

    pos_treshold = 0.5#0.2

    while not contact_sensed :

        contact_sensed = False
        ci.setVelocityReference(end_effector, vel_foot)

        if impact_detector.run_foot(robot, ft, direction, force_treshold):
            print end_effector, ': waiting..'
            n_cycle += 1
        else:
            n_cycle = 0

        if n_cycle > 10:
            contact_sensed = True
            # print 'cycling for false positive..'
            n_cycle = 0

        # FOR SIMULATION
        if ci.getPoseFromTf('ci/com', ci_end_effector).translation[direction] > initial_pose_foot + pos_treshold :
            contact_sensed = True

        robot.sense()

    print end_effector, ': Contact sensed.'
    ci.update()

    ci.setControlMode(end_effector, pyci.ControlType.Position)
