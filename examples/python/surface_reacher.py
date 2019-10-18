from cartesian_interface.pyci_all import *
import impact_detector as impact_detector

def run_hand(ci, robot, model, ft_map, hands_list, f_est) :

    # velocity desired
    vel_hands = [0, 0, -0.03, 0, 0, 0]


    ci.setControlMode(hands_list[0], pyci.ControlType.Velocity)
    ci.setControlMode(hands_list[1], pyci.ControlType.Velocity)


    contact_treshold = 35
    direction = 0

    while not impact_detector.run(robot, ft_map['l_arm_ft'], direction, contact_treshold) or not impact_detector.run(robot, ft_map['r_arm_ft'], direction, contact_treshold) :


        if not impact_detector.run(robot, ft_map['l_arm_ft'], direction, contact_treshold) :
            ci.setVelocityReference(hands_list[0], vel_hands)

        if not impact_detector.run(robot, ft_map['r_arm_ft'], direction, contact_treshold) :
            ci.setVelocityReference(hands_list[1], vel_hands)

        robot.sense()
        model.syncFrom(robot)
        f_est.update()

    # ci.setControlMode(hands_list[0], pyci.ControlType.Position)
    # ci.setControlMode(hands_list[1], pyci.ControlType.Position)

    ci.update()

def run_foot(ci, robot, ft_map, end_effector):

    print('starting surface reacher for: ', end_effector)
    # velocity desired
    vel_foot = [-0.03, 0, 0, 0, 0, 0]

    if end_effector == 'l_sole':
        ft = ft_map['l_leg_ft']
    elif end_effector == 'r_sole':
        ft = ft_map['r_leg_ft']
    else:
        print 'WRONG FOOT'

    # SETTING VELOCITY
    ci.setControlMode(end_effector, pyci.ControlType.Velocity)

    contact_treshold = 250
    direction = 2

    contact_sensed = False
    n_cycle = 0

    while not contact_sensed:
        contact_sensed = False
        ci.setVelocityReference(end_effector, vel_foot)

        if impact_detector.run(robot, ft, direction, contact_treshold):
            print end_effector, ': waiting..'
            n_cycle += 1
        else:
            n_cycle = 0

        if n_cycle > 0:
            contact_sensed = True
            print end_effector, ': Contact sensed.'
            n_cycle = 0

        robot.sense()

    ci.update()

    ci.setControlMode(end_effector, pyci.ControlType.Position)
