import impact_detector as impact_detector
from cartesian_interface.pyci_all import *

def run(robot, ft_map, ci, hands_list, sol_centroidal) :
    # ==================================================================================================================
    # SET HANDS WRT WORLD
    for hand_i in hands_list:
        ci.setBaseLink(hand_i, 'world')

    # for FEEDBACK CONTACT
    stop_before_contact = 0.05

    # PLACE HANDS ON THE GROUND
    reach_time_hand = 30
    dist_from_ground = 0.15 + stop_before_contact

    # SET POSITION MODE FOR HANDS
    ci.setControlMode(hands_list[0], pyci.ControlType.Position)
    ci.setControlMode(hands_list[1], pyci.ControlType.Position)

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
        if impact_detector.run(robot, ft_map['l_arm_ft'], direction, force_threshold) :
            print hands_list[0], ': contact detected, stopping task.'
            contact_flag_l = 1


        contact_flag_r = ci.waitReachCompleted(hands_list[1], 0.001)
        if impact_detector.run(robot, ft_map['r_arm_ft'], direction, force_threshold) :
            print hands_list[1], ': contact detected, stopping task.'
            contact_flag_r = 1

        robot.sense()


    print hands_list[0], ': task finished.'
    print hands_list[1], ': task finished.'
    ci.update()