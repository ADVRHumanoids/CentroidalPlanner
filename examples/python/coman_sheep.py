#!/usr/bin/env python
from cartesian_interface.pyci_all import *
import centroidal_planner.pycpl as cpl
import numpy as np
def main():

    # get robot
    
    # get cartesio ros client
    ci = pyci.CartesianInterfaceRos()

    #select contacts of COMAN+
    feet_list = ['l_sole', 'r_sole']
    hands_list = ['l_ball_tip', 'r_ball_tip']
    contacts = feet_list + hands_list

    mass = 70.0

    height_robot = (ci.getPoseFromTf('ci/com', 'ci/world_odom').translation)[2]
    # --------------------  centroidal planner  ------------------------------------------------------------------------
    # get environment (just ground)
    env = cpl.Ground()

    env.SetGroundZ(ci.getPoseReference('l_sole')[0].translation[2])
    # distance hands from feet
    dist_hands_x = 0.5
    dist_hands_y = 0.1

    ctrl_pl = cpl.CentroidalPlanner(contacts, mass, env)

    # ctrl_pl.SetCoMWeight(100000)
    # ctrl_pl.SetPosWeight(100000000)
    # ctrl_pl.SetForceWeight(0)

    # size of the bound region for solver
    bound_size = 0.25

    # set position reference for links and set bounds
    for foot in feet_list :
        contact_foot = ci.getPoseReference(foot)[0].translation
        # contact_foot[2] = 0.
        ctrl_pl.SetPosRef(foot, contact_foot)
        print "Setting contact position for",  foot, " to ", ctrl_pl.GetPosRef(foot)
        lower_bound = ctrl_pl.GetPosRef(foot) - bound_size
        upper_bound = ctrl_pl.GetPosRef(foot) + bound_size
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound

    i = 0
    for hand in hands_list :
        contact_hand = ci.getPoseReference(feet_list[i])[0].translation
        # contact_hand[2] = 0.
        contact_hand[0] += dist_hands_x
        if hand == "l_ball_tip":
            contact_hand[1] += dist_hands_y
        else:
            contact_hand[1] -= dist_hands_y

        i = i+1
        ctrl_pl.SetPosRef(hand, contact_hand)
        print "Setting contact position for",  hand, " to ", ctrl_pl.GetPosRef(hand)
        lower_bound = ctrl_pl.GetPosRef(hand) - bound_size
        upper_bound = ctrl_pl.GetPosRef(hand) + bound_size
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound

    sol_centroidal = ctrl_pl.Solve()

    print sol_centroidal

    # print list(sol.contact_values_map)
    # print sol.contact_values_map["l_sole"].force
    # print sol.contact_values_map["l_sole"].position
    # print sol.contact_values_map["l_sole"].normal

    #===================================================================================================================
    #===================================================================================================================
    #===================================================================================================================

    # ---------------------------- preparing tasks ---------------------------------------------------------------------

    # HANDS WRT WAIST
    for hand_i in hands_list:
        ci.setBaseLink(hand_i, 'Waist')

    # WAIST DISABLED
    ci.setControlMode('Waist', pyci.ControlType.Disabled)

        # ci.setBaseLink(hand_i, )

    # -------------------------- com planner ---------------------------------------------------------------------------
    # set up com planner
    com_pl = cpl.CoMPlanner(contacts, mass)

    # WEIGHTS
    mu = 0.5
    com_pl.SetMu(mu)
    com_pl.SetCoMWeight(1000000000000)    #100000.0
    com_pl.SetForceWeight(0.)  #0.0000001


    for c in feet_list :
        com_pl.SetForceThreshold(c, 50.0)

    # SET FEET CONTACTS
    for c_f in feet_list :
        contact_pos = ctrl_pl.GetPosRef(c_f)
        com_pl.SetContactPosition(c_f, contact_pos)
        print("Setting contact position for ", c_f, " to ", com_pl.GetContactPosition(c_f))

    # SET HANDS CONTACTS
    for c_h in hands_list :
        contact_pos = ctrl_pl.GetPosRef(c_h)
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
        com_disp = [sol.com[0], sol.com[1], height_robot]
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
            contact_pos = ctrl_pl.GetPosRef(c)
            com_pl.SetContactPosition(c, contact_pos)
            print("Setting contact position for ", c, " to ", com_pl.GetContactPosition(c))


    # ============================== PUT BACK COM IN THE MIDDLE ========================================================
    for c_f in feet_list :
        contact_pos = ctrl_pl.GetPosRef(c_f)
        com_pl.SetContactPosition(c_f, contact_pos)
        print("Setting contact position for ", c_f, " to ", com_pl.GetContactPosition(c_f))

    sol = com_pl.Solve()

    com_disp = [sol.com[0], sol.com[1], height_robot]
    com_ci = Affine3(pos=com_disp)
    reach_time = 2.0
    ci.setTargetPose('com', com_ci, reach_time)
    ci.waitReachCompleted('com')
    ci.update()

    # ==================================================================================================================
    # SET HANDS WRT WORLD
    for hand_i in hands_list:
        ci.setBaseLink(hand_i, 'world')

    # PLACE HANDS ON THE GROUND
    reach_time_hand = 20
    dist_from_ground = 0.15
    hand_one = [sol_centroidal.contact_values_map[hands_list[0]].position[0], sol_centroidal.contact_values_map[hands_list[0]].position[1], sol_centroidal.contact_values_map[hands_list[0]].position[2]+dist_from_ground]
    hand_two = [sol_centroidal.contact_values_map[hands_list[1]].position[0], sol_centroidal.contact_values_map[hands_list[1]].position[1], sol_centroidal.contact_values_map[hands_list[1]].position[2]+dist_from_ground]

    hand_one = Affine3(pos=hand_one)
    hand_two = Affine3(pos=hand_two)

    ci.setTargetPose(hands_list[0], hand_one, reach_time_hand / 2.0)
    ci.setTargetPose(hands_list[1], hand_two, reach_time_hand / 2.0)
    ci.update()

    ci.waitReachCompleted(hands_list[0])
    ci.waitReachCompleted(hands_list[1])

    # ADVANCE WITH COM
    com_advance = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    com_advance[0] = 0.15
    com_ci = Affine3(pos=com_advance)
    reach_time = 4.0
    ci.setTargetPose('com', com_ci, reach_time)
    ci.waitReachCompleted('com')
    ci.update()

    # ==================================================================================================================

if __name__ == '__main__':

    np.set_printoptions(precision=3, suppress=True)

    main()

















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