from cartesian_interface.pyci_all import *
import numpy as np
import lower_ankle_impedance as lower_ankle_impedance
import surface_reacher as surface_reacher
import xbot_stiffness as xbotstiff
import xbot_damping as xbotdamp
import send_Force_and_Normal as send_F_n

def run(robot, ft_map, ci, ctrl_pl, contacts_links, hands_list, feet_list, sol_centroidal, com_pl, forcepub, world_odom_T_world) :


    # com_pl.SetCoMWeight(10)  # 100000.0
    # com_pl.SetForceWeight(1) # 0.0000001
    # SET THRESHOLD FOR FEET
    # for c in feet_list:
    #     com_pl.SetForceThreshold(c, 50.0)
    force_threshold = 300
    direction = 2

    distance_for_reaching = - 0.15

    move_time_com = 2
    lift_time = 10
    reach_time = 25

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
    com_ref[1] = -0.05 # MOVE COM ON THE SIDE!
    print 'Setting com to: ', com_ref
    com_pl.SetCoMRef(com_ref)


    print("Starting foot placing...")

    # putting feet in the right position
    for foot_i in feet_list:

        # lifting foot in solver to find optimal solution of the CoM
        com_pl.SetLiftingContact(foot_i)
        print 'LIFTING ', foot_i

        # find solution
        sol = com_pl.Solve()
        print(sol)

        com_pl.ResetLiftingContact(foot_i)
        print 'RESETTING LIFT ', foot_i
        # send commands to cartesio


        # SEND FORCE ------
        send_F_n.send(forcepub, contacts_links, hands_list, feet_list, sol)
        # -----------------

        raw_input("Press Enter to move CoM.")
        # move com
        print "Starting displacement of CoM ..."
        com_disp = [sol.com[0], sol.com[1], world_odom_T_world]

        com_ci = Affine3(pos=com_disp)
        ci.setTargetPose('com', com_ci, move_time_com)
        ci.waitReachCompleted('com')
        ci.update()
        print "Done: ", com_disp

        raw_input("Press Enter to lift foot.")

        # RISE STIFFNESS AND DAMPING FOR MOVEMENT IN AIR ---------------------------------------------------------------

        default_stiffness_leg = 1000  # 1500
        default_damping_leg = 10

        xbotstiff.set_leg_stiffness(robot, foot_i,
                                    [default_stiffness_leg, default_stiffness_leg, default_stiffness_leg,
                                     default_stiffness_leg, default_stiffness_leg, default_stiffness_leg])

        xbotdamp.set_leg_damping(robot, foot_i,
                                 [default_damping_leg, default_damping_leg, default_damping_leg,
                                  default_damping_leg, default_damping_leg, default_damping_leg])

        # --------------------------------------------------------------------------------------------------------------
        ci.setControlMode(foot_i, pyci.ControlType.Position)

        # prepare ROTATION OF SOLE
        theta = [0, 0, 0]
        rot_mat = rotation(theta)

        if foot_i == 'r_sole' :
            lift_heigth = 0.08


        # lift sole
        print "lifting sole..."
        sole_ci = ci.getPoseReference(foot_i)[0]
        sole_ci.translation_ref()[2] += lift_heigth
        # sole_ci.linear = rot_mat
        ci.setTargetPose(foot_i, sole_ci, lift_time / 2.0)
        ci.waitReachCompleted(foot_i)
        ci.update()

        theta = [0, - np.pi / 2, 0]
        rot_mat = rotation(theta)

        if foot_i == 'l_sole' :
            print 'setting LHipYaw postural reference'
            LHipYaw_map = dict()
            LHipYaw_map['LHipYaw'] = 0.10
            ci.setReferencePosture(LHipYaw_map)


        if foot_i == 'r_sole' :
            distance_for_reaching = -0.15

        if foot_i == 'r_sole' :

            print 'sending current reference for postural ...'
            joint_pos = robot.getJointPositionMap()
            postural_map = joint_pos

            # postural_map = {'RKneePitch': joint_pos['RKneePitch'],
            #                 'RHipSag': joint_pos['RHipSag'],
            #                 'RHipLat': joint_pos['RHipLat'],
            #                 'RAnklePitch': joint_pos['RAnklePitch'],
            #                 'RAnkleRoll': joint_pos['RAnkleRoll'],
            #                 'RHipYaw': joint_pos['RHipYaw']
            #                 }


            ci.setReferencePosture(postural_map)

            print 'Enabling Task on Waist.'
            ci.setControlMode('Waist', pyci.ControlType.Position)

            print 'Disabling Task on right leg.'
            ci.setControlMode('r_sole', pyci.ControlType.Disabled)

            knee_pos_map = dict()
            knee_pos_map['RKneePitch'] = robot.getJointPositionMap()['RKneePitch'] + 0.6
            print 'Changing postural for Right Knee'
            ci.setReferencePosture(knee_pos_map)

            raw_input('Press Enter to Enable Task on foot and Disable Task on Waist')

            print 'Enabling Task on right leg.'
            ci.setControlMode('r_sole', pyci.ControlType.Position)

            print 'Disabling Task on Waist.'
            ci.setControlMode('Waist', pyci.ControlType.Disabled)

        raw_input("Press Enter to move foot.")


        # move foot
        print "moving foot..."
        goal_wall = [sol_centroidal.contact_values_map[foot_i].position[0],
                     sol_centroidal.contact_values_map[foot_i].position[1],
                     sol_centroidal.contact_values_map[foot_i].position[2]]

        goal_wall[0] -= distance_for_reaching
        foot_ci = Affine3(pos=goal_wall)
        foot_ci.linear = rot_mat
        ci.setTargetPose(foot_i, foot_ci, reach_time / 2.0)
        ci.waitReachCompleted(foot_i)
        ci.update()
        print foot_i, ': task finished.'

        # lowering stiffness of ankle of foot_i
        lower_ankle_impedance.run(robot, foot_i)

        raw_input("Press Enter to start surface reacher.")
        # reaching for the wall with foot_i
        surface_reacher.run_foot(ci, robot, ft_map, foot_i)

        # LOWER STIFFNESS AND DAMPING FOR FORCE CONTROL ---------------------------------------------------------------

        default_stiffness_leg = 200  # 1500
        default_damping_leg = 10

        xbotstiff.set_leg_stiffness(robot, foot_i,
                                    [default_stiffness_leg, default_stiffness_leg, default_stiffness_leg,
                                     default_stiffness_leg, default_stiffness_leg, default_stiffness_leg])

        xbotdamp.set_leg_damping(robot, foot_i,
                                 [default_damping_leg, default_damping_leg, default_damping_leg,
                                  default_damping_leg, default_damping_leg, default_damping_leg])

        # --------------------------------------------------------------------------------------------------------------


        if foot_i == 'r_sole' :
            # SEND FORCE
            print "sending force to ", foot_i
            send_F_n.send(forcepub, contacts_links, hands_list, feet_list, sol_centroidal)

        # WAIST ENABLE
        # ci.setControlMode('Waist', pyci.ControlType.Position)

        # SET NEW CONTACT POSITION AND NORMAL
        # get contacts references from cartesio and set them to the planner
        contact_pos = ci.getPoseReference(foot_i)[0].translation
        com_pl.SetContactPosition(foot_i, contact_pos)
        normal = [1, 0, 0]
        com_pl.SetContactNormal(foot_i, normal)
        print("Setting contact position for ", foot_i, " to ", com_pl.GetContactPosition(foot_i))
        print "Setting normal for ", foot_i, "to ", normal



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

def rotation(theta):

    tx, ty, tz = theta

    Rx = np.array([[1, 0, 0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
    Ry = np.array([[np.cos(ty), 0, -np.sin(ty)], [0, 1, 0], [np.sin(ty), 0, np.cos(ty)]])
    Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0, 0, 1]])

    return np.dot(Rx, np.dot(Ry, Rz))