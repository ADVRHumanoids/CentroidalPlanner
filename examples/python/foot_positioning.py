from cartesian_interface.pyci_all import *
import numpy as np
import lower_ankle_impedance as lower_ankle_impedance
import surface_reacher as surface_reacher
import xbot_stiffness as xbotstiff
import xbot_damping as xbotdamp
import send_Force_and_Normal as send_F_n
import cartesio_planner as cp
import rospy
import xbot_interface.xbot_interface as xbot

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

        if foot_i == 'l_sole':
            raw_input("Press Enter start planner.")


            joint_map_start = {}

            joint_names = ["VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5",
                           "VIRTUALJOINT_6",
                           "LHipLat", "LHipSag", "LHipYaw", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipLat", "RHipSag",
                           "RHipYaw", "RKneePitch", "RAnklePitch", "RAnkleRoll", "WaistLat", "WaistYaw", "LShSag", "LShLat",
                           "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2", "RShSag", "RShLat", "RShYaw", "RElbj",
                           "RForearmPlate",
                           "RWrj1", "RWrj2"]
            goal_pos = [0.034024568029280396, -0.07504811373351948, -0.13005339070165617, 0.5389633459902076,
                             1.1316223514550658, -0.5782745461699126,
                             0.12017636588623921, -1.3675381173889825, 0.2861684800879465, 2.3004434846336106,
                             -0.5554418427259643, -0.06447166956915328,
                             0.13446850110139266, -1.2784161487093035, 0.3594922487120925, -1.0567824419490034e-06,
                             0.15151463334427212, -0.023498357083749467,
                             -0.14791747591957194, 0.02249369930571717, 0.055043039095789156, 0.40445979282440764,
                             0.44855455623617274, -1.2711413691047815, 0.05843132761777578,
                             -0.15068438165289186, 2.065246981657416e-05, -0.2084342669783355, -0.2603362661793475,
                             -0.18636649374679587, -1.2425577397166045, -0.02686941655774241,
                             -0.12371413442781486, 1.883659982707331e-05]

            joint_map_goal = dict(zip(joint_names, goal_pos))
            frames_in_contact = ["TCP_L", "TCP_R", "r_foot_upper_right_link", "r_foot_upper_left_link",
                                 "r_foot_lower_right_link", "r_foot_lower_left_link"]
            max_time = 10.0
            planner_type = 'RRTConnect'
            interpolation_time = 0.01

            mapJointTraj = cp.compute(joint_map_start, joint_map_goal, frames_in_contact, max_time, planner_type, interpolation_time)

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

            raw_input("Press Enter to execute planning. Remember to disable Cartesio")

            vect_map = dict()
            sizeMap = mapJointTraj[mapJointTraj.keys()[0]].size
            robot.setControlMode(xbot.ControlMode.Position())
            for val in range(0, int(sizeMap)):

                for key in mapJointTraj:
                    vect_map[key] = mapJointTraj[key][val]


                robot.setPositionReference(vect_map)
                robot.move()
                rospy.sleep(0.01)


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

        raw_input("Press Enter to start surface reacher. Restart Cartesio.")
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