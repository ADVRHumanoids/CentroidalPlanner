from cartesian_interface.pyci_all import *
import numpy as np
import lower_ankle_impedance as lower_ankle_impedance
import surface_reacher as surface_reacher
import xbot_stiffness as xbotstiff
import xbot_damping as xbotdamp
import send_Force_and_Normal as send_F_n
import cartesio_planner_1 as cp
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

            ## without singularity
            goal_pos_left = [0.05573729400167113, -0.06947883510411024, -0.20751821206670207, 0.12429667516944151, 0.8849470128966822, -0.022411855389329754, -0.11666467659105043, -0.7627741830617885, -0.2734355503290767, 2.3214736573594834, -0.872665808000081, -0.19540428798391687, -0.03468311724094864, -1.7866746567944674, -0.025411763194786535, 1.496657523972716, -0.5655945029726186, -0.10496579393534855, -0.08098361306596884, -0.07331380995906533, 0.17934555560316112, 0.30786277563783015, 0.3270633311559586, -1.2694988284807516, 0.034189528418359566, -0.12636131675787898, 1.204561182183304e-05, 0.04982332460143657, -0.20438964313392347, -0.16974017674830408, -1.2428230667694045, -0.019438513997855827, -0.11188556609797803, 1.090003907467298e-05]

            joint_map_goal = dict(zip(joint_names, goal_pos_left))
            frames_in_contact = ["TCP_L", "TCP_R", "r_sole"]

            mat = rotation([0, 0, 0])
            rot_mat = Affine3()
            rot_mat.linear = mat
            normals = [rot_mat.quaternion, rot_mat.quaternion, rot_mat.quaternion]

            max_time = 10.0
            planner_type = 'RRTConnect'
            interpolation_time = 0.01

            mapJointTraj = cp.compute(joint_map_start, joint_map_goal, frames_in_contact, max_time, planner_type, interpolation_time, normals=normals)

            if foot_i == 'r_sole':
                raw_input("Press Enter start planner.")

                joint_map_start = {}

                joint_names = ["VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5",
                               "VIRTUALJOINT_6",
                               "LHipLat", "LHipSag", "LHipYaw", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipLat",
                               "RHipSag",
                               "RHipYaw", "RKneePitch", "RAnklePitch", "RAnkleRoll", "WaistLat", "WaistYaw", "LShSag",
                               "LShLat",
                               "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2", "RShSag", "RShLat", "RShYaw",
                               "RElbj",
                               "RForearmPlate",
                               "RWrj1", "RWrj2"]

                ## without singularity
                goal_pos_left = []

                joint_map_goal = dict(zip(joint_names, goal_pos_left))
                frames_in_contact = ["TCP_L", "TCP_R", "l_sole"]

                mat_1 = rotation([0, np.pi / 2, 0])
                rot_mat_1 = Affine3()
                rot_mat_1.linear = mat_1

                mat = rotation([0, 0, 0])
                rot_mat = Affine3()
                rot_mat.linear = mat
                normals = [rot_mat.quaternion, rot_mat.quaternion, rot_mat_1.quaternion]

                max_time = 10.0
                planner_type = 'RRTConnect'
                interpolation_time = 0.01

                mapJointTraj = cp.compute(joint_map_start, joint_map_goal, frames_in_contact, max_time, planner_type, interpolation_time, normals=normals)

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

        robot.setControlMode(xbot.ControlMode.Stiffness() + xbot.ControlMode.Damping())

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