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
from sensor_msgs.msg import JointState

start_pos = None

def start_pos_callback(data):
    global start_pos
    start_pos = data

def run(robot, ft_map, ci, ctrl_pl, contacts_links, hands_list, feet_list, sol_centroidal, com_pl, forcepub, world_odom_T_world, logger) :


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


    initial_com = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    print 'initial_com: ', initial_com


    print("Starting foot placing...")

    # putting feet in the right position
    for foot_i in feet_list:

        # SET COM
        com_ref = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation

        if foot_i == 'l_sole' :
            com_ref[1] = initial_com[1] - 0.05  # MOVE COM ON THE SIDE!

        if foot_i == 'r_sole' :
            com_ref[0] = com_ref[0] + 0.05
            com_ref[1] = initial_com[1]  # MOVE COM IN THE MIDDLE!

        print 'Setting com ref to: ', com_ref
        com_pl.SetCoMRef(com_ref)

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
        com_disp = [sol.com[0], sol.com[1], sol.com[2]]

        com_ci = Affine3(pos=com_disp)
        ci.setTargetPose('com', com_ci, move_time_com)
        ci.waitReachCompleted('com')
        ci.update()
        print "Done: ", ci.getPoseFromTf('ci/com', 'ci/world_odom').translation

        if foot_i == 'l_sole':

            raw_input("Press Enter to get starting pose from cartesian solution.")

            rospy.Subscriber("/cartesian/solution", JointState, start_pos_callback)
            rospy.sleep(1)
            raw_input("Turn on left foot Planner. Disable cartesio. Press Enter to start planner left.")

            joint_names = ["VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5",
                           "VIRTUALJOINT_6",
                           "LHipLat", "LHipSag", "LHipYaw", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipLat", "RHipSag",
                           "RHipYaw", "RKneePitch", "RAnklePitch", "RAnkleRoll", "WaistLat", "WaistYaw", "LShSag", "LShLat",
                           "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2", "RShSag", "RShLat", "RShYaw", "RElbj",
                           "RForearmPlate",
                           "RWrj1", "RWrj2"]

            ## without singularity
            goal_pos_left = [0.05573729400167113, -0.06947883510411024, -0.20751821206670207, 0.12429667516944151, 0.8849470128966822, -0.022411855389329754, -0.11666467659105043, -0.7627741830617885, -0.2734355503290767, 2.3214736573594834, -0.872665808000081, -0.19540428798391687, -0.03468311724094864, -1.7866746567944674, -0.025411763194786535, 1.496657523972716, -0.5655945029726186, -0.10496579393534855, -0.08098361306596884, -0.07331380995906533, 0.17934555560316112, 0.30786277563783015, 0.3270633311559586, -1.2694988284807516, 0.034189528418359566, -0.12636131675787898, 1.204561182183304e-05, 0.04982332460143657, -0.20438964313392347, -0.16974017674830408, -1.2428230667694045, -0.019438513997855827, -0.11188556609797803, 1.090003907467298e-05]
            # goal_pos_left = [0.03593148162583641, -0.06174289792910162, -0.18092641325187578, -0.03397445438325034, 1.091167470384951, 0.088616440875038, -0.18784379710283558, -1.175086146194122, -0.46547700230031197, 2.3977062303227856, -0.7325906610245712, -0.2562330269765112, -0.08902206267966255, -1.8200010191407394, -0.16959076863504335, 1.13691817320529, -0.35317465529543524, -0.1165526395989365, 0.013491695439021772, -0.058489678664295844, 0.18432051146554337, 0.29882476741094877, 0.3987933073023432, -1.4792184399426465, 0.06335064699172555, -0.22411389286727987, 1.2551339524889185e-05, 0.07667686123855365, -0.23510019923923467, -0.22069188439435133, -1.471018514051273, -0.03411253252438734, -0.20969768972816052, 1.1295249611336383e-05]
            joint_map_start = dict(zip(joint_names, start_pos.position))
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

            # RISE STIFFNESS AND DAMPING FOR MOVEMENT IN AIR ---------------------------------------------------------------

            default_stiffness_leg = 1500  # 1500
            default_damping_leg = 10

            xbotstiff.set_leg_stiffness(robot, foot_i,
                                        [default_stiffness_leg, default_stiffness_leg, default_stiffness_leg,
                                         default_stiffness_leg, default_stiffness_leg, default_stiffness_leg])

            xbotdamp.set_leg_damping(robot, foot_i,
                                     [default_damping_leg, default_damping_leg, default_damping_leg,
                                      default_damping_leg, default_damping_leg, default_damping_leg])

            # --------------------------------------------------------------------------------------------------------------

            raw_input("Press Enter to execute planning. You can turn off planner.")

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

        if foot_i == 'r_sole':
            raw_input("Press Enter to get starting pose from cartesian solution.")

            rospy.Subscriber("/cartesian/solution", JointState, start_pos_callback)
            rospy.sleep(1)
            raw_input("Turn on RightFoot planner. Disable cartesio. Press Enter start planner right.")

            joint_names = ["VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5",
                           "VIRTUALJOINT_6",
                           "LHipLat", "LHipSag", "LHipYaw", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipLat", "RHipSag",
                           "RHipYaw", "RKneePitch", "RAnklePitch", "RAnkleRoll", "WaistLat", "WaistYaw", "LShSag", "LShLat",
                           "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2", "RShSag", "RShLat", "RShYaw", "RElbj",
                           "RForearmPlate",
                           "RWrj1", "RWrj2"]


            ## without singularity
            # goal_pos_right = [0.1945239809767894, 0.0026299124743108153, -0.15811195550665297, -0.11798770043382477, 1.0874527820527142, 0.14049468208575305, 0.05001929556741128, -0.19630040531170267, -0.038119101658657006, 0.9695843768799659, -0.295044466715187, -0.0031151629187466745, 0.02310138966368068, -0.7874185281921204, -0.05753490272895689, 1.8721041086854857, -0.605693655997356, -0.010265437096064886, 0.052560521958857784, -0.035888591605421086, 0.23121732935096442, 0.2392932118544543, 0.21200878064287884, -1.3377621269672166, 0.02105713039572486, -0.14244172258643367, 6.2346643807816655e-06, 0.2332193619008002, -0.2440227372796754, -0.21237343583767934, -1.3356032482769158, -0.020850491649974434, -0.1417855097605663, 5.057770279089842e-06]

            # goal_pos_right = [0.2129395339593295, 0.003564472705763921, -0.1881106906840834, -0.09723095573264, 0.8511143369083485, 0.15854483284941073, 0.0866618728916122, 0.23462752886414276, -0.08892715047960058, 0.7720613026209996, -0.319784745235784, 0.001596762362295597, 0.04248403354221857, -0.34066193528564875, -0.10150402561479487, 1.756221398555215, -0.7031729575080056, -0.01608730127009427, 0.061956848552536144, -0.08642878335407554, 0.23626936480478355, 0.3703864450306634, -0.013432717454037575, -1.1889078276580844, 0.028016807441083105, 0.04061086612319407, -1.0015741070534482e-05, 0.23056870194923823, -0.37306624754096496, 0.021006321848159558, -1.1847640334464073, -0.010327621883329955, 0.044402819246105604, -5.352675902884152e-06]
            goal_pos_right = [0.20324161397602533, 0.009590569752187507, -0.14275866537665968, -0.1763403260690361, 0.8316962823196724, 0.3037760688755297, 0.13927697863026894, 0.08195909856264712, -0.17595140745804427, 0.9145956137459178, -0.31089356732397117, -0.002187572731195116, 0.04375838761996091, -0.4609743977700319, -0.20435983361438154, 1.7078680509888184, -0.6011738238332143, -0.01891127338725881, 0.11776178160999558, -0.17290487441930497, 0.023163637343657503, 0.2877918018778392, 0.08928269886050044, -0.9609938863694034, 0.05516893085809396, 0.15521460579049629, -1.0872968126503481e-05, 0.021830513050282982, -0.3216172440875754, -0.042145311897442285, -0.9560976192829505, -0.031323788455077765, 0.16229011973812102, -6.1715554302031005e-06]

            joint_map_start = dict(zip(joint_names, start_pos.position))
            joint_map_goal = dict(zip(joint_names, goal_pos_right))

            frames_in_contact = ["TCP_L", "TCP_R", "l_sole"]

            mat_1 = rotation([0, -np.pi / 2, 0])
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

            default_stiffness_leg = 1500  # 1500
            default_damping_leg = 10

            xbotstiff.set_leg_stiffness(robot, foot_i,
                                        [default_stiffness_leg, default_stiffness_leg, default_stiffness_leg,
                                         default_stiffness_leg, default_stiffness_leg, default_stiffness_leg])

            xbotdamp.set_leg_damping(robot, foot_i,
                                     [default_damping_leg, default_damping_leg, default_damping_leg,
                                      default_damping_leg, default_damping_leg, default_damping_leg])

            # --------------------------------------------------------------------------------------------------------------

            raw_input("Press Enter to execute planning.")

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

        raw_input("Press Enter to start surface reacher. Restart Cartesio.")

        # PUT BACK STACK AS IT WAS
        # disable Waist
        ci.setControlMode('Waist', pyci.ControlType.Disabled)
        # change back hands fixed in world
        for hand_i in hands_list:
            ci.setBaseLink(hand_i, 'world')


        # reaching for the wall with foot_i
        surface_reacher.run_foot(ci, robot, ft_map, foot_i)

        # LOWER STIFFNESS AND DAMPING FOR FORCE CONTROL ---------------------------------------------------------------

        default_stiffness_leg = 500  # 1500
        default_damping_leg = 10

        xbotstiff.set_leg_stiffness(robot, foot_i,
                                    [default_stiffness_leg, default_stiffness_leg, default_stiffness_leg,
                                     default_stiffness_leg, default_stiffness_leg, default_stiffness_leg])

        xbotdamp.set_leg_damping(robot, foot_i,
                                 [default_damping_leg, default_damping_leg, default_damping_leg,
                                  default_damping_leg, default_damping_leg, default_damping_leg])

        # --------------------------------------------------------------------------------------------------------------

        lower_ankle_impedance.run(robot, foot_i)


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

        if foot_i == 'r_sole' :
            # SEND FORCE

            com_ref = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
            print 'Setting com ref to: ', com_ref
            com_pl.SetCoMRef(com_ref)
            # find solution
            sol = com_pl.Solve()
            print "sending force to ", foot_i
            send_F_n.send(forcepub, contacts_links, hands_list, feet_list, sol)

    # # PUT BACK COM IN THE MIDDLE
    # for c_f in feet_list:
    #     contact_pos = ctrl_pl.GetPosRef(c_f)
    #     com_pl.SetContactPosition(c_f, contact_pos)
    #     print("Setting contact position for ", c_f, " to ", com_pl.GetContactPosition(c_f))
    #
    # sol = com_pl.Solve()
    #
    # com_disp = [sol.com[0], sol.com[1], world_odom_T_world]
    # com_ci = Affine3(pos=com_disp)
    # reach_time = 2.0
    # ci.setTargetPose('com', com_ci, reach_time)
    # ci.waitReachCompleted('com')
    # ci.update()

def rotation(theta):

    tx, ty, tz = theta

    Rx = np.array([[1, 0, 0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
    Ry = np.array([[np.cos(ty), 0, -np.sin(ty)], [0, 1, 0], [np.sin(ty), 0, np.cos(ty)]])
    Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0, 0, 1]])

    return np.dot(Rx, np.dot(Ry, Rz))