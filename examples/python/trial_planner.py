#!/usr/bin/env python
import cartesio_planning.pyplan as cpp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import query_yes_no as query_yn
import cartesian_interface.affine3


roscpp_init('cartesio_planning', [])

planner = cpp.PlannerClient()


joint_names = ["VIRTUALJOINT_1", "VIRTUALJOINT_2", "VIRTUALJOINT_3", "VIRTUALJOINT_4", "VIRTUALJOINT_5", "VIRTUALJOINT_6", "LHipLat", "LHipSag", "LHipYaw", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipLat", "RHipSag", "RHipYaw", "RKneePitch", "RAnklePitch", "RAnkleRoll", "WaistLat", "WaistYaw", "LShSag", "LShLat", "LShYaw", "LElbj", "LForearmPlate", "LWrj1", "LWrj2", "RShSag", "RShLat", "RShYaw", "RElbj", "RForearmPlate", "RWrj1", "RWrj2"]

start_position = [-0.014130067578267716, -0.0003950225138868169, -0.1986007829637569, -0.0006509170597944057, 0.8826449737683223,
                  0.000872636860768782, 0.00028897706287522954, -1.8312116317937361, -0.0006430702571908346, 1.3901233356218243,
                  -0.3815580773111069, -0.0007305196988230341, 0.0002886714460364018, -1.8310894135954228, -0.0006433656662272768,
                  1.3902253281031371, -0.3817822880161119, -0.0007305198085548906, -0.0017565787886696135, 6.49293263958643e-05,
                  -0.027415126795615335, 0.35957581411273704, 0.21939513751962691, -1.1557043020776563, 0.010977362157928367,
                  -0.05852717520634846, 5.8580515206005804e-06, -0.027530704068700118, -0.35857980401440936, -0.2192078206212112,
                  -1.1547169324732134, -0.010928758355845523, -0.058163622523019876, 4.3997414411386765e-06]


goal_position = [0.034024568029280396, -0.07504811373351948, -0.13005339070165617, 0.5389633459902076, 1.1316223514550658, -0.5782745461699126, 0.12017636588623921, -1.3675381173889825, 0.2861684800879465, 2.3004434846336106, -0.5554418427259643, -0.06447166956915328, 0.13446850110139266, -1.2784161487093035, 0.3594922487120925, -1.0567824419490034e-06, 0.15151463334427212, -0.023498357083749467, -0.14791747591957194, 0.02249369930571717, 0.055043039095789156, 0.40445979282440764, 0.44855455623617274, -1.2711413691047815, 0.05843132761777578, -0.15068438165289186, 2.065246981657416e-05, -0.2084342669783355, -0.2603362661793475, -0.18636649374679587, -1.2425577397166045, -0.02686941655774241, -0.12371413442781486, 1.883659982707331e-05]


frames_in_contact = ["TCP_L", "TCP_R", "r_foot_upper_right_link", "r_foot_upper_left_link", "r_foot_lower_right_link", "r_foot_lower_left_link"]

joint_map_start = dict(zip(joint_names, start_position))
joint_map_goal = dict(zip(joint_names, goal_position))

planner.setStartState(joint_map_start)
planner.setGoalState(joint_map_goal)
planner.setContactFrames('SET', frames_in_contact)


max_time = 10.0
planner_type = 'RRTConnect'
interp_time = 0.01

base_link = ['l_sole']
distal_link = ['r_sole']

traj_space = "Joint"

flag_ok = False
while (not flag_ok) :
    planner.callPlanner(max_time, planner_type, interp_time, traj_space, distal_link, base_link)  #traj_space, distal_link, base_link
    flag_ok = query_yn.ask('Is this trajectory good for you?')

mapJointTraj = planner.getJointTrajectory()
# mapCartTraj = planner.getCartesianTrajectory()

print mapJointTraj
# print mapCartTraj.keys()




