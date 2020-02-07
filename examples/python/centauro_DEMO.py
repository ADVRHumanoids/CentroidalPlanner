#!/usr/bin/env python
import centroidal_planner.pycpl as cpl
import centroidal_planner.srv as cpl_srv
import numpy as np
from cartesian_interface.pyci_all import *
import h5py
import rospy
import geometry_msgs.msg
import matlogger2.matlogger as matl
import xbot_interface.config_options as cfg
import xbot_interface.xbot_interface as xbot
import cartesian_interface.srv as cisrv
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
import geometry_msgs.msg as msg

f_est_map = {}

def callback(data):
    f_est_map[data.header.frame_id] = data.wrench.force

def force_norm(data):
    return np.sqrt(data.x*data.x + data.y*data.y + data.z*data.z)

def wrench_msg_to_nparray(data):
    return np.array([data.x, data.y, data.z])

# ROS client for on_force_ref_srv service
def change_contact_force_client(link_name, force):
    rospy.wait_for_service('/cpl_torque_feedforward/change_contact_force')
    try:
       change_contact_force = rospy.ServiceProxy('/cpl_torque_feedforward/change_contact_force', cpl_srv.SetContactForce)
       req = cpl_srv.SetContactForceRequest()
       req.link_name = link_name
       req.contact_force.wrench.force.x = force[0]
       req.contact_force.wrench.force.y = force[1]
       req.contact_force.wrench.force.z = force[2]
       req.contact_force.wrench.torque.x = 0.0
       req.contact_force.wrench.torque.y = 0.0
       req.contact_force.wrench.torque.z = 0.0
       res = change_contact_force(req)
       print(res)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# ROS client for change_contact_frame service
def change_contact_frame_client(link_name, quat):
    rospy.wait_for_service('/cpl_torque_feedforward/change_contact_frame')
    try:
       change_contact_frame = rospy.ServiceProxy('/cpl_torque_feedforward/change_contact_frame', cpl_srv.SetContactFrame)
       req = cpl_srv.SetContactFrameRequest()
       req.link_name = link_name
       req.orientation.x = quat[0]
       req.orientation.y = quat[1]
       req.orientation.z = quat[2]
       req.orientation.w = quat[3]
       res = change_contact_frame(req)
       print(res)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# ROS client for on_manip_wrench_recv service
def change_manipulation_wrench_client(manip_wrench):
    rospy.wait_for_service('/cpl_torque_feedforward/change_manipulation_wrench')
    try:
       change_manipulation_wrench = rospy.ServiceProxy('/cpl_torque_feedforward/change_manipulation_wrench', cpl_srv.SetManipulationWrench)
       req = cpl_srv.SetManipulationWrenchRequest()
       req.manip_wrench.wrench = manip_wrench
       res = change_manipulation_wrench(req)
       print(res)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def set_rear_legs_low_stiffness(robot):

    ctrl_map = {'hip_pitch_3': xbot.ControlMode.Stiffness(),
                'knee_pitch_3': xbot.ControlMode.Stiffness(),
                'ankle_pitch_3': xbot.ControlMode.Stiffness(),
                'hip_pitch_4': xbot.ControlMode.Stiffness(),
                'knee_pitch_4': xbot.ControlMode.Stiffness(),
                'ankle_pitch_4': xbot.ControlMode.Stiffness()}

    robot.setControlMode(ctrl_map)

    K = robot.getStiffnessMap()
    K['hip_pitch_3'] = 500.0
    K['knee_pitch_3'] = 500.0
    K['ankle_pitch_3'] = 250.0
    K['hip_pitch_4'] = 500.0
    K['knee_pitch_4'] = 500.0
    K['ankle_pitch_4'] = 250.0

    robot.setStiffness(K)

    for k in range(100):
        robot.move()
        rospy.Rate(1000.).sleep()

    print "REAR LEGS LOW STIFFNESS"

def set_rear_legs_init_stiffness(robot, K_init):

    ctrl_map = {'hip_pitch_3': xbot.ControlMode.Stiffness(),
                'knee_pitch_3': xbot.ControlMode.Stiffness(),
                'ankle_pitch_3': xbot.ControlMode.Stiffness(),
                'hip_pitch_4': xbot.ControlMode.Stiffness(),
                'knee_pitch_4': xbot.ControlMode.Stiffness(),
                'ankle_pitch_4': xbot.ControlMode.Stiffness()}

    robot.setControlMode(ctrl_map)

    K = robot.getStiffnessMap()
    K['hip_pitch_3'] = K_init['hip_pitch_3']
    K['knee_pitch_3'] = K_init['knee_pitch_3']
    K['ankle_pitch_3'] = K_init['ankle_pitch_3']
    K['hip_pitch_4'] = K_init['hip_pitch_4']
    K['knee_pitch_4'] = K_init['knee_pitch_4']
    K['ankle_pitch_4'] = K_init['ankle_pitch_4']

    robot.setStiffness(K)

    for k in range(100):
        robot.move()
        rospy.Rate(1000.).sleep()

    print "REAR LEGS DEFAULT STIFFNESS"

rate_param = rospy.get_param('~rate', 10.0)

rospy.init_node('centauro_DEMO')
rate = rospy.Rate(rate_param)

logger = matl.MatLogger2('/tmp/centauro_DEMO_log')
logger.setBufferMode(matl.BufferMode.CircularBuffer)

opt = cfg.ConfigOptions.FromConfigFile()
robot = xbot.RobotInterface(opt)
robot.sense()
robot.setControlMode(xbot.ControlMode.Idle())

model = xbot.ModelInterface(opt)
model.update()

ci = pyci.CartesianInterfaceRos()

contacts = ['wheel_1', 'wheel_2', 'wheel_3', 'wheel_4']

for contact_name in contacts:
    ci.setVelocityLimits(contact_name, 1000, 1000)
    ci.setAccelerationLimits(contact_name, 1000, 1000)

K_init = robot.getStiffnessMap()
K_init_hip_pitch = K_init.get('hip_pitch_3')
K_init_knee_pitch = K_init.get('knee_pitch_3')
K_init_ankle_pitch = K_init.get('ankle_pitch_3')

# Lowering the Homing CoM
com0_offset = rospy.get_param('~com0_offset', [0.0, 0.0, 0.0])
com = ci.getPoseFromTf('ci/com', 'ci/world_odom')
com0 = Affine3(pos=com.translation + com0_offset)
ci.setTargetPose('com', com0, 2.0)
ci.waitReachCompleted('com')

# Set up superquadric environment
superquadric_env = cpl.Superquadric()
mu = rospy.get_param('~mu', 0.3)
superquadric_env.SetMu(mu)

# Set up Centroidal Planner
cpl_planner = cpl.CentroidalPlanner(contacts, model.getMass(), superquadric_env)
cpl_planner.SetCoMRef(com0.translation)

F_min = [-500.0, -500.0, -500.0]
F_max = [500.0, 500.0, 500.0]

contacts_pose = {}

for contact_name in contacts:
    contacts_pose[contact_name] = ci.getPoseFromTf('ci/' + contact_name, 'ci/world_odom')
    cpl_planner.SetPosRef(contact_name, contacts_pose[contact_name].translation)
    cpl_planner.SetForceBounds(contact_name, F_min, F_max)

# set up WALL superquadric parameters
pelvis_pose = ci.getPoseFromTf('ci/pelvis', 'ci/world_odom')
ground_z = contacts_pose['wheel_1'].translation[2]
wall_pelvis_offset_x = rospy.get_param('~wall_pelvis_offset_x', 0.6)
C = [pelvis_pose.translation[0]+20.0, pelvis_pose.translation[1], pelvis_pose.translation[2]]
R = [C[0] + wall_pelvis_offset_x, 20.0, C[2] - ground_z]
P = [10.0, 10.0, 10.0]
superquadric_env.SetParameters(C, R, P)

cpl_planner.SetCoMWeight(100.0)
cpl_planner.SetPosWeight(10.0)

# Work-space BOX constraints
delta = 0.3
wheel_1_pos_lb = contacts_pose['wheel_1'].translation
wheel_1_pos_ub = contacts_pose['wheel_1'].translation + [delta, delta, delta]
wheel_2_pos_lb = contacts_pose['wheel_2'].translation - [0.0, delta, 0.0]
wheel_2_pos_ub = contacts_pose['wheel_2'].translation + [delta, 0.0, delta]
wheel_3_pos_lb = contacts_pose['wheel_3'].translation - [delta, 0.0, 0.0]
wheel_3_pos_ub = contacts_pose['wheel_3'].translation + [0.0, delta, delta]
wheel_4_pos_lb = contacts_pose['wheel_4'].translation - [delta, delta, 0.0]
wheel_4_pos_ub = contacts_pose['wheel_4'].translation + [0.0, 0.0, delta]

cpl_planner.SetPosBounds('wheel_1', wheel_1_pos_lb, wheel_1_pos_ub)
cpl_planner.SetPosBounds('wheel_2', wheel_2_pos_lb, wheel_2_pos_ub)
cpl_planner.SetPosBounds('wheel_3', wheel_3_pos_lb, wheel_3_pos_ub)
cpl_planner.SetPosBounds('wheel_4', wheel_4_pos_lb, wheel_4_pos_ub)

tau_ext = rospy.get_param('~tau_ext', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
cpl_planner.SetManipulationWrench(tau_ext)

sol = cpl_planner.Solve()
print sol

com_ref = sol.com

F_wheel_ref = {}
p_wheel_ref = {}
n_wheel_ref = {}
pose_wheel_ref = {}

for contact in contacts:
    F_wheel_ref[contact] = sol.contact_values_map[contact].force
    p_wheel_ref[contact] = sol.contact_values_map[contact].position
    n_wheel_ref[contact] = sol.contact_values_map[contact].normal
    pose_wheel_ref[contact] = cpl.GetAffineFromNormal(p_wheel_ref[contact], n_wheel_ref[contact])

# Set up surface reacher
surface_reach_thr = rospy.get_param('~surface_reach_thr', 0.0)
surface_reacher_link_names = ['wheel_1', 'wheel_2', 'wheel_3', 'wheel_4', 'arm1_8', 'arm2_8']
surface_reacher = cpl.SurfaceReacher(surface_reacher_link_names)

# Re-orientation front wheels
change_contact_frame_client('wheel_1', pose_wheel_ref['wheel_1'].quaternion)
change_contact_frame_client('wheel_2', pose_wheel_ref['wheel_2'].quaternion)
ci.setTargetPose('ankle2_1', pose_wheel_ref['wheel_1'], 1.0)
ci.waitReachCompleted('ankle2_1')
ci.setTargetPose('ankle2_2', pose_wheel_ref['wheel_2'], 1.0)
ci.waitReachCompleted('ankle2_2')

# Set up CoM Planner
com_planner = cpl.CoMPlanner(contacts, model.getMass())
com_planner.SetMu(mu)
com_planner.SetCoMWeight(100000.0)
com_planner.SetForceWeight(0.0000001)

F_thr = rospy.get_param('~F_thr', 200.0)

for c in contacts:
    com_planner.SetForceThreshold(c, F_thr)

# get contacts references from cartesio and set them to the planner
for c in contacts:
    com_planner.SetContactPosition(c, ci.getPoseReference(c)[0].translation)
    print "Setting contact position for ", c, " to ", com_planner.GetContactPosition(c)

    # get current com from cartesio and set it to planner
    com_ref = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    print 'Desired com is ', com_ref
    com_planner.SetCoMRef(com_ref)

contact_lift_names = ['wheel_3', 'wheel_4']

contact_lift_map = {'wheel_3': 'ankle2_3',
                    'wheel_4': 'ankle2_4'
                    }

set_rear_legs_low_stiffness(robot)

for contact_lift in contact_lift_names:

    # we lift contact
    com_planner.SetLiftingContact(contact_lift)

    # find solution
    sol = com_planner.Solve()
    print(sol)

    com_planner.ResetLiftingContact(contact_lift)
    com_planner.SetContactNormal(contact_lift, n_wheel_ref[contact_lift])
    for c in contacts:
        change_contact_force_client(c, sol.contact_values_map[c].force)

    change_contact_frame_client(contact_lift, pose_wheel_ref[contact_lift].quaternion)

    # send commands to cartesio
    com_ci = Affine3(pos=sol.com)
    reach_time = 2.0
    ci.setTargetPose('com', com_ci, reach_time)
    ci.waitReachCompleted('com')

    ci.update()

    ci.setTargetPose(contact_lift,  pose_wheel_ref[contact_lift], 4.0)
    ci.setTargetPose(contact_lift_map[contact_lift], pose_wheel_ref[contact_lift], 4.0)
    ci.waitReachCompleted(contact_lift_map[contact_lift])

    ci.update()

    # surface reacher
    surf_flag = surface_reacher.ReachSurface(ci, contact_lift, -0.01*n_wheel_ref[contact_lift], surface_reach_thr)

    if surf_flag:
        print 'Contact established!'
        for c in contacts:
            com_planner.SetContactPosition(c, ci.getPoseReference(c)[0].translation)
            print "Setting contact position for ", c, " to ", com_planner.GetContactPosition(c)


while not rospy.is_shutdown():
    rate.sleep()

set_rear_legs_init_stiffness(robot, K_init)

del logger
