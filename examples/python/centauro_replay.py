#!/usr/bin/env python
import numpy as np
import matlogger2.matlogger as matl
import xbot_interface.config_options as cfg
import xbot_interface.xbot_interface as xbot
import h5py
import rospy

logger = matl.MatLogger2('/tmp/centauro_replay_log')
logger.setBufferMode(matl.BufferMode.CircularBuffer)

f = h5py.File('/home/matteo/advr-superbuild/external/CentroidalPlanner/examples/data_replay/centauro_wall.mat', 'r')
q = f.get('q_replay')
q = np.array(q)

qdot = f.get('qdot_replay')
qdot = np.array(qdot)

tau = f.get('tau_replay')
tau = np.array(tau)

opt = cfg.ConfigOptions()
robot = xbot.RobotInterface(opt)
robot.sense()
robot.setControlMode(xbot.ControlMode.Position())

ctrl_map = {'neck_velodyne': xbot.ControlMode.Idle(),
            'j_wheel_1': xbot.ControlMode.Idle(),
            'j_wheel_2': xbot.ControlMode.Idle(),
            'j_wheel_3': xbot.ControlMode.Idle(),
            'j_wheel_4': xbot.ControlMode.Idle()
            }
robot.setControlMode(ctrl_map)

q0 = robot.getMotorPosition()
robot.setPositionReference(q0)
print robot.eigenToMap(q0)

rospy.init_node('centauro_replay')
rate = rospy.Rate(1000.)

joint_names = ('hip_yaw_1', 'hip_pitch_1', 'knee_pitch_1',
               'hip_yaw_2', 'hip_pitch_2', 'knee_pitch_2',
               'hip_yaw_3', 'hip_pitch_3', 'knee_pitch_3',
               'hip_yaw_4', 'hip_pitch_4', 'knee_pitch_4')

# while not rospy.is_shutdown():
for k in range(len(q)):

    q_map = {}
    for i in range(len(joint_names)):
        q_map[joint_names[i]] = q[k][i]

    qdot_map = {}
    for i in range(len(joint_names)):
        qdot_map[joint_names[i]] = qdot[k][i]

    tau_map = {}
    for i in range(len(joint_names)):
        tau_map[joint_names[i]] = tau[k][i]

    robot.setPositionReference(q_map)
    # robot.setVelocityReference(qdot_map)
    # robot.setEffortReference(tau_map)

    robot.move()

    rate.sleep()
