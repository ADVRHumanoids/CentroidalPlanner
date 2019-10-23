#!/usr/bin/env python
import numpy as np
from cartesian_interface.pyci_all import *
import h5py
import rospy

f = h5py.File('/home/matteo/advr-superbuild/external/CentroidalPlanner/examples/data_replay/template_jumping.mat', 'r')
q = f.get('q_template_replay')
q = np.array(q)

# get cartesio ros client
ci = pyci.CartesianInterfaceRos()

ci.setVelocityLimits('wheel_1', 1000, 1000)
ci.setVelocityLimits('wheel_2', 1000, 1000)
ci.setVelocityLimits('wheel_3', 1000, 1000)
ci.setVelocityLimits('wheel_4', 1000, 1000)

ci.setAccelerationLimits('wheel_1', 1000, 1000)
ci.setAccelerationLimits('wheel_2', 1000, 1000)
ci.setAccelerationLimits('wheel_3', 1000, 1000)
ci.setAccelerationLimits('wheel_4', 1000, 1000)

wheel1_init_pos = Affine3(pos=q[0][0:3])
wheel2_init_pos = Affine3(pos=q[0][3:6])
wheel3_init_pos = Affine3(pos=q[0][9:12])
wheel4_init_pos = Affine3(pos=q[0][6:9])

reach_time = 3.0

ci.setTargetPose('wheel_1', wheel1_init_pos, reach_time)
ci.setTargetPose('wheel_2', wheel2_init_pos, reach_time)
ci.setTargetPose('wheel_3', wheel3_init_pos, reach_time)
ci.setTargetPose('wheel_4', wheel4_init_pos, reach_time)

ci.waitReachCompleted('wheel_4')

ci.update()

rospy.init_node('centauro_template_jumping_replay')
rate = rospy.Rate(1000.)

for k in range(len(q)):

        wheel1_pos = Affine3(pos=q[k][0:3])
        wheel2_pos = Affine3(pos=q[k][3:6])
        wheel3_pos = Affine3(pos=q[k][9:12])
        wheel4_pos = Affine3(pos=q[k][6:9])

        ci.setPoseReference('wheel_1', wheel1_pos)
        ci.setPoseReference('wheel_2', wheel2_pos)
        ci.setPoseReference('wheel_3', wheel3_pos)
        ci.setPoseReference('wheel_4', wheel4_pos)

        rate.sleep()
