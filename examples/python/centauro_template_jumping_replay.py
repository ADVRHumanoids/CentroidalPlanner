#!/usr/bin/env python
import numpy as np
import matlogger2.matlogger as matl
from cartesian_interface.pyci_all import *
import h5py
import rospy

logger = matl.MatLogger2('/tmp/centauro_template_jumping_log')
logger.setBufferMode(matl.BufferMode.CircularBuffer)

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
