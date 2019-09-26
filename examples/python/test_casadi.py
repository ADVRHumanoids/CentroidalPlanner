#!/usr/bin/env python
from casadi import *
import matlogger2.matlogger as matl
import centroidal_planner.pycpl_casadi as cpl_cas
import rospy

logger = matl.MatLogger2('/tmp/opt_control_log')
logger.setBufferMode(matl.BufferMode.CircularBuffer)

urdf = rospy.get_param('robot_description')

fk_string = cpl_cas.generate_forward_kin(urdf, 'Contact4')
FK = Function.deserialize(fk_string)

id_string = cpl_cas.generate_inv_dyn(urdf)
ID = Function.deserialize(id_string)

print('Exiting..')
