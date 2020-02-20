import cartesio_planning.pyplan as cpp
import query_yes_no as query_yn
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
from cartesian_interface.pyci_all import *
import numpy as np

def rotation(theta):

    tx, ty, tz = theta

    Rx = np.array([[1, 0, 0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]])
    Ry = np.array([[np.cos(ty), 0, -np.sin(ty)], [0, 1, 0], [np.sin(ty), 0, np.cos(ty)]])
    Rz = np.array([[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0, 0, 1]])

    return np.dot(Rx, np.dot(Ry, Rz))


if __name__ == '__main__':

    np.set_printoptions(precision=3, suppress=True)

    roscpp_init('coman_sheep', [])
    planner = cpp.PlannerClient()

    mat = rotation([0, 0, 0])

    rot_mat = Affine3()
    rot_mat.linear = mat

    print rot_mat.quaternion
    frames_in_contact = ["TCP_L", "TCP_R", "r_sole"]
    normals = [rot_mat.quaternion, rot_mat.quaternion, rot_mat.quaternion]

    planner.setContactFrames('SET', frames_in_contact, 1, True, normals)


