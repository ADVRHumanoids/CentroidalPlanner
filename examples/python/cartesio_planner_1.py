#!/usr/bin/env python
import cartesio_planning.pyplan as cpp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import query_yes_no as query_yn

def compute(joint_map_start, joint_map_goal, frames_in_contact, max_time, planner_type, interpolation_time, traj_space = "Joint", distal_link = [], base_link = [], normals = []) :

    planner = cpp.PlannerClient()

    planner.setStartState(joint_map_start)
    planner.setGoalState(joint_map_goal)



    planner.setContactFrames('SET', frames_in_contact, 0.5, False, normals)

    raw_input('check if start and goal position are good, then press any Key.')
    print 'RRT started planning...'

    flag_ok = False
    while (not flag_ok) :
        planner.callPlanner(max_time, planner_type, interpolation_time, traj_space, distal_link, base_link)
        flag_ok = query_yn.ask('Is this trajectory good for you?')


    if traj_space == "Joint" :
        return planner.getJointTrajectory()
    elif traj_space == "Cartesian" :
        return planner.getCartesianTrajectory()





