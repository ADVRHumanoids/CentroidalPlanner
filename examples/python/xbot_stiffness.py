import rospy
import xbot_interface.xbot_interface as xbot
import numpy as np

N_ITER = 50

def set_legs_initial_stiffness(robot) :

    # robot.setControlMode(xbot.ControlMode.Stiffness() + xbot.ControlMode.Damping())

    # print robot.getEnabledJointNames
    print robot.leg(0).getJointNames()
    # print robot.leg(0).getBaseLinkName()
    # print robot.leg(0).getStiffness()
    # print robot.leg(0).getJointNum()

    K_0 = robot.leg(0).getStiffness()
    K_end = [1000, 1000, 1000, 1000, 1000, 1000]

    global N_ITER
    N_LEGS = 2
    for k in range(N_ITER) :
        for i in range(N_LEGS) :
            leg_k = np.array(robot.leg(i).getJointNum())
            leg_k = K_0 + float(k) / (N_ITER - 1) * (K_end - K_0)
            robot.leg(i).setStiffness(leg_k)

        print "Completed: ", float(k) / N_ITER * 100, "%"
        robot.move()
        rospy.sleep(0.01)

    for i in range(N_LEGS) :
        print "Stiffness of ", robot.leg(i).getChainName(), " is: ", robot.leg(i).getStiffness()

def set_legs_default_stiffness(robot, K_end) :

    # robot.setControlMode(xbot.ControlMode.Stiffness() + xbot.ControlMode.Damping())

    K_0 = robot.leg(0).getStiffness()

    global N_ITER
    N_LEGS = 2

    for k in range(N_ITER):
        for i in range(N_LEGS):
            leg_k = np.array(robot.leg(i).getJointNum())
            leg_k = K_0 + float(k) / (N_ITER - 1) * (K_end - K_0)
            robot.leg(i).setStiffness(leg_k)

        # print "Completed: ", float(k)/N_ITER * 100, "%"
        robot.move()
        rospy.sleep(0.01)

    for i in range(N_LEGS):
        print "Stiffness of ", robot.leg(i).getChainName(), " is: ", robot.leg(i).getStiffness()

def set_legs_low_stiffness(robot) :

    # robot.setControlMode(xbot.ControlMode.Stiffness() + xbot.ControlMode.Damping())

    K_0 = robot.leg(0).getStiffness()
    K_end = [50, 50, 50, 50, 50, 50]

    global N_ITER
    N_LEGS = 2
    for k in range(N_ITER) :
        for i in range(N_LEGS) :
            leg_k = np.array(robot.leg(i).getJointNum())
            leg_k = K_0 + float(k) / (N_ITER - 1) * (K_end - K_0)
            robot.leg(i).setStiffness(leg_k)

        # print "Completed: ", float(k) / N_ITER * 100, "%"
        robot.move()
        rospy.sleep(0.01)

    for i in range(N_LEGS) :
        print "New Stiffness of ", robot.leg(i).getChainName(), " is: ", robot.leg(i).getStiffness()


def set_arms_default_stiffness(robot, K_end) :

    # robot.setControlMode(xbot.ControlMode.Stiffness() + xbot.ControlMode.Damping())
    K_0 = robot.arm(0).getStiffness()

    global N_ITER
    N_ARMS = 2

    for k in range(N_ITER):
        for i in range(N_ARMS):
            arm_k = np.array(robot.arm(i).getJointNum())
            arm_k = K_0 + float(k) / (N_ITER - 1) * (K_end - K_0)
            robot.arm(i).setStiffness(arm_k)

        # print "Completed: ", float(k) / N_ITER * 100, "%"
        robot.move()
        rospy.sleep(0.01)

    for i in range(N_ARMS):
        print "New Stiffness of ", robot.arm(i).getChainName(), " is: ", robot.arm(i).getStiffness()

def set_arms_low_stiffness(robot) :

    # robot.setControlMode(xbot.ControlMode.Stiffness() + xbot.ControlMode.Damping())

    K_0 = robot.arm(0).getStiffness()
    K_end = [50, 50, 50, 50, 50, 50, 50]


    global N_ITER
    N_ARMS = 2
    for k in range(N_ITER) :
        for i in range(N_ARMS) :
            arm_k = np.array(robot.arm(i).getJointNum())
            arm_k = K_0 + float(k) / (N_ITER - 1) * (K_end - K_0)
            robot.arm(i).setStiffness(arm_k)

        # print "Completed: ", float(k) / N_ITER * 100, "%"
        robot.move()
        rospy.sleep(0.01)

    for i in range(N_ARMS) :
        print "New Stiffness of ", robot.arm(i).getChainName(), " is: ", robot.arm(i).getStiffness()



def set_leg_stiffness(robot, ee, K_end) :

    if ee == 'l_sole' :
        selected_leg = robot.leg(0)
    elif ee == 'r_sole' :
        selected_leg = robot.leg(1)

    K_0 = selected_leg.getStiffness()
    #
    global N_ITER

    for k in range(N_ITER):

        leg_values = np.array(selected_leg.getJointNum())
        leg_values = K_0 + float(k) / (N_ITER - 1) * (K_end - K_0)
        selected_leg.setStiffness(leg_values)

        # print "Completed: ", float(k) / N_ITER * 100, "%"
        robot.move()
        rospy.sleep(0.01)

    print "New Stiffness of ", selected_leg.getChainName(), " is: ", selected_leg.getStiffness()