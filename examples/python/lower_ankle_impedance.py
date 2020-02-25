import xbot_interface.xbot_interface as xbot

def run(robot, end_effector) :

    if end_effector == 'l_sole':
        anklePitch = 'LAnklePitch'
        ankleRoll = 'LAnkleRoll'
    elif end_effector == 'r_sole':
        anklePitch = 'RAnklePitch'
        ankleRoll = 'RAnkleRoll'
    else:
        print 'WRONG FOOT'

    # LOWERING STIFFNESS AND DAMPING
    print "lowering stiffness and impedance of ankle ..."
    # stiffness = robot.getStiffnessMap()[anklePitch]
    # damping = robot.getDampingMap()

    stiffness_map = dict()
    damping_map = dict()

    stiffness_map[anklePitch] = 1
    damping_map[anklePitch] = 0.1


    stiffness_map[ankleRoll] = 1
    damping_map[ankleRoll] = 0.1

    robot.setStiffness(stiffness_map)
    robot.setDamping(damping_map)

    print "Stiffness of ankle: Pitch -> ", robot.getStiffnessMap()[anklePitch], " Roll -> ", robot.getStiffnessMap()[ankleRoll]
    print "Damping of ankle: Pitch -> ", robot.getDampingMap()[anklePitch], " Roll -> ", robot.getDampingMap()[ankleRoll]

    robot.move()