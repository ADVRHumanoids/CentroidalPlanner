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
    stiffness = robot.getStiffnessMap()
    damping = robot.getDampingMap()

    stiffness[anklePitch] = 1
    damping[anklePitch] = 0.1

    stiffness[ankleRoll] = 1
    damping[ankleRoll] = 0.1


    robot.setStiffness(stiffness)
    robot.setDamping(damping)

    print "Stiffness of ankle: Pitch -> ", robot.getStiffnessMap()[anklePitch], " Roll -> ", robot.getStiffnessMap()[ankleRoll]
    print "Damping of ankle: Pitch -> ", robot.getDampingMap()[anklePitch], " Roll -> ", robot.getDampingMap()[ankleRoll]

    robot.move()