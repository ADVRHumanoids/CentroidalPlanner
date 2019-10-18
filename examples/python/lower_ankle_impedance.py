def run(robot, end_effector) :

    if end_effector == 'l_sole':
        anklePitch = 'LAnklePitch'
    elif end_effector == 'r_sole':
        anklePitch = 'RAnklePitch'
    else:
        print 'WRONG FOOT'

    # LOWERING STIFFNESS AND DAMPING
    print "lowering stiffness and impedance of ankle ..."
    stiffness = robot.getStiffnessMap()
    damping = robot.getDampingMap()

    stiffness[anklePitch] = 10
    damping[anklePitch] = 1

    robot.setStiffness(stiffness)
    robot.setDamping(damping)

    print "Stiffness of robot:", robot.getStiffnessMap
    print "Damping of robot: ", robot.getDampingMap

    robot.move()