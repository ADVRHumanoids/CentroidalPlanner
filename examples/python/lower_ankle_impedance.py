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

    print 'Stiffness of ankle pitch: ', stiffness[anklePitch]
    print 'Damping of ankle pitch: ', damping[anklePitch]

    stiffness[anklePitch] = 10
    damping[anklePitch] = 1

    robot.setStiffness(stiffness)
    robot.setDamping(damping)

    print "New stiffness of ankle pitch: ", robot.getStiffnessMap()[anklePitch]
    print "New damping of ankle pitch: ", robot.getDampingMap()[anklePitch]

    robot.move()