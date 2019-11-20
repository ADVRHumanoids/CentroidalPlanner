def run(robot, ft, direction, magnitude) :


    detect_bool = 0
    wrench = ft.getWrench()
    # print wrench[direction]

    if (wrench[direction] >= magnitude) :
        detect_bool = 1

    return detect_bool