def run(robot, ft, direction, magnitude) :


    detect_bool = 0
    wrench = ft.getWrench()

    if (wrench[direction] >= magnitude) :
        detect_bool = 1

    return detect_bool