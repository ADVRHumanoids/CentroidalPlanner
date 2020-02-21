def run_hand(robot, ft, direction, magnitude) :


    detect_bool = 0
    wrench = ft.getWrench()
    wrench[direction] = 0
    print wrench[direction]

    if (wrench[direction] >= magnitude) :
        detect_bool = 1

    return detect_bool

def run_foot(robot, ft, direction, magnitude) :


    detect_bool = 0
    wrench = ft.getWrench()
    # print wrench[direction]
    print wrench[direction]

    if (- wrench[direction] >= magnitude) :
        detect_bool = 1

    return detect_bool