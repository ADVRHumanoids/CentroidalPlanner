import centroidal_planner.pycpl as cpl

def compute(ci, contacts, hands_list, feet_list, mass, dist_hands, bound_size) :

    env = cpl.Ground()
    env.SetGroundZ(ci.getPoseReference('l_sole')[0].translation[2])

    ctrl_pl = cpl.CentroidalPlanner(contacts, mass, env)

    # ctrl_pl.SetCoMWeight(100000)
    # ctrl_pl.SetPosWeight(100000000)
    # ctrl_pl.SetForceWeight(0)

    # set position reference and bounds for FEET
    for foot in feet_list :
        contact_foot = ci.getPoseReference(foot)[0].translation
        # contact_foot[2] = 0.
        ctrl_pl.SetPosRef(foot, contact_foot)
        print "Setting contact position for",  foot, " to ", ctrl_pl.GetPosRef(foot)
        lower_bound = ctrl_pl.GetPosRef(foot) - bound_size
        upper_bound = ctrl_pl.GetPosRef(foot) + bound_size
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound
        ctrl_pl.SetPositionBound(foot, lower_bound, upper_bound)

    # set position reference and bounds for HANDS
    i = 0
    for hand in hands_list :
        contact_hand = ci.getPoseReference(feet_list[i])[0].translation
        # contact_hand[2] = 0.
        contact_hand[0] += dist_hands[0]
        if hand == "l_ball_tip":
            contact_hand[1] += dist_hands[1]
        else:
            contact_hand[1] -= dist_hands[1]

        i = i+1
        ctrl_pl.SetPosRef(hand, contact_hand)
        print "Setting contact position for",  hand, " to ", ctrl_pl.GetPosRef(hand)
        lower_bound = ctrl_pl.GetPosRef(hand) - bound_size
        upper_bound = ctrl_pl.GetPosRef(hand) + bound_size
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound
        ctrl_pl.SetPositionBound(hand, lower_bound, upper_bound)

    sol_centroidal = ctrl_pl.Solve()

    print sol_centroidal

    return ctrl_pl, sol_centroidal
