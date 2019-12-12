import centroidal_planner.pycpl as cpl
import numpy as np

def compute(ci, contacts, hands_list, feet_list, mass, mu_feet) :


    print "starting OPTIMIZATION for foot positioning on wall .."

    # get x position of feet
    contact_foot_x = ci.getPoseReference(feet_list[0])[0].translation[0]
    # height of hand
    contact_hand_z = ci.getPoseReference(hands_list[0])[0].translation[2]

    # p: point from which the superquadric is computed
    p = [contact_foot_x, 0, contact_hand_z]   # point in between the two soles, at height of hands
    print "point from which the superquadric is computed: ", p

    # distance of wall from p
    wall_distance = 0.7 #x

    # distance of ground from p
    ground_distance = 0.0 #z

    # d: p w.r.t superquadric
    d = [wall_distance, 0.0,  ground_distance]
    print "p w.r.t superquadric: ", d

    # height from superquadricd
    height_contact = 0.3
    # distance foot from center (how distanced feet are from the center of the superquadric)
    d_feet = ci.getPoseReference("l_sole")[0].translation[1] + ci.getPoseReference("l_sole")[0].translation[1]

    # set bounds
    bound_size_wall = 0.1
    bound_size_hands = 0.1

    superquadric_env = cpl.Superquadric()

    superquadric_env.SetMu(mu_feet)


    C_translation = [10, 0, 10]

    radius = [10, 10, 10]

    C = np.array([p[0] + C_translation[0], p[1] + C_translation[1], p[2] + C_translation[2]])  # center of superquadric
    R = np.array([d[0] + radius[0], d[1] + radius[1], d[2] + radius[2]]) # radius of superquadric
    P = np.array([50.0, 50.0, 50.0])

    superquadric_env.SetParameters(C,R,P)

    print "center of superquadric: ", C
    print "radius of superquadric: ", R

    # create a centroidal planner
    ctrl_pl = cpl.CentroidalPlanner(contacts, mass, superquadric_env)

    # set hands in world
    for hand_i in hands_list:
        ci.setBaseLink(hand_i, 'world')

    # ctrl_pl.SetCoMWeight(100000)
    # ctrl_pl.SetPosWeight(100000000)
    # ctrl_pl.SetForceWeight(0)

    # set position reference and bounds for HANDS
    for hand in hands_list:
        contact_hand = ci.getPoseReference(hand)[0].translation
        ctrl_pl.SetPosRef(hand, contact_hand)

        print "Setting contact position for", hand, " to ", ctrl_pl.GetPosRef(hand)
        lower_bound = ctrl_pl.GetPosRef(hand) - bound_size_hands
        upper_bound = ctrl_pl.GetPosRef(hand) + bound_size_hands
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound
        ctrl_pl.SetPositionBound(hand, lower_bound, upper_bound)

    print "current_foot_position: ", ci.getPoseReference("l_sole")[0].translation
    print "height contact: ", height_contact

    # # set position reference and bounds for FEET
    for foot in feet_list:

        if foot == 'l_sole' :
            dist = d_feet/2
        else :
            dist = - d_feet/2

        contact_foot = [p[0] - d[0], p[1] - d[1] + dist, p[2] - d[2] + height_contact]
        p = [contact_foot_x, 0, contact_hand_z]
        ctrl_pl.SetPosRef(foot, contact_foot)
        print "Setting contact position for", foot, " to ", ctrl_pl.GetPosRef(foot)
        lower_bound = ctrl_pl.GetPosRef(foot) - bound_size_wall
        upper_bound = ctrl_pl.GetPosRef(foot) + bound_size_wall
        print "Lower bound: ", lower_bound
        print "Upper bound: ", upper_bound
        ctrl_pl.SetPositionBound(foot, lower_bound, upper_bound)

    sol_centroidal = ctrl_pl.Solve()

    print sol_centroidal

    return ctrl_pl, sol_centroidal