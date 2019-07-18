from cartesian_interface.pyci_all import *
import centroidal_planner.pycpl as cpl
import numpy as np
import xbot_interface.xbot_interface as xb
import xbot_interface.config_options as copt


def main():

    # get robot


    # get cartesio ros client
    ci = pyci.CartesianInterfaceRos()

    # set up cpl com planner
    contacts = ['wheel_1', 'wheel_2', 'wheel_3', 'wheel_4']
    mass = 90.0
    mu = 0.5
    com_pl = cpl.CoMPlanner(contacts, mass)
    com_pl.SetMu(mu)
    com_pl.SetCoMWeight(1000000.0)
    com_pl.SetForceWeight(0.0000001)
    for c in contacts:
        com_pl.SetForceThreshold(c, 15.0)

    # get current com from cartesio and set it to planner
    com_ref = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    print 'Desired com is ', com_ref
    com_pl.SetCoMRef(com_ref)

    # get contacts references from cartesio and set them to the planner
    for c in contacts:
        com_pl.SetContactPosition(c, ci.getPoseReference(c)[0].translation)
        print "Setting contact position for ", c, " to ", com_pl.GetContactPosition(c)

    while True:
        for contact_lift in contacts:
            # we lift contact
            com_pl.SetLiftingContact(contact_lift)

            # find solution
            sol = com_pl.Solve()
            print(sol)

            com_pl.ResetLiftingContact(contact_lift)

            # send commands to cartesio
            com_ci = Affine3(pos=sol.com)
            reach_time = 2.0
            ci.setTargetPose('com', com_ci, reach_time)
            ci.waitReachCompleted('com')

            wheel_1_ci = ci.getPoseReference(contact_lift)[0]
            wheel_1_ci.translation_ref()[2] += 0.1
            ci.setTargetPose(contact_lift, wheel_1_ci, reach_time/2.0)
            ci.waitReachCompleted(contact_lift)

            ci.update()

            wheel_1_ci = ci.getPoseReference(contact_lift)[0]
            wheel_1_ci.translation_ref()[2] -= 0.1
            ci.setTargetPose(contact_lift, wheel_1_ci, reach_time/2.0)
            ci.waitReachCompleted(contact_lift)


    print 'Terminating...'



if __name__ == '__main__':

    np.set_printoptions(precision=3, suppress=True)

    main()