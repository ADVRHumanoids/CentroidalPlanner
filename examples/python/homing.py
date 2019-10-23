from cartesian_interface.pyci_all import *

def run(ci) :

    duration = 1.
    straight_com = ci.getPoseFromTf('ci/com', 'ci/world_odom').translation
    straight_com[0] += 0.0
    straight_com = Affine3(pos=straight_com)
    print "homing ...."
    ci.setTargetPose('com', straight_com, duration)
    ci.waitReachCompleted('com')
    ci.update()

    # foot_one = ci.getPoseReference(feet_list[0])[0].translation
    # foot_two = ci.getPoseReference(feet_list[1])[0].translation
    #
    # foot_ci_one = Affine3(pos=foot_one)
    # foot_ci_two = Affine3(pos=foot_two)
    #
    # ci.setTargetPose(feet_list[0], foot_ci_one, 5)
    # ci.setTargetPose(feet_list[1], foot_ci_two, 5)
    #
    # ci.waitReachCompleted(feet_list[0])
    # ci.waitReachCompleted(feet_list[1])
    # ci.update()

    ci.setControlMode('Waist', pyci.ControlType.Disabled)
    # ci.setControlMode('torso', pyci.ControlType.Disabled)
