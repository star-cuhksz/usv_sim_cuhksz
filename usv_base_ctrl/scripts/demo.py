from std_srvs.srv import Empty
import rospy

from sailboat_controller import SailboatController

sbc = SailboatController()


def model(state):
    rP, rV, rE, sP, sV, sE = [50]*6
    print state
    # print sbc.get_param_names()
    pred_rudder = [rP, rV, rE]
    pred_sail = [sP, sV, sE]
    return pred_rudder, pred_sail


def Demo():
    rospy.init_node('sbc')
    rate = rospy.Rate(10)  # 10h

    # Recommended that reset when running the script.
    # sbc.reset()
    while not rospy.is_shutdown():
        try:
            sbc.pub_state(*model(sbc.get_state()))
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr(
                "ROS Interrupt Exception! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")


if __name__ == '__main__':
    try:
        Demo()
    except rospy.ROSInterruptException:
        pass
