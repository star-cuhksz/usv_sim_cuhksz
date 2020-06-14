from sensor_msgs.msg import JointState
import rospy

from sailboat_controller import SailboatController

sbc = SailboatController()


def TEST():
    rospy.init_node('test')
    rate = rospy.Rate(10)  # 10hz

    rospy.Subscriber("sailboat_cuhksz/joint_states", JointState,)

    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException:
            rospy.logerr(
                "ROS Interrupt Exception! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")


if __name__ == '__main__':
    try:
        TEST()
    except rospy.ROSInterruptException:
        pass
