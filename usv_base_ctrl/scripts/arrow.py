#!usr/bin/python
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from nav_msgs.msg import Odometry

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from sailboat_controller import SailboatController

import rospy
import math
import tf

sbc = SailboatController()

OriginState = ModelState()
OriginState.model_name = 'arrow'
OriginState.reference_frame = "world"
OriginState.twist = Twist(Point(0, 0, 0), Point(0, 0, 0))


def Arrow():
    rospy.init_node('arrow_node')
    rate = rospy.Rate(10)  # 10h

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state_gazebo_srv = rospy.ServiceProxy(
        '/gazebo/set_model_state', SetModelState)

    wind_x, wind_y = rospy.get_param(
        '/uwsim/wind/x'), rospy.get_param('/uwsim/wind/y')
    global_dir = math.atan2(wind_y, wind_x)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, global_dir)

    while not rospy.is_shutdown():
        try:
            state = sbc.get_state()
            boat_x, boat_y = state.pose.pose.position.x, state.pose.pose.position.y
            OriginPoint = (boat_x, boat_y, 6)
            OriginState.pose = Pose(
                Point(*OriginPoint), Quaternion(*quaternion))

            set_state_gazebo_srv(OriginState)
            rate.sleep()
        except:
            pass


if __name__ == '__main__':
    try:
        Arrow()
    except rospy.ROSInterruptException:
        pass
