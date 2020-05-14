from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from std_srvs.srv import Empty
import rospy
import math
import tf

from sailboat_controller import SailboatController

sbc = SailboatController("sailboat_cuhksz")
sbc.OriginState.pose = Pose(Point(*sbc.OriginPoint), Quaternion(0, 0, 0, 1))
f = sbc.createFile("fsf.txt")


def angle_saturation(sensor):
    if sensor > 180:
        sensor = sensor - 360
    if sensor < -180:
        sensor = sensor + 360
    return sensor


def get_sail_position(current_heading):
    x = rospy.get_param('/uwsim/wind/x')
    y = rospy.get_param('/uwsim/wind/y')

    global_dir = math.degrees(math.atan2(y, x))
    # heeling = angle_saturation(math.degrees(global_dir)+180)
    wind_dir = global_dir - current_heading
    wind_dir = angle_saturation(wind_dir+180)
    sail_angle = math.radians(wind_dir)/2
    if math.degrees(sail_angle) < -80:
        sail_angle = -sail_angle

    return -sail_angle


def get_rudder_position(state, goal):
    # solve goalSailboatAcuteAngle
    sailboatX = state.pose.pose.position.x
    sailboatY = state.pose.pose.position.y
    goalX = goal.pose.pose.position.x
    goalY = goal.pose.pose.position.y
    sub_x, sub_y = goalX - sailboatX, goalY - sailboatY
    goalSailboatAcuteAngle = math.degrees(math.atan2(sub_y, sub_x))
    goalSailboatAcuteAngle = -angle_saturation(goalSailboatAcuteAngle)
    print "goalSailboatAcuteAngle : ", goalSailboatAcuteAngle

    # solve currentYaw
    quaternion = (state.pose.pose.orientation.x, state.pose.pose.orientation.y,
                  state.pose.pose.orientation.z, state.pose.pose.orientation.w)
    # [roll, pitch, yaw]
    stateYawRad = tf.transformations.euler_from_quaternion(quaternion)[2]
    currentYaw = math.degrees(stateYawRad)
    currentYaw = -angle_saturation(currentYaw)

    # solve rudderAngle
    err = angle_saturation(goalSailboatAcuteAngle - currentYaw)
    # err = err + 0
    rudderAngle = err/2

    targetDistance = math.hypot(sub_x, sub_y)
    goalSailboatAcuteAngle = -goalSailboatAcuteAngle

    rP = math.radians(rudderAngle)
    return rP, currentYaw


def model(state, goal):
    rP, currentYaw = get_rudder_position(state, goal)
    sP = get_sail_position(currentYaw)

    print "rP, sP", rP, sP

    sbc.writeFile(f, currentYaw, rp, sP)

    pred_rudder = [rP, 0, 0]
    pred_sail = [sP, 0, 0]
    return pred_rudder, pred_sail


def Demo():
    rospy.init_node('sbc')
    rate = rospy.Rate(10)  # 10h

    # Recommended that reset when running the script.
    sbc.show_log(isShow=False)
    sbc.pub_goal((35, 0))
    sbc.reset()
    while not rospy.is_shutdown():
        try:
            sbc.pub_state(*model(sbc.get_state(), sbc.get_goal()))
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
