#!/usr/bin/env python
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import rospy
import math
import tf

from sailboat_controller import SailboatController

sbc = SailboatController("sailboat_cuhksz", origin=(240, 100, 0))
sbc.OriginState.pose = Pose(Point(*sbc.OriginPoint), Quaternion(0, 0, 0, 1))

waypoints = [
    [(250.0, 95.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(255.0, 100.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(260.0, 105.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(265.0, 100.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(270.0, 95.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(275.0, 100.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(270.0, 105.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(265.0, 100.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(260.0, 95.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(255.0, 100.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(250.0, 105.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(245.0, 100.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
]

Kp = 1
Ki = 0
rate_value = 10
Interior = 0
target_distance = 999


def angle_saturation(sensor):
    if sensor > 180:
        sensor = sensor - 360
    if sensor < -180:
        sensor = sensor + 360
    return sensor


def get_sail_position(current_heading):
    x = rospy.get_param('/uwsim/wind/x')
    y = rospy.get_param('/uwsim/wind/y')

    global_dir = math.atan2(y, x)
    heeling = angle_saturation(math.degrees(global_dir)+180)
    wind_dir = global_dir - current_heading
    wind_dir = angle_saturation(math.degrees(wind_dir)+180)

    sail_angle = math.radians(wind_dir)/2
    print "current_heading: ", current_heading
    print "sail_angle: ", sail_angle
    if math.degrees(sail_angle) < -80:
        sail_angle = -sail_angle

    return -sail_angle


def P(Kp, error):
    return Kp * error


def I(Ki, Interior, rate_value, error):
    if (Interior > 0 and error < 0) or (Interior < 0 and error > 0):
        Interior = Interior + Ki * error * 50 * (1./rate_value)
    else:
        Interior = Interior + Ki * error * (1./rate_value)
    return Interior


def get_rudder_position(state, goal):
    global target_distance
    x1 = state.pose.pose.position.x
    y1 = state.pose.pose.position.y
    x2 = goal.pose.pose.position.x
    y2 = goal.pose.pose.position.y

    radians = math.atan2(y2-y1, x2-x1)
    sp_angle = math.degrees(radians)

    target_distance = math.hypot(x2-x1, y2-y1)
    quaternion = (state.pose.pose.orientation.x, state.pose.pose.orientation.y,
                  state.pose.pose.orientation.z, state.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    target_angle = math.degrees(euler[2])

    sp_angle = angle_saturation(sp_angle)
    spHeading = sp_angle
    sp_angle = -sp_angle
    target_angle = angle_saturation(target_angle)
    target_angle = -target_angle
    current_heading = math.radians(target_angle)

    err = sp_angle - target_angle
    err = angle_saturation(err)
    err = P(Kp, err) + I(Ki, Interior, rate_value, err)

    rudder_angle = err/2

    if err > 60:
        err = 60
    if err < -60:
        err = -60

    rP = math.radians(rudder_angle)
    return rP, current_heading


def model(state, goal):
    # print goal
    rP, current_heading = get_rudder_position(state, goal)
    sP = get_sail_position(current_heading)

    print "rP, sP", rP, sP
    # print "state: ", state

    pred_rudder = [rP, 0, 0]
    pred_sail = [sP, 0, 0]
    return pred_rudder, pred_sail


def isArrvied(target_distance):
    f_distance = 4
    if target_distance < f_distance:
        result = True
    if target_distance >= f_distance:
        result = False
    return result


def Scene2():
    rospy.init_node('sbc')
    rate = rospy.Rate(10)  # 10h

    waypoint = iter(waypoints)
    # Recommended that reset when running the script.
    sbc.show_log()
    sbc.pub_goal(next(waypoint), absolute=True)
    sbc.reset()
    while not rospy.is_shutdown():
        try:
            print "outside target_distance : ", target_distance
            if isArrvied(target_distance):
                print "is arrvied? :", isArrvied(target_distance)
                sbc.pub_goal(next(waypoint), absolute=True)
            sbc.pub_state(*model(sbc.get_state(), sbc.get_goal()))
            rate.sleep()
        except StopIteration:
            rospy.loginfo("Scene 2 had finished, shutdown now")
            rospy.on_shutdown(0)
        except rospy.ROSInterruptException:
            rospy.logerr(
                "ROS Interrupt Exception! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")


if __name__ == '__main__':
    try:
        Scene2()
    except rospy.ROSInterruptException:
        pass
