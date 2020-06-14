# system
import time
import math

# ros
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from std_srvs.srv import Empty
from std_msgs.msg import Float32, Float32MultiArray
import rospy
import tf

# self
from sailboat_controller import SailboatController

# sbc = SailboatController("sailboat", origin=(240, 100, 1.13))
sbc = SailboatController()
sbc.OriginState.pose = Pose(Point(*sbc.OriginPoint), Quaternion(0, 0, 0, 1))

euler_pub = rospy.Publisher(
    sbc.OriginState.model_name + '/euler', Float32, queue_size=10)
energy_pub = rospy.Publisher(
    sbc.OriginState.model_name + '/energy', Float32MultiArray, queue_size=10)

Kp = 1
Ki = 0
rate_value = 10
Interior = 0
heading = 0


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

    sail_angle = 0

    # print "current_heading: ", current_heading
    # print "x, y: ", x, y
    # print "global_dir: ", global_dir
    # print "wind_dir: ", wind_dir
    # print "sail_angle: ", sail_angle
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
    global heading
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
    euler_pub.publish(euler[2])  # NOTE:Publish
    target_angle = math.degrees(euler[2])
    # print "current yaw before : ", target_angle
    # target_angle = angle_saturation(target_angle + 90)
    # print "current yaw after : ", target_angle

    sp_angle = angle_saturation(sp_angle)
    spHeading = sp_angle
    sp_angle = -sp_angle
    target_angle = angle_saturation(target_angle)
    target_angle = -target_angle
    current_heading = math.radians(target_angle)
    err = angle_saturation(sp_angle - target_angle)
    # err = P(Kp, err) + I(Ki, Interior, rate_value, err)

    rudder_angle = err/2

    # rP = math.radians(rudder_angle)
    rP = 0

    heading = current_heading
    return rP, current_heading


def model(state, goal):
    # print goal
    rP, current_heading = get_rudder_position(state, goal)
    sP = get_sail_position(current_heading)

    print "rP, sP", rP, sP
    # print "state: ", state

    pred_rudder = [0, 0, 0]
    pred_sail = [0, 0, 0]
    return pred_rudder, pred_sail


def energyModel(rP, sP, prP, psP, dt):
    global heading
    tol = 0.001  # 0.001 tol between motor
    duration = 0.1  # 0.1 time
    baseV = 26.6  # V
    baseAr = 0.7  # A
    peakAr = 1.4  # A
    baseAs = 0.7  # A
    peakAs = 1.4  # A
    # This is the 1.8 degree, which corresponding to the tolerance of motor.
    if abs(rP - prP) > tol:
        if dt < duration:  # The duration of start motor
            baseAr = peakAr * dt / duration
        else:
            baseAr = peakAr
    # This is the 1.8 degree, which corresponding to the tolerance of motor.
    if abs(sP - psP) > 0.01:
        if dt < duration:  # The duration of start motor
            baseAs = peakAs * dt / duration
        else:
            baseAs = peakAs

    energyr = baseAr*baseV*dt
    energyWr = energyr/dt
    energys = baseAs*baseV*dt
    energyWs = energys/dt
    return energyr, energys, energyWr, energyWs, sP, rP, heading, dt


if __name__ == '__main__':
    rospy.init_node('sbc')
    rate = rospy.Rate(10)  # 10h

    # Recommended that reset when running the script.
    sbc.show_log(isShow=False)
    sbc.pub_goal((32, 0))
    sbc.reset()
    rP, sP = model(sbc.get_state(), sbc.get_goal())  # Initial the rP, sP
    while not rospy.is_shutdown():
        try:
            # Add the energy model part
            start = time.time()
            prP, psP = model(sbc.get_state(), sbc.get_goal())
            sbc.pub_state(prP, psP)
            time.sleep(0.1)
            dt = time.time() - start
            energy = energyModel(rP[0], sP[0], prP[0], psP[0], dt)
            energylist = Float32MultiArray(data=energy)
            energy_pub.publish(energylist)  # NOTE:Publish
            rP, sP = prP, psP
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr(
                "ROS Interrupt Exception! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")
