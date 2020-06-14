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

sbc = SailboatController("sailboat_cuhksz")
quater = tf.transformations.quaternion_from_euler(0, 0, 1.5708)
sbc.OriginState.pose = Pose(Point(*sbc.OriginPoint), Quaternion(*quater))
f = sbc.createFile("0.csv")

euler_pub = rospy.Publisher(
    sbc.OriginState.model_name + '/euler', Float32, queue_size=10)
energy_pub = rospy.Publisher(
    sbc.OriginState.model_name + '/energy', Float32MultiArray, queue_size=10)
linear_x_pub = rospy.Publisher(
    sbc.OriginState.model_name + '/linear_x', Float32, queue_size=10)
linear_y_pub = rospy.Publisher(
    sbc.OriginState.model_name + '/linear_y', Float32, queue_size=10)

heading = 0


def model(state, goal):
    pred_rudder = [0, 0, 0]
    pred_sail = [0, 0, 0]
    return pred_rudder, pred_sail


def energyModel(rP, sP, prP, psP, dt):
    state = sbc.get_state()
    quaternion = (state.pose.pose.orientation.x, state.pose.pose.orientation.y,
                  state.pose.pose.orientation.z, state.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    euler_pub.publish(euler[2])  # NOTE:Publish
    target_angle = math.degrees(euler[2])

    tol = 0.000  # 0.001 tol between motor
    duration = 0.0  # 0.1 time
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
            linear_x = sbc.get_state().twist.twist.linear.x
            linear_y = sbc.get_state().twist.twist.linear.y
            linear_x_pub.publish(linear_x)  # NOTE:Publish
            linear_y_pub.publish(linear_y)  # NOTE:Publish
            sbc.writeFile(f, linear_x, linear_y, *energy)

            rP, sP = prP, psP
            rate.sleep()
        except rospy.ROSInterruptException:
            sbc.closeFile(f)
            rospy.logerr(
                "ROS Interrupt Exception! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
            sbc.closeFile(f)
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")
