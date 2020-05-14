#!/usr/bin/env python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import rospy
import sys
import select
import termios
import tty
import roslib
from sailboat_controller import SailboatController
# roslib.load_manifest('keyboard_teleop')

msg = """
USV-SIM, CUHKSZ
referee: https://github.com/ros-teleop

Keyboard Teleop
---------------------------
Reading from the keyboard  and Publishing to Twist and joint controllers!
Infantry base controller and cradlehead controller
---------------------------
Moving around:
   q         e
   a         d
---------------------------
For Holonomic mode (strafing), hold down the shift key:
   Q         E
   A         D
---------------------------
anything else : stop

r/v : increase/decrease max speeds by 10%
t/b : increase/decrease only linear speed by 10%
y/n : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'q': (0.0175, 0),
    'e': (-0.0175, 0),
    'a': (0, 0.0175),
    'd': (0, -0.0175),
    'Q': (0.0175, 0),
    'E': (-0.0175, 0),
    'A': (0, 0.0175),
    'D': (0, -0.0175),
}

speedBindings = {
    'r': (1.1, 1.1),
    'v': (.9, .9),
    't': (1.1, 1),
    'b': (.9, 1),
    'y': (1, 1.1),
    'n': (1, .9),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def printlog(rudder_p, rudder_v, sail_p, sail_v):
    # return "currently:\n\trudder_p %s\trudder_v %s\n\tsail_p %s\tsail_v %s" % (rudder_p, rudder_v, sail_p, sail_v)
    return "currently:\n\trudder_p %s\n\tsail_p %s" % (rudder_p, sail_p) # , sail_p, sail_v)


def iff(a, b):
    if a > b:
        a = b
    if a < -b:
        a = -b
    return a


if __name__ == "__main__":
    sbc = SailboatController("sailboat_cuhksz")
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('usvsim_teleop_keyboard')
    set_state_pub = rospy.Publisher(
        'sailboat_cuhksz/joint_setpoint', JointState, queue_size=10)

    sbc.reset()
    rudder_p, sail_p, rudder_v, sail_v = 0, 0, 1, 1
    status = 0
    try:
        print msg
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                rudder_p += moveBindings[key][0]
                sail_p += moveBindings[key][1]

            elif key in speedBindings.keys():
                rudder_v *= speedBindings[key][0]
                sail_v *= speedBindings[key][1]

            else:
                if (key == '\x03'):
                    break

            rudder_p = iff(rudder_p, 1.570796)
            sail_p = iff(sail_p, 1.570796)
            print printlog(rudder_p, rudder_v, sail_p, sail_v)

            msg = JointState()
            msg.header = Header()
            msg.name = ['rudder_joint', 'sail_joint']
            msg.position = [rudder_p, sail_p]
            msg.velocity = [rudder_v, sail_v]
            msg.effort = [0, 0]
            set_state_pub.publish(msg)

    finally:
        msg = JointState()
        msg.header = Header()
        msg.name = ['rudder_joint', 'sail_joint']
        msg.position = [0, 0]
        msg.velocity = [0, 0]
        msg.effort = [0, 0]
        set_state_pub.publish(msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
