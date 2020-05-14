import matplotlib.pyplot as plt
import rospy
import math


def drawArrow(A, B):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.title("wind vane")
    """
    arrow start point (A[0],A[1]) end point (B[0],B[1])
    length_includes_head = True
    fc:filling color
    ec:edge color
    """
    ax.arrow(A[0], A[1], B[0]-A[0], B[1]-A[1], length_includes_head=True,
             head_width=0.05, head_length=0.15, fc='g', ec='g')
    ax.set_xlim(1, -1)  # set around of canvas, default [0,1]
    ax.set_ylim(-1, 1)
    ax.grid()
    ax.set_aspect('equal')
    plt.show()
    plt.tight_layout()


if __name__ == "__main__":
    rospy.init_node("Canvas")
    rate = rospy.Rate(10)

    wind_x = rospy.get_param('/uwsim/wind/x')
    wind_y = rospy.get_param('/uwsim/wind/y')

    rad = math.atan2(wind_x, wind_y)
    x = math.sqrt(1 / (math.pow(rad, 2) + 1))
    y = math.sqrt(1 - math.pow(x, 2))
    drawArrow([0, 0], [x, y])
    # rospy.Subscriber("sailboat_cuhksz/joint_setpoint", JointState)
    # rospy.Subscriber("sailboat_cuhksz/goal", JointState)
