from water_current.msg import Reset
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_srvs.srv import Empty

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import rospy
import sys
import os
import subprocess
import math
import tf
import argparse
arg = argparse.ArgumentParser("For Sailboat Controller")


class SailboatController():
    def __init__(self, namespace="sailboat_cuhksz",
                 origin=(240, 95, 1.13),
                 quaternion=(0, 0, 0, 1)):
        self.OriginPoint = origin
        self.OriginState = ModelState()
        self.OriginState.model_name = namespace
        self.OriginState.reference_frame = "world"
        self.OriginState.pose = Pose(
            Point(*self.OriginPoint), Quaternion(*quaternion))
        self.OriginState.twist = Twist(Point(0, 0, 0), Point(0, 0, 0))

        # private
        self._show_log = False
        self._get_state = Odometry()
        self._set_state_pub = rospy.Publisher(
            self.OriginState.model_name + '/joint_setpoint', JointState, queue_size=10)
        rospy.Subscriber(self.OriginState.model_name +
                         '/state', Odometry, self._get_state_cb)

        self._get_goal = Odometry()
        self._goal_pub = rospy.Publisher(
            self.OriginState.model_name + '/goal', Odometry, queue_size=10)
        rospy.Subscriber(self.OriginState.model_name +
                         '/goal', Odometry, self._get_goal_cb)

        self._reset_map_pub = rospy.Publisher(
            self.OriginState.model_name + '/reset', Reset, queue_size=1)

        # gazebo
        rospy.wait_for_service('/gazebo/set_model_state')
        self._set_state_gazebo_srv = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)

    # public
    def get_state(self):
        '''
        return the sailboat_cuhksz state
        '''
        return self._get_state

    def pub_state(self, pred_rudder, pred_sail):
        self._set_state_pub.publish(self._action_msg(pred_rudder, pred_sail))
        if self._show_log:
            rospy.loginfo('State Publishing...done')

    def get_goal(self):
        '''
        return the sailboat_cuhksz goal
        '''
        return self._get_goal

    def pub_goal(self, pose, absolute=False):
        self._goal_pub.publish(self._set_goal(pose, absolute))
        if self._show_log:
            rospy.loginfo('Goal Publishing...done')

    def save_map(self, mapname="mymap"):
        if self._show_log:
            rospy.loginfo("Saving...")
        if subprocess.call("rosrun map_server map_saver --occ 1 --free 0 -f '" + mapname + "'", shell=True):
            rospy.logerr("Can not save!")
        if self._show_log:
            rospy.loginfo("Save Map done!")

    def reset(self):
        self._reset_map_pub.publish(Reset(True))
        self._set_state_gazebo_srv(self.OriginState)
        rospy.loginfo('Sent reset requests')

    def show_log(self, isShow=True):
        self._show_log = isShow

    # file control
    def createFile(self, file_name, mode='a'):
        print 'file opening...'
        return open(file_name, mode)

    def writeFile(self, file_handle, *args):
        print 'file writing...'
        return file_handle.write(','.join([str(arg) for arg in args])+'\n')

    def closeFile(self, file_handle):
        print '..file closed'
        return file_handle.close()

    # Private
    def _set_goal(self, pose, absolute=False):
        # TODO: maybe have bugs here
        if type(pose[0]) == int:
            pose = [(pose[0], pose[1], 0.0), (0.0, 0.0, 0.0, 1.0)]
        goal_pose = Odometry()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = 'world'
        if absolute is False:
            goal_pose.pose.pose.position = Point(
                pose[0][0]+self.OriginPoint[0], pose[0][1]+self.OriginPoint[1], 0.)
        if absolute is True:
            goal_pose.pose.pose.position = Point(pose[0][0], pose[0][1], 0.)
        return goal_pose

    def _get_state_cb(self, odometry):
        self._get_state = odometry
        return self._get_state

    def _get_goal_cb(self, odometry):
        self._get_goal = odometry
        return self._get_goal

    def _action_msg(self, pred_rudder, pred_sail):
        msg = JointState()
        msg.header = Header()
        msg.name = ['rudder_joint', 'sail_joint']
        msg.position = [pred_rudder[0], pred_sail[0]]
        msg.velocity = [pred_rudder[1], pred_sail[1]]
        msg.effort = [pred_rudder[2], pred_sail[2]]
        return msg


if __name__ == '__main__':
    arg.add_argument('-m', '--manual', type=bool,
                     default=False, help='Manual Control')
    arg.add_argument('-f', '--file', type=str,
                     help='A File included Kinematic Function')
    arg.add_argument('-n', '--namespace', type=str,
                     default='sailboat_cuhksz', help="Robot's Name")
    arg.add_argument('-o', '--origin', type=float,
                     default=(240, 95, 1.13), help='Set The Origin')
    arg.add_argument('-g', '--goal', type=tuple,
                     default=(35, 0, False), help='Set The Goal, (x, y, is absolute coordinates?)')
    arg.add_argument('-q', '--quaternion', type=float,
                     default=(0., 0., 0., 1.), help='Set The Quaternion of Robot when start')
    arg.add_argument('-r', '--reset', type=bool,
                     default=False, help='Reset The Simulation')
    args = arg.parse_args()
    sbc = SailboatController(args.namespace, args.origin, args.quaternion)
    rospy.init_node('cuhksz')
    rate = rospy.Rate(10)  # 10hz

    if args.reset:
        sbc.reset()
    else:
        file, goal = args.file, args.goal
        file_name, file_type = [(file[:i], file[i:])
                                for i in range(1, len(file)) if file[i] is '.'][0]
        if file_type != '.py':
            print('Error: Invalid value for \'file\': "' +
                  file_type + '" is not a python file')
            exit()
        exec "import " + file_name

        sbc.pub_goal(goal[:2], goal[2])
        sbc.reset()
        while not rospy.is_shutdown():
            try:
                sbc.pub_state(
                    *eval(file_name + '.model(sbc.get_state(), sbc.get_goal())'))
                rate.sleep()
            except rospy.ROSInterruptException:
                SailboatController.save_map()
                rospy.logerr(
                    "ROS Interrupt Exception! Just ignore the exception!")
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")
