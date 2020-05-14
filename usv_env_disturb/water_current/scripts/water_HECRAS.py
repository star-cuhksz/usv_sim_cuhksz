#!/usr/bin/env python
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Int64

from water_current.srv import GetSpeed, GetSpeedResponse
from water_current.msg import Reset

import os
import sys
import h5py
import numpy as np
import yaml
import math
import rospy
import copy


class WATER_HECRAS():
    def __init__(self):
        # public
        self.map = OccupancyGrid()
        self.map_pub = rospy.Publisher(
            'map', OccupancyGrid, queue_size=1)

        rospy.Subscriber('sailboat_cuhksz/reset', Reset, self.__Reset_cb)
        self.datasetX = []
        self.datasetY = []
        self.datasetCoord = []

        # yaml params
        self.filename = 0
        self.refX = 0
        self.refY = 0
        self.nodeCoordinate = 0
        self.nodeNameX = 0
        self.nodeNameY = 0
        self.resolution = 0
        self.originX = 0
        self.originY = 0
        self.width = 0
        self.height = 0
        self.startTime = 0
        self.mapTorviz = 0
        self.showVehicleOnMap = 0

        # private
        self.__indexMap = 0
        self.__map_data = []
        self.__path_color = 51

    # public
    def ParseConfigFile(self, config_file_name):
        with open(config_file_name, 'r') as file_stream:
            data_loaded = yaml.load(file_stream)
        print '[WATER_HECRAS]: Loading yaml file: ' + config_file_name

        self.filename = data_loaded['filename']
        self.refX, self.refY = data_loaded['ref']
        self.nodeCoordinate = data_loaded['nodeCoordinate']
        self.nodeNameX = data_loaded['nodeNameX']
        self.nodeNameY = data_loaded['nodeNameY']
        self.resolution = data_loaded['resolution']
        self.originX, self.originY = data_loaded['origin'][:2]
        self.width = data_loaded['width']
        self.height = data_loaded['height']
        self.startTime = data_loaded['startTime']
        self.showVehicleOnMap = data_loaded['showVehicleOnMap']
        self.mapTorviz = data_loaded['mapTorviz']

    def PrintHierarchy(self, gid, level, text):
        for hidStr in h5py.h5g.GroupIter(gid):
            lid = gid.links.get_info(hidStr)
            try:
                hid = h5py.h5o.open(gid, hidStr)
            except KeyError as e:
                hid, t = None, None

            t = type(hid)
            if t == h5py.h5g.GroupID:
                self.PrintHierarchy(hid, level+1, text+" > "+hidStr)
            elif t == h5py.h5d.DatasetID:
                if hidStr == self.nodeNameX:
                    self.datasetX = h5py.Dataset(hid)
                elif hidStr == self.nodeNameY:
                    self.datasetY = h5py.Dataset(hid)
                elif hidStr == self.nodeCoordinate:
                    self.datasetCoord = h5py.Dataset(hid)
            elif t == np.ndarray:
                if hidStr == self.nodeNameX:
                    self.datasetX = hid
                elif hidStr == self.nodeNameY:
                    self.datasetY = hid
                elif hidStr == self.nodeCoordinate:
                    self.datasetCoord = hid

    def StartRosService(self):
        rospy.Subscriber("waterCurrentTime", Int64, self.__DefineTime_cb)
        self.__PreProcessDataset()
        print "[WATER_HECRAS]: Ready to answer water current."

    def LoadMap(self):
        _w, _h = self.width, self.height
        self.map.header.frame_id = "odom"
        self.map.info.resolution = self.resolution
        self.map.info.width = _w
        self.map.info.height = _h
        self.map.info.origin.position.x = self.originX
        self.map.info.origin.position.y = self.originY
        self.map.info.origin.position.z = 0
        self.map.info.origin.orientation.x = 0
        self.map.info.origin.orientation.y = 0
        self.map.info.origin.orientation.z = 0
        self.map.info.origin.orientation.w = 1

        # DEBUG:
        print "[WATER_HECRAS]: Preparing map to rviz. Size (%d, %d) origin(%d, %d)" % (
            _w, _h, 0, 0)

        self.map.data = [0] * _w * _h
        for y in range(0, _h):
            sys.stdout.write('\r%d rows processed' % y)
            sys.stdout.flush()
            _yw = y * _w
            for x in range(0, _w):
                # 1.2:delta, 105.833:127/1.2
                value = 105.833 * \
                    self.datasetX[self.startTime][self.__indexMap[x][y]]
                # limited value 0 to 127
                value = self.__iff(value, (-1)*value, 127)
                self.map.data[_yw + x] = int(value)

        self._map_data = copy.deepcopy(self.map.data)
        rospy.Service('waterCurrent', GetSpeed, self.__WaterCurrent_hd)
        # DEBUG:
        print "[WATER_HECRAS]: Map loaded!"

    # private
    def __DefineTime_cb(self, data):
        self.startTime = data.data
        print "[WATER_HECRAS]: New time defined: ", self.startTime

    def __PreProcessDataset(self):
        self.__indexMap = np.zeros((self.width, self.height), dtype=np.int)

        dataSetLen = len(self.datasetCoord)
        print "[WATER_HECRAS]: Preprocessing the dataset indexes(length: ", dataSetLen, "). This can take some minutes..."

        amount = dataSetLen / 100
        estimatedRefX, estimatedRefY = self.datasetCoord[0]
        for i in range(0, dataSetLen):
            sys.stdout.write('\r%d %% processed' % (i / amount))
            _x, _y = self.datasetCoord[i]
            estimatedRefX = _x if estimatedRefX > _x else estimatedRefX
            estimatedRefY = _y if estimatedRefY > _y else estimatedRefY

            x, y = int(math.floor(_x - self.refX)
                       ), int(math.floor(_y - self.refY))
            x = self.__iff(x, 0, self.width)
            y = self.__iff(y, 0, self.height)
            self.__indexMap[x][y] = i

        print "[WATER_HECRAS]: estimatedRefX: ", estimatedRefX
        print "[WATER_HECRAS]: estimatedRefY: ", estimatedRefY

    def __WaterCurrent_hd(self, req):
        i = self.__indexMap[req.x][req.y]
        if self.showVehicleOnMap:
            self.map.data[req.y * self.width + req.x] = 127
        return GetSpeedResponse(self.datasetX[self.startTime][i], self.datasetY[self.startTime][i])

    def __Reset_cb(self, reset):
        if reset.isReset is True:
            self.map.data = copy.deepcopy(self._map_data)
            print "[WATER_HECRAS]: Map reload!"
        print "[WATER_HECRAS]: Subscriber Received!"

    def __iff(self, a, b, c):
        if a < b:
            a = b
        if a > c:
            a = c
        return a


if __name__ == '__main__':
    rospy.init_node('water_current')

    water_hecras = WATER_HECRAS()
    water_hecras.ParseConfigFile(sys.argv[1])
    filename = water_hecras.filename
    try:
        fid = h5py.h5f.open(filename, flags=h5py.h5f.ACC_RDONLY)
    except IOError as e:
        sys.stderr.write(
            '[WATER_HECRAS]: Unable to open File: ' + filename + '\n')
        exit()
    except:
        sys.stderr.write(
            "[WATER_HECRAS]: Unexpected error: ", sys.exc_info()[0])
        raise

    fid_type = type(fid)
    if fid_type == h5py.h5f.FileID:
        txt = os.path.basename(fid.name)
    elif fid_type == h5py.h5g.GroupID:
        txt = fid.name
    else:
        txt = 'root'
    print "[WATER_HECRAS]: Root: " + txt

    # DEBUG:
    water_hecras.PrintHierarchy(fid, 0, ">")
    print "[WATER_HECRAS]: DatasetX: ", len(
        water_hecras.datasetX), ", ", len(water_hecras.datasetX[0])
    print "[WATER_HECRAS]: DatasetY: ", len(
        water_hecras.datasetY), ", ", len(water_hecras.datasetY[0])
    print "[WATER_HECRAS]: DatasetCoord: ", len(
        water_hecras.datasetCoord), ", ", len(water_hecras.datasetCoord[0])
    print "[WATER_HECRAS]: Time interval: 0 - ", (len(water_hecras.datasetX)-1)

    water_hecras.StartRosService()
    if water_hecras.mapTorviz is 1:
        water_hecras.LoadMap()

    rate = rospy.Rate(1)  # 10hz
    while not rospy.is_shutdown():
        try:
            if water_hecras.mapTorviz is 1:
                print "[WATER_HECRAS]: Sending map! Time: %d" % water_hecras.startTime
                water_hecras.map_pub.publish(water_hecras.map)
                print "[WATER_HECRAS]: Map sent!"

            rate.sleep()
            print "[WATER_HECRAS]: End sleeping"
        except rospy.ROSInterruptException:
            rospy.logerr(
                "[WATER_HECRAS]: ROS InterruptException! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr(
                "[WATER_HECRAS]: ROS Time Backwards! Just ignore the exception!")
