import sys 
import os
import time 
import argparse
from enum import Enum
from math import pi
from unittest import case

import rclpy
from rclpy import utilities
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from sjtu_drone_control.drone_utils.drone_object import DroneObject
from geometry_msgs.msg import Twist, Pose, Vector3
from std_msgs.msg import Empty, Bool, Int8, String
from sensor_msgs.msg import LaserScan

import numpy as np


#angle_min: -3.140000104904175
#angle_max: 3.140000104904175
#angle_increment: 0.017493
#time_increment: 0.0
#scan_time: 0.0
#range_min: 0.30000001192092896
#range_max: 12.0




class Scan_feldolg(Node):
    def __init__(self):
        super().__init__('scan_feldolg')
        self.sub_state = self.create_subscription(LaserScan, 'simple_drone/scan', self.cb_scan, 1024)
        self.ScanData = []
        self.obstacleInPath = False
        self.obstacleAngles = []
        self.obstacleRanges = []
        self.x = 0

        self.radToDeg = 180.00/3.14
        self.angle_increment = 0.017493
        self.degVal = 0.0

        self.logger = self.get_logger()

        self.create_timer(1.0, self.timer_callback)


    def ScanResult(self):
        for x, value in np.ndenumerate(self.ScanData):
            index = int(str(x).translate({ord(i): None for i in '(),'}))
            
            if value > 0.4 and value < 0.8:
                self.obstacleInPath = True
                self.degVal = index*self.angle_increment*self.radToDeg

                self.obstacleAngles.insert(index,self.degVal)
                self.obstacleRanges.insert(index,value)
            else:
                self.obstacleAngles.insert(index,0)
                self.obstacleRanges.insert(index,0)

        if self.obstacleInPath == True:
            #self.logger.info("Obstacle angle array: {}".format(self.obstacleAngles))
            #self.logger.info("Obstacle range array: {}".format(self.obstacleRanges))
            #print(self.obstacleAngles, "\n", self.obstacleRanges, "\n")
            return True
        else:
            return False


    def cb_scan(self, msg: LaserScan):
        """Callback for the command mode"""
        self.ScanData = msg.ranges

    def timer_callback(self):
        self.ScanResult()



def main(args=None):
    rclpy.init(args=args)
    lidar_scan_feldolg_node = Scan_feldolg()
    rclpy.spin(lidar_scan_feldolg_node)
    lidar_scan_feldolg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
            
