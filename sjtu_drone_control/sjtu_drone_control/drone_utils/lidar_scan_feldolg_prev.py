import sys 
import os
import time 
import argparse
from enum import Enum
from math import pi
from unittest import case
import math 

import rclpy
from rclpy import utilities
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

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
        self.obstacleRanges = [0] * 181
        self.x = 0

        self.radToDeg = 180.00/3.14
        self.angle_increment = 0.017493
        self.degVal = 0.0
        self.deg = 0

        self.logger = self.get_logger()
        self.my_callback_group = ReentrantCallbackGroup()


        #self.create_timer(0.1, self.timer_callback)
        #self.pubObstAngles = self.create_publisher(Float32MultiArray, 'simple_drone/ObstArrayAngl', 1024)

        self.pubObstRanges = self.create_publisher(Float32MultiArray, 'simple_drone/ObstArrayRang', 1024)




    def ScanResult(self):
        
        #if x_ori > 0.0:
        #a = self.radToDeg*math.atan(y_ori / x_ori)
        #r_min = a
        #r_max = a + 90
        #l_min = 
        #l_max = 
        
        self.obstacleInPath = False 
        self.deg = 0

        for x, value in np.ndenumerate(self.ScanData):
            #self.deg = int(str(x).translate({ord(i): None for i in '(),'})) folyamatosan pörög felfele

            if (self.deg >= 270 and self.deg < 360):
                
                #self.degVal = index_left*self.angle_increment*self.radToDeg
                #Redundáns, mert a végeredmény 1° körül lesz
                index_left = self.deg - 270
                self.obstacleRanges[index_left] = value


                if value > 0.4 and value < 0.8:
                    self.obstacleInPath = True
                else:
                    self.obstacleInPath = False 

            elif (self.deg >= 0 and self.deg <= 90):
                
                index_right = self.deg + 90
                self.obstacleRanges[index_right] = value
                

                if value > 0.4 and value < 0.8:
                    self.obstacleInPath = True
                else:
                    self.obstacleInPath = False 


            if self.deg < 360 : self.deg += 1 
            else: self.deg = 0

        ArrayMsgRang = Float32MultiArray()
        ArrayMsgRang.data = self.obstacleRanges

        
        self.pubObstRanges.publish(ArrayMsgRang)
        b = len(self.obstacleRanges)
        print(b, "\n")

        if self.obstacleInPath == True:
            #self.logger.info("Obstacle angle array: {}".format(self.obstacleAngles))
            #self.logger.info("Obstacle range array: {}".format(self.obstacleRanges))
            #print("Obstacle encountered")

            self.logger.info("Obstacle encountered")
            #print(ArrayMsgRang, "\n", "\n")
            return True
        else:
            self.logger.info("No obstacle encountered")
            #print("No obstacle encountered")
            return False


    def cb_scan(self, msg: LaserScan):
        """Callback for the command mode"""
        self.ScanData = msg.ranges
        self.ScanResult()

    #def timer_callback(self):
        #self.ScanResult()

    #def scan_callback(self):
        #self.ScanResult()


def main(args=None):
    rclpy.init(args=args)
    lidar_scan_feldolg_node = Scan_feldolg()
    rclpy.spin(lidar_scan_feldolg_node)
    lidar_scan_feldolg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
            
