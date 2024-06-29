import sys 
from math import pi
import termios
import tty
import numpy as np
import os

import rclpy
from rclpy import utilities
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String, Bool
from sensor_msgs.msg import LaserScan 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup




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

        self.ScanData = []
        self.obstacleInPath = False
        self.obstacleRanges = [0] * 361
        self.x = 0

        self.radToDeg = 180.00/3.14
        self.angle_increment = 0.017493
        self.degVal = 0.0
        self.deg = 0

        

        self.logger = self.get_logger()

        self.sub_state = self.create_subscription(LaserScan, 'simple_drone/scan', self.cb_scan, 1024)
        self.pubObstRanges = self.create_publisher(Float32MultiArray, 'simple_drone/ObstArrayRang', 1024)
        self.pubObstEnc = self.create_publisher(Bool, 'simple_drone/ObstEnc', 1024)
        

    def ScanResult(self):
        
        self.deg = 0
        self.obstacleInPath = False
        BoolMsg = Bool()
        BoolMsg.data = True

        for x, value in np.ndenumerate(self.ScanData):
            #self.deg = int(str(x).translate({ord(i): None for i in '(),'})) folyamatosan pörög felfele


            if self.deg < 360: 
                index_left = self.deg
                self.obstacleRanges[index_left] = (value)

                #if (self.deg >= 320 and self.deg < 360) or (self.deg >= 0 and self.deg <= 40):
                    #if (value > 0.4 and value < 0.8):

                self.deg += 1 
            else: self.deg = 0

        for data in self.obstacleRanges[:40]:
            if data < 0.7 and data > 0.3:
                self.obstacleInPath = True

        for data in self.obstacleRanges[320:]:
            if data < 0.7 and data > 0.3:
                self.obstacleInPath = True

        ArrayMsgRang = Float32MultiArray()
        ArrayMsgRang.data = self.obstacleRanges

        
        self.pubObstRanges.publish(ArrayMsgRang)
        

        if self.obstacleInPath == True:
            #self.logger.info("Obstacle angle array: {}".format(self.obstacleAngles))
            #self.logger.info("Obstacle range array: {}".format(self.obstacleRanges))
            #print("Obstacle encountered")

            self.pubObstEnc.publish(BoolMsg)
            self.logger.info("Obstacle encountered")

            #print(ArrayMsgRang, "\n", "\n")
        #else:
            #self.logger.info("No obstacle encountered")
            #print("No obstacle encountered")


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
            
