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

from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import LaserScan 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

sys.path.append(os.path.abspath("/home/dev/drone_stuff/src/sjtu_drone/sjtu_drone_control/sjtu_drone_control/ml_dataset"))



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
        self.mldataset = [0] * 361
        self.x = 0

        self.radToDeg = 180.00/3.14
        self.angle_increment = 0.017493
        self.degVal = 0.0
        self.deg = 0
        self.input = "s"
        

        self.logger = self.get_logger()
        self.my_callback_group = ReentrantCallbackGroup()

        self.sub_state = self.create_subscription(LaserScan, 'simple_drone/scan', self.cb_scan, 1024)
        self.sub_keyboard_input= self.create_subscription(String, 'simple_drone/kb_input', self.cb_kb_input, 1024)

        self.pubObstRanges = self.create_publisher(Float32MultiArray, 'simple_drone/ObstArrayRang', 1024)
        

    def ScanResult(self):
        
        self.file = open('src/sjtu_drone/sjtu_drone_control/sjtu_drone_control/ml_dataset/DataSet_1.txt', 'a')
        
        self.obstacleInPath = False 
        self.deg = 0
        

        for x, value in np.ndenumerate(self.ScanData):
            #self.deg = int(str(x).translate({ord(i): None for i in '(),'})) folyamatosan pörög felfele

            index_left = self.deg
            self.obstacleRanges[index_left] = (value)
            self.mldataset[index_left] = value


            if value > 0.4 and value < 0.8:
                self.obstacleInPath = True
            else:
                self.obstacleInPath = False 
            

            if self.deg < 360 : self.deg += 1 
            else: self.deg = 0

        self.mldataset[360] = self.input
        ArrayMsgRang = Float32MultiArray()
        ArrayMsgRang.data = self.obstacleRanges

        
        self.pubObstRanges.publish(ArrayMsgRang)
        
        
        self.file.write(self.mldataset + self.input + '\n')
        #self.file.write('\n')
        self.file.close()

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

    def cb_kb_input(self, msg: String):
        key = msg.data
        if key.lower() == 'w' or key.lower() == 's' or key.lower() == 'x' or key.lower() == 'a' or key.lower() == 'd':
            self.input = key.lower()    



def main(args=None):
    rclpy.init(args=args)
    lidar_scan_feldolg_node = Scan_feldolg()
    rclpy.spin(lidar_scan_feldolg_node)
    lidar_scan_feldolg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
            
