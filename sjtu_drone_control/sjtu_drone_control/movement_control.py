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

from geometry_msgs.msg import Twist, Pose, Vector3
from std_msgs.msg import Empty, Bool, Int8, String
from sensor_msgs.msg import LaserScan

from sjtu_drone_control.drone_utils.drone_object import DroneObject
from sjtu_drone_control.lidar_scan_feldolg import Scan_feldolg

v_linear: Vector3 = Vector3()
v_angular: Vector3 = Vector3()

STATES = {
    0: "Landed",
    1: "Flying",
    2: "Taking off",
    3: "Landing",
}

MODES = ["velocity", "position"]

class DronePositionControl(DroneObject):
    def __init__(self):
        super().__init__('drone_position_control')
        #Flying = self.isFlying
        #state = self._state
        
    

    def move_drone_to_pose(self, x, y, z):
        # Override the move_drone_to_pose method if specific behavior is needed
        super().moveTo(x, y, z)
        self.get_logger().info(f'Moving drone to pose: x={x}, y={y}, z={z}')   
     
    def move_drone_along_vector(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, a: float = 0.0, b: float = 0.0, c: float = 0.0):
	# Override the move_drone_to_pose method if specific behavior is needed
        v_linear.x = x
        v_linear.y = y
        v_linear.z = z
        v_angular.x = a
        v_angular.y = b
        v_angular.z = c
        super().move(v_linear, v_angular)
        self.get_logger().info(f'Moving drone along vector: x={x}, y={y}, z={z}') 


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.create_timer(1.0, self.timer_callback)
        self.StateSelector = 0
        self.Unit = DronePositionControl()
        self.ScanNode = Scan_feldolg()
        self._DroneState = STATES[0]
        self._isFlying = False
        self._DroneMode = MODES[0]
        

        self.sub_state = self.create_subscription(Int8, 'simple_drone/state', self.cb_state, 1024)
        self.sub_cmd_mode = self.create_subscription(String, 'simple_drone/cmd_mode', self.cb_cmd_mode, 1024)



    def timer_callback(self):
        

        match self.StateSelector:
            case 0:
                print("{}" .format(self._DroneState))
                print("{}" .format(self._isFlying))
                if format(self._DroneState) == 'Flying' : self.StateSelector = 1
                else: self.Unit.takeOff()
                
                
                
            case 1:
                if self._DroneMode == 'velocity' : 
                    self.ScanNode.ScanResult()
                    
                    self.StateSelector = 2
                    
                else:
                    self.Unit.velMode(True)
                    self.get_logger().info('Velocity control mode set to True')


            case 2:
                self.Unit.move_drone_along_vector(0.1, 0.2, 0.5, 0.0, 0.0, 0.0)

                self.StateSelector = 3

            case 3:
                self.Unit.move_drone_along_vector(0.3, 0.7, 0.1, 0.0, 0.0, 0.0)
                
                self.StateSelector = 4
            
            case 4:
                self.Unit.hover()


    def cb_state(self, msg: Int8):
        """Callback for the drone state"""
        self._DroneState = STATES[msg.data]

    def cb_cmd_mode(self, msg: String):
        """Callback for the command mode"""
        if msg.data in MODES:
            self._DroneMode = msg.data


def main(args=None):
    rclpy.init(args=args)

    Scandata = Scan_feldolg()
    Drone = DronePositionControl()
    Drone.logger.set_level(LoggingSeverity.DEBUG)
    StateM = StateMachine()
    
    executor = MultiThreadedExecutor()
    executor.add_node(Drone)
    executor.add_node(StateM)
    executor.add_node(Scandata)

    try:
        executor.spin()
    except (KeyboardInterrupt, ROSInterruptException) as e:
        Drone.get_logger().error(f'Error: {e}')
    finally:
        sys.exit()


if __name__ == '__main__':
    main()
