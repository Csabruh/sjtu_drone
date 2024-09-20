import sys 
import os
import time 
import argparse
import math
import numpy as np
from enum import Enum
from math import pi
from unittest import case
import joblib
from collections import Counter

import rclpy
from rclpy import utilities
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, Vector3, Quaternion
from std_msgs.msg import Empty, Bool, Int8, String, Float32MultiArray
import tf_transformations
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from sjtu_drone_control.drone_utils.drone_object import DroneObject
from sjtu_drone_control.drone_utils.lidar_scan_feldolg import Scan_feldolg


v_linear: Vector3 = Vector3()
v_angular: Vector3 = Vector3()

#Target_position = (-1.67553, 41.0594, 0), Orientation(0, 0, 0.699463, 0.714669)


STATES = {
    0: "Landed",
    1: "Flying",
    2: "Taking off",
    3: "Landing",
}

MODES = ["velocity", "position"]

def most_frequent(List):
    occurence_count = Counter(List)
    return occurence_count.most_common(1)[0][0]

class DronePositionControl(DroneObject):
    def __init__(self):
        super().__init__('drone_position_control')
        #Flying = self.isFlying
        #state = self._state
        
    def yaw_to_pose(self, speed :float):
        if speed > 0.0:
            self.yaw((0.05 + (speed * 0.1)))
        elif speed < 0.0:
            self.yaw((-0.05 + (speed * 0.1)))

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
        self.create_timer(0.1, self.SM_timer_callback)
        self.StateSelector = 0
        self.Unit = DronePositionControl()
        self.ScanNode = Scan_feldolg()
        self.loaded_ml = joblib.load("install/sjtu_drone_control/share/sjtu_drone_control/sjtu_drone_control/random_forest_1.joblib")

        self._DroneState = STATES[0]
        self._isFlying = False
        self._DroneMode = MODES[0]
        self._ObstAngles = []
        self._ObstRanges = []
        self._gt_pose = Pose()
        self._obs = Bool()
        self.move_ack = False
        self.delay_ = 0

        self.q1 = [0.0, 0.0, 0.0, 0.0]
        self.rpy1 = [0.0, 0.0, 0.0]
        self.q_orientation = [0.0, 0.0, 0.0, 0.0]
        self.rpy2 = [0.0, 0.0, 0.0]
        self.TargetOrientation = [0.0, 0.0, 0.0]
        
        self.goal_position = [-1.67553, 41.0594, 1.0]
	feature_selection = [27, 28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44, 45,  46,
                    		47,  48,  49, 122, 123, 124, 173, 174, 175, 176, 179, 180, 181, 182, 183, 184, 185, 186, 189, 190, 
                    		191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 
                    		211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 232]

        self.sub_state = self.create_subscription(Int8, 'simple_drone/state', self.cb_state, 1024)
        self.sub_ObstEnc = self.create_subscription(Bool, 'simple_drone/ObstEnc', self.cb_obs, 1024)
        self.sub_cmd_mode = self.create_subscription(String, 'simple_drone/cmd_mode', self.cb_cmd_mode, 1024)
        self.sub_pose = self.create_subscription(Pose, 'simple_drone/gt_pose', self.cb_pose, 1024)
        self.sub_ObstRanges = self.create_subscription(Float32MultiArray, 'simple_drone/ObstArrayRang',self.cb_ObstRanges, 1024)

        
        self.Unit.velMode(False)
        self.Unit.posCtrl(False)
        
        # ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/sjtu_drone/sjtu_drone_description/src/mapper_params_online_async.yaml use_sim_time:=true


    def SM_timer_callback(self): 
        
        z1 = self._gt_pose.position.z

        match self.StateSelector:
            case 0:
                
                self.Unit.takeOff()
                self.Unit.velMode(True)
                #self.Unit.posCtrl(True)

                print("Drone state jelenleg: {}" .format(self._DroneState))
                #print(self.Unit.isPosctrl)
                #print(self.Unit.isVelMode)
                #print("{}" .format(self._isFlying))

                #if (format(self._DroneState) == 'Flying'): self.StateSelector = 1
                if (z1 >= 0.7 and z1 <= 1.5):
                    self.StateSelector = 1 
                    self.Unit.hover()
                else: 
                    print('Repülési magasság:', z1)
                    if z1 >= 1.5 : self.Unit.rise(-0.2)
                    if z1 <= 0.7 : self.Unit.rise(0.2)
                    
                
            case 1: #Switch to velocity mode
                
                print('Setting velocity control mode')

                if self._DroneMode == 'velocity': 
                    self.StateSelector = 2
                    
                else:
                    self.Unit.posCtrl(False)
                    self.Unit.velMode(True)
                    print('Velocity control mode set to True')
                    #self.get_logger().info('Velocity control mode set to True')


            case 2: #Get pose of and vektor to target. Rotate to target
                
                self.move_ack = False

                print('Get pose of and vektor to goal target')
                self.TargetOrientation = self.getVektor_toPose(self.goal_position[0], self.goal_position[1], self.goal_position[2])
                print('Rotate to target', self.TargetOrientation)
                RotationDone = self.rotate_to_pose(self.TargetOrientation)

                if RotationDone == True:
                    print('Rotation complete')
                    self.Unit.hover()
                    self.StateSelector = 3
                    
                

            case 3: #Move to target
                
                self.Unit.posCtrl(False)
                self.Unit.velMode(True)

                goal_x_min = self.goal_position[0] - 0.1
                goal_x_max = self.goal_position[0] + 0.1
                goal_y_min = self.goal_position[1] - 0.1
                goal_y_max = self.goal_position[1] + 0.1

                self_pos_x = self._gt_pose.position.x
                self_pos_y = self._gt_pose.position.y

                #print(self.goal_position)
                #print(self._gt_pose.position)  

                if self._DroneMode == 'velocity': 

                    print('Checking path to target')
                    #print('akadaly', self.ScanNode.obstacleInPath)
                    #print('mozgás megadva', self.move_ack)

                    if (goal_x_max > self_pos_x > goal_x_min) and (goal_y_max > self_pos_y > goal_y_min):
                        print('Movement completed')
                        self.StateSelector = 5

                    else: 
                        if self.ScanNode.obstacleInPath == True:
                            self.move_ack = False
                            print('Encountered obstacle')
                            self.Unit.hover()
                            self.StateSelector = 4

                        else: #elif self.move_ack == False:
                            print('Moving to target')

                            '''
                            x = self.goal_position[0]
                            y = self.goal_position[1]
                            z = self.goal_position[2]
                            '''
                            
                            
                            self.Unit.move_drone_along_vector(0.1, 0.0, 0.0)
                            #self.Unit.move_drone_to_pose(x, y, z)
                            self.move_ack = True
                                    

                
                else:
                        self.Unit.posCtrl(False)
                        self.Unit.velMode(True)
                        
                        print('Position control mode set to True')
                        #self.get_logger().info('Velocity control mode set to True')
                     
                                    
            case 4: #Evade obstacle

                self.Unit.posCtrl(False)
                self.Unit.velMode(True)        

                if self._DroneMode == 'velocity': 
                    avoid_done = self.avoid_obstacle()

                    if avoid_done == True:

                        self.StateSelector = 2

                else:

                    self.Unit.posCtrl(False)
                    self.Unit.velMode(True)
                    

            case 5: #Target reached. Landing drone
                self.get_logger().info('Target position reached. Landing drone')
                self.Unit.land()
                exit()


    def getVektor_toPose(self, x, y, z):
        #get initial data
        self.drone_pose = self._gt_pose
        x1 = self._gt_pose.position.x
        y1 = self._gt_pose.position.y
        z1 = self._gt_pose.position.z
	Target_vektor = [(x-x1), (y-y1), (z-z1)]
        
	#így áll jelenleg a drón. Betölti ezt a class-ba.
        self.q1[0] = self._gt_pose.orientation.w
        self.q1[1] = self._gt_pose.orientation.x
        self.q1[2] = self._gt_pose.orientation.y
        self.q1[3] = self._gt_pose.orientation.z
        self.rpy1 = euler_from_quaternion(self.q1)

        #Célvektor bemérése és differencia 
        
        #vektor_magn = math.sqrt(math.pow(Target_vektor[0],2) + math.pow(Target_vektor[1],2) + math.pow(Target_vektor[2],2))
	#Csax X-Y síkban keressük a szögeltérést, hogy az elfordulandó szöget megkapjuk. Nincs vertikális mozgás.

	vektor_magn = math.sqrt(math.pow(Target_vektor[0],2) + math.pow(Target_vektor[1],2))
	mag1 = math.sqrt(math.pow(Target_vektor[0], 2) + math.pow(Target_vektor[1], 2))
	mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
	beta = math.acos(skew_x / (mag1 * mag2)) 
	
	if Target_vektor[1] < 0:
	    if Target_vektor[0] < 0:
	        TargetOrientation[0] = -beta 
	    elif Target_vektor[0] > 0:
	        TargetOrientation[0] = -beta
	    elif Target_vektor[0] == 0:
	        TargetOrientation[0] = - 1.57079
	elif Target_vektor[1] > 0:
	    if Target_vektor[0] < 0:
	        TargetOrientation[0] = beta
	    elif Target_vektor[0] < 0:
	        TargetOrientation[0] = beta
	    elif Target_vektor[0] == 0:
	        TargetOrientation[0] = 1.57079  
	else: 
	    if skew_x < 0:
	        TargetOrientation[0] = 3.14159
	    elif skew_x >= 0:
	        TargetOrientation[0] = 0

	TargetOrientation[1] = 0
	TargetOrientation[2] = 0

        self.q_orientation = quaternion_from_euler(self.TargetOrientation[0], self.TargetOrientation[1], self.TargetOrientation[2])
        return self.TargetOrientation
    


    def rotate_to_pose(self, TargetOrientation :list[float]):

        print('Jelenlegi orientáció', self.rpy1)
        
        if self.rpy1[0] < (self.TargetOrientation[0] - 0.01) or self.rpy1[0] > (self.TargetOrientation[0] + 0.01):
  
            if self.TargetOrientation[0] >= 0.0 and self.rpy1[0] >= 0.0:
                differencia = (-0.8)*((self.TargetOrientation[0] - self.rpy1[0]) / 3.14)
            elif self.TargetOrientation[0] < 0.0 and self.rpy1[0] < 0.0:
                differencia = 0.8*((self.TargetOrientation[0] + self.rpy1[0]) / 3.14)
            elif self.TargetOrientation[0] >= 0.0 and self.rpy1[0] < 0.0:
                if (-3.14+abs(self.TargetOrientation[0])) >= self.rpy1[0]:
                    differencia = 0.5
                elif (-3.14+abs(self.TargetOrientation[0])) < self.rpy1[0]:
                    differencia = -0.5
            elif self.TargetOrientation[0] < 0.0 and self.rpy1[0] >= 0.0:    
                if (3.14-abs(self.TargetOrientation[0])) >= self.rpy1[0]:
                    differencia = -0.5
                elif (3.14-abs(self.TargetOrientation[0])) < self.rpy1[0]:
                    differencia = 0.5
            self.Unit.yaw_to_pose(differencia)
        
        elif self.rpy1[0] > (self.TargetOrientation[0] - 0.01) and self.rpy1[0] < (self.TargetOrientation[0] + 0.01):
            return True


    def avoid_obstacle(self):

        dataset = [0.0] * 80
        index = 0
        selector = 0

        for x in self._ObstRanges:
            
            if index == feature_selection[selector]:

                dataset[selector] = x

                if selector < 79 : selector += 1
                else: selector = 0

            if index < 359 : index += 1 
            else: index = 0

        prediction = most_frequent(self.loaded_ml.clf.predict(dataset))


        if prediction == 0:
            self.Unit.yaw(0.3)
        elif prediction == 1:
            self.Unit.yaw(-0.3)
        elif prediction == 2:
            self.Unit.hover()
        elif prediction == 3:
            self.Unit.move_drone_along_vector(0.1, 0.0, 0.0)

        if self.ScanNode.obstacleInPath == False:

            self.Unit.hover()
            return True





    def cb_state(self, msg: Int8):
        """Callback for the drone state"""
        self._DroneState = STATES[msg.data]

    def cb_cmd_mode(self, msg: String):
        """Callback for the command mode"""
        if msg.data in MODES:
            self._DroneMode = msg.data

    def cb_ObstRanges(self, msg: Float32MultiArray):
        """Callback for the drone state"""
        self._ObstRanges = msg.data           

    def cb_pose(self, msg: Pose) -> None:
        """Callback for the ground truth pose"""
        self._gt_pose = msg

    def cb_obs(self, msg: Bool) -> None:
        """Callback for the ground truth pose"""
        self._obs = msg

def main(args=None):
    rclpy.init(args=args)

    Scandata = Scan_feldolg()
    Drone = DronePositionControl()
    Drone.logger.set_level(LoggingSeverity.DEBUG)
    StateM = StateMachine()
    
    executor = MultiThreadedExecutor()
    executor.add_node(Drone)
    executor.add_node(Scandata)
    executor.add_node(StateM)

    try:
        executor.spin()
    except (KeyboardInterrupt, ROSInterruptException) as e:
        Drone.get_logger().error(f'Error: {e}')
    finally:
        sys.exit()


if __name__ == '__main__':
    main()
