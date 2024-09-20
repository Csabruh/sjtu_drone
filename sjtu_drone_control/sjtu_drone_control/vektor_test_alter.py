import math
from math import pi
#import tf_transformations
#from tf_transformations import euler_from_quaternion, quaternion_from_euler


x = 0
y = 0
z = 0


x1 = 10.681232
y1 = -45.123123
z1 = 0

'''
print("Input x value")
x1 = float(input())
print("Input y value")
y1 = float(input())
print("Input z value")
z1 = float(input())
'''

print("Values given:", (x1,y1,z1))
print("Starting positions:", (x,y,z))

Target_pos = [x1, y1, z1]
Odom_pos = [x, y, z]
Target_vektor = [(x1-x), (y1-y), (z1-z)]
rpy1 = [0.0, 0.0, 0.0]
Target_orientation = [0, 0, 0]


vektor_magn = math.sqrt(math.pow(Target_vektor[0],2) + math.pow(Target_vektor[1],2) + math.pow(Target_vektor[2],2))
print("Target vektor:", Target_vektor)
print("vektor magnitude:",vektor_magn)

skew_x = Target_pos[0] - Odom_pos[0]
skew_y = Target_pos[1] - Odom_pos[1]
#dot = skew_x * 1 + skew_y * 0

mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
beta = math.acos(skew_x / (mag1 * mag2))    #skew_x helyett dot volt
print("Absolute calculation in rad:", beta)

if skew_y < 0:
    if skew_x < 0:
        Target_orientation[0] = -beta 
    elif skew_x > 0:
        Target_orientation[0] = -beta
    elif skew_x == 0:
        Target_orientation[0] = - 1.57079
elif skew_y > 0:
    if skew_x < 0:
        Target_orientation[0] = beta
    elif skew_x < 0:
        Target_orientation[0] = beta
    elif skew_x == 0:
        Target_orientation[0] = 1.57079  
else: 
    if skew_x < 0:
        Target_orientation[0] = 3.14159
    elif skew_x >= 0:
        Target_orientation[0] = 0
    
#Target_orientation[0] = gamma
Target_orientation[1] = 0
Target_orientation[2] = 0

print("Target orientation",Target_orientation)

#z körüli tengely forgása
szög_diff = Target_orientation[2] - rpy1[2] 

#rpy1 = quaternion_from_euler(Target_orientation[0], Target_orientation[1], Target_orientation[2])
#rpy2 = quaternion_from_euler(0,0,0)
#print(rpy1, "\n", rpy2)