import math
from math import pi
#import tf_transformations
#from tf_transformations import euler_from_quaternion, quaternion_from_euler

x = 0
x1 = -10.681232
y = 0
y1 = 45.123123
z = 0
z1 = 0

Target_vektor = [(x1-x), (y1-y), (z1-z)]
pointer_vektor = [1.0, 0.0, 0.0]
rpy1 = [0.0, 0.0, 0.0]
print(Target_vektor)

vektor_magn = math.sqrt(math.pow(Target_vektor[0],2) + math.pow(Target_vektor[1],2) + math.pow(Target_vektor[2],2))
print(vektor_magn)
Elemi_vektor = [0.0, 0.0, 0.0]
Elemi_vektor[0] = Target_vektor[0] / vektor_magn    #x
Elemi_vektor[1] = Target_vektor[1] / vektor_magn    #y
Elemi_vektor[2] = Target_vektor[2] / vektor_magn    #z

Target_orientation = [0, 0, 0]
#Target_orientation[0] = 3.14 * Elemi_vektor[0] / Elemi_vektor[1]
Target_orientation[1] = 0
Target_orientation[2] = 0

gamma = math.atan(Elemi_vektor[1]/Elemi_vektor[0])
print(gamma)

if Elemi_vektor[0] > 0:
    if Elemi_vektor[1] > 0:
        Target_orientation[0] = gamma
    elif Elemi_vektor[1] < 0:    
        Target_orientation[0] = gamma
    elif Elemi_vektor[1] == 0: 
        Target_orientation[0] = 0
elif Elemi_vektor[0] < 0:
    if Elemi_vektor[1] > 0:
        Target_orientation[0] = gamma
    elif Elemi_vektor[1] < 0:    
        Target_orientation[0] = 0 - gamma
    elif Elemi_vektor[1] == 0: 
        Target_orientation[0] = 3.14159
elif Elemi_vektor[0] == 0:
    if Elemi_vektor[1] > 0:
        Target_orientation[0] = 1.57079 
    elif Elemi_vektor[1] < 0:
        Target_orientation[0] = -1.57079 
    elif Elemi_vektor[1] == 0:
        Target_orientation[0] = 0

szög_diff = Target_orientation[2] - rpy1[2]
print("Target orientation[rad]:",Target_orientation)

#rpy1 = quaternion_from_euler(Target_orientation[0], Target_orientation[1], Target_orientation[2])
#rpy2 = quaternion_from_euler(0,0,0)
#print(rpy1, "\n", rpy2)
