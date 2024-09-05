import numpy as np
import math


'''
Joint nomeclature:
    joint1: Coxa
    joint2: Thigh
    joint3: Tibia
'''

L1 = 0.09 # Length between joint1 (Near the base joint) and joint2
L2 = 0.28 # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.34 # Length between Joint3 and Tip

D1 = 0.240 # Distance between Origin of base and origin of the joint1

'''
limp_pos contains the angle of the orientation of the limb w.r.t. base frame
'''
limb_angle = (45, 135, -135, -45)

# Robot parameters
offset_x = 0.2
offset_y = 0.2
coxa_length = L1 
femur_length = L2 
tibia_length = L3

coxa_upperlimit = math.pi/2
coxa_lowerlimit = -math.pi/2

femur_upperlimit = 1*math.pi/18
femur_lowerlimit = -5*math.pi/6

tibia_upperlimit = 2*math.pi/3
tibia_lowerlimit = -8*math.pi/9


# Motion related
sec = 1.0
span = 0.13
foward = -0.07
height = 0.24

std_movement_time = 1 #sec
movement_update_rate = 20 #Hz