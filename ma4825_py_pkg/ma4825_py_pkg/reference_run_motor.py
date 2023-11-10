"""
Reference of the main script before integrated with ROS2
"""


from Ax12 import Ax12
from utils_robot import get_angle_from_position


def deg_to_bits(theta):
    return int((theta+150.0)/0.29297)

def move_motor_from_position(X: float,Y: float,Z: float,THETA_DEG: float):
    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = get_angle_from_position(X, Y, Z, THETA_DEG)
    print(deg_to_bits(theta_1))
    print(deg_to_bits(theta_2))
    print(deg_to_bits(theta_3))
    print(deg_to_bits(theta_4))
    print(deg_to_bits(theta_5))
    print(deg_to_bits(theta_6))
    J1.set_goal_position(deg_to_bits(theta_1))
    J2.set_goal_position(deg_to_bits(theta_2))
    J3.set_goal_position(deg_to_bits(theta_3))
    J4.set_goal_position(deg_to_bits(theta_4))
    J5.set_goal_position(deg_to_bits(theta_5))
    J6.set_goal_position(deg_to_bits(theta_6))

#ID Dictionary
ID_Dict = {
    'Joint_1': 6,
    'Joint_2': 3,
    'Joint_3': 1,
    'Joint_4': 4,
    'Joint_5': 5,
    'Joint_6': 2,
    'Joint_7': 7,
}

# e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
Ax12.DEVICENAME = '/dev/ttyUSB0'

# sets baudrate and opens com port
Ax12.BAUDRATE = 1_000_000
Ax12.connect()

# create AX12 instances for every joint
J1, J2, J3, J4, J5, J6, J7 = Ax12(ID_Dict['Joint_1']), Ax12(ID_Dict['Joint_2']), Ax12(ID_Dict['Joint_3']), Ax12(ID_Dict['Joint_4']), Ax12(ID_Dict['Joint_5']), Ax12(ID_Dict['Joint_6']), Ax12(ID_Dict['Joint_7'])

#Set speed
J1.set_moving_speed(100)
J2.set_moving_speed(100)
J3.set_moving_speed(100)
J4.set_moving_speed(100)
J5.set_moving_speed(100)
J6.set_moving_speed(100)
J7.set_moving_speed(100)

# pass in AX12 object
try:
    X, Y, Z, THETA_DEG = 10.0, 14.0, -5, 0 #change according to input of CV. Create function later.
    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = get_angle_from_position(X, Y, Z, THETA_DEG)

    print("theta1: ", theta_1)
    print("theta2: ", theta_2)
    print("theta3: ", theta_3)
    print("theta4: ", theta_4)
    print("theta5: ", theta_5)
    print("theta6: ", theta_6)

    print()

    print("bit1: ",deg_to_bits(theta_1))
    print("bit2: ",deg_to_bits(theta_2))
    print("bit3: ",deg_to_bits(theta_3))
    print("bit4: ",deg_to_bits(theta_4))
    print("bit5: ",deg_to_bits(theta_5))
    print("bit6: ",deg_to_bits(theta_6))
    #move_motor_from_position(X, Y, Z, THETA_DEG)

    print("current position J1: ",J1.id)

except:
    pass

# disconnect
# J1.set_torque_enable(0)
# J2.set_torque_enable(0)
# J3.set_torque_enable(0)
# J4.set_torque_enable(0)
# J5.set_torque_enable(0)
# J6.set_torque_enable(0)
#Ax12.disconnect()
