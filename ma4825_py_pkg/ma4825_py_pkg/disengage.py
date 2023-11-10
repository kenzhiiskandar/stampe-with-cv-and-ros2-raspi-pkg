"""
Author: Bryan Timothy Galenius
Project: MA4825 - White Box Coordinate and Orientation Detection

This program is used to disable all the torque in all the robot's joints.
"""

from Ax12 import Ax12

#ID Dictionary
ID_Dict = {
    'Joint_1': 6,
    'Joint_2': 3,
    'Joint_3': 1,
    'Joint_4': 4,
    'Joint_5': 5,
    'Joint_6': 2,
    'Joint_7': 7
}

# e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
Ax12.DEVICENAME = '/dev/ttyUSB0'

# sets baudrate and opens com port
Ax12.BAUDRATE = 1_000_000
Ax12.connect()

# create AX12 instances for every joint
J1, J2, J3, J4, J5, J6, J7 = Ax12(ID_Dict['Joint_1']), Ax12(ID_Dict['Joint_2']), Ax12(ID_Dict['Joint_3']), Ax12(ID_Dict['Joint_4']), Ax12(ID_Dict['Joint_5']), Ax12(ID_Dict['Joint_6']), Ax12(ID_Dict['Joint_7'])


# disconnect
J1.set_torque_enable(0)
J2.set_torque_enable(0)
J3.set_torque_enable(0)
J4.set_torque_enable(0)
J5.set_torque_enable(0)
J6.set_torque_enable(0)
J7.set_torque_enable(0)
Ax12.disconnect()
