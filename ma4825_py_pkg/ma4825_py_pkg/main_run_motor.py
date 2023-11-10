"""
Author: Kenzhi Wong, Bryan Timothy Galenius
Project: Project: Stamp-e with Conveyer Belt

This program is the main program to control the movement of the robot's with respect to the given coordinate.
In summary the program does the following:
1. It takes the coordinate wrt the centre of robot's arm via ROS2 Topic.
2. It moves all the robot's joints to a particular location.
3. It stamps the box using GPIO Solenoid.
"""

# ROS2 Python Standard Library
import rclpy
from rclpy.node import Node

# ROS2 Custom Interfaces
from std_msgs.msg import String
from my_interfaces.msg import BoxCoord, RedDotCoord, BlueDotCoord

# Robot Custom Library
import sys
print(sys.path)
sys.path.append("/home/kenzhi/ros2_ws/install/ma4825_py_pkg/lib/python3.10/site-packages/ma4825_py_pkg")
import utils_camera
import utils_robot
from Ax12 import Ax12

# Python Library
import time

# Raspberry Pi GPIO Library
import RPi.GPIO as GPIO


class MyRunMotor(Node):
    # ================================= INITIALIZATION =================================
    def __init__(self):
        super().__init__('run_motor') #node name

        # ================================= ROS2 - Subscribing to the Coordinates =================================
        self.subscribe_box = self.create_subscription(BoxCoord, "box_coordinate", self.listener_box_callback, 10)
        self.subscribe_red_dot = self.create_subscription(RedDotCoord, "red_dot_coordinate", self.listener_red_callback, 10)
        self.subscribe_blue_dot = self.create_subscription(BlueDotCoord, "blue_dot_coordinate", self.listener_blue_callback, 10)

        # ================================= Robot - Configuring Hardware Interfaces =================================
        # ID Dictionary
        ID_Dict = {
            'Joint_1': 6,
            'Joint_2': 3,
            'Joint_3': 1,
            'Joint_4': 4,
            'Joint_5': 5,
            'Joint_6': 2,
            'ConvBelt' : 7}

        # e.g 'COM3' windows or '/dev/ttyUSB0' for Linux
        Ax12.DEVICENAME = '/dev/ttyUSB0'
    
        # Set baudrate and opens com port
        Ax12.BAUDRATE = 1_000_000
        Ax12.connect()

        # Create AX12 instances for every joint
        self.J1, self.J2, self.J3, self.J4, self.J5, self.J6, self.ConvBeltMotor = \
            Ax12(ID_Dict['Joint_1']), Ax12(ID_Dict['Joint_2']), Ax12(ID_Dict['Joint_3']), \
            Ax12(ID_Dict['Joint_4']), Ax12(ID_Dict['Joint_5']), Ax12(ID_Dict['Joint_6']), \
            Ax12(ID_Dict['ConvBelt'])

        # Set speed
        self.J1.set_moving_speed(100)
        self.J2.set_moving_speed(100)
        self.J3.set_moving_speed(100)
        self.J4.set_moving_speed(100)
        self.J5.set_moving_speed(100)
        self.J6.set_moving_speed(100)

        # Declare Variable
        self.ready1 = False
        self.ready2 = False
        self.ready3 = False
        self.robot_in_origin = True
        self.robot_in_position = False
        self.X_offset = 30 
        self.travel_time = 11.2 #time travel to x offset

        # Configure GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(26, GPIO.OUT)

        # ================================= Robot - Starting the Program =================================
        # Turn the Conveyer Belt Motor
        self.ConvBeltMotor.set_moving_speed(100)

        # Set robot arm to default coordinate
        self.move_motor_to_default_position()

        # ================================= ROS2 & Robot - Set the timer to call the main program =================================
        self.subscribe_box
        self.subscribe_red_dot
        self.subscribe_blue_dot

        self.create_timer(0.2, self.turning_motor_program)

    # ================================= ROS2 & Robot - Main Program =================================
    def turning_motor_program(self):
        
        if (self.robot_in_origin == True) and (self.ready1 == True) and (self.ready2 == True) and (self.ready3 == True):
            
            # The coordinate is received, wait for one cycle of robot to be executed.
            self.ready1, self.ready2, self.ready3 = False, False, False
            time0_copy = self.time0

            # Move robot to default upped position
            self.move_motor_to_default_position_add()
            time.sleep(0.5)

            # Move robot to middle location
            self.move_motor_to_default_position_ready()
            time.sleep(0.5)

            # Get values box's coordinate wrt to robot arm
            X, Y = utils_camera.get_object_coordinate_wrt_centre_of_arm_total(self.x1box, self.y1box, self.x2box, self.y2box)
            X, Y = round(X,1), round(Y,1) - 1
            print("X,Y Real Time coordinate of the box wrt to arm : " ,X,Y)
            
            X = round(X - self.X_offset)
            print("X,Y Offset coordinate of the box wrt to arm : ",X,Y)
            
            # Get the rotation of the box wrt the arm
            THETA_DEG = utils_camera.get_box_orientation(self.x1blue, self.y1blue, self.x2blue, self.y2blue, self.x1red, self.y1red, self.x2red, self.y2red)
            THETA_DEG = round(THETA_DEG, 1)
            print("Turning degree of Joint 6 : ", THETA_DEG)
            
            # Configure the height of the box wrt to the (0,0,0) point
            Z = -10

            # Perform Inverse Kinematics
            self.theta_1, self.theta_2, self.theta_3, self.theta_4, self.theta_5, self.theta_6 = utils_robot.get_angle_from_position(X, Y, Z, THETA_DEG)
            self.printing()

            # Move robot to offset location, and pull up the stample
            self.move_motor_from_position(X, Y, Z, THETA_DEG)
            GPIO.output(26, GPIO.HIGH)
            print("aActuate Actuator ... Move the solenoid position up ....")
            print("Moving the robot ...")

            # Wait for the robot to be in position
            while (((self.J1.get_present_position() < self.deg_to_bits(self.theta_1) - 2) or (self.J1.get_present_position() > self.deg_to_bits(self.theta_1) + 2)) and \
                ((self.J2.get_present_position() < self.deg_to_bits(self.theta_2) - 2) or (self.J2.get_present_position() > self.deg_to_bits(self.theta_2) + 2)) and \
                    ((self.J3.get_present_position() < self.deg_to_bits(self.theta_3) - 2) or (self.J3.get_present_position() > self.deg_to_bits(self.theta_3) + 2)) and \
                        ((self.J4.get_present_position() < self.deg_to_bits(self.theta_4) - 2) or (self.J4.get_present_position() > self.deg_to_bits(self.theta_4) + 2)) and \
                            ((self.J5.get_present_position() < self.deg_to_bits(self.theta_5) - 2) or (self.J5.get_present_position() > self.deg_to_bits(self.theta_5) + 2)) and \
                                ((self.J6.get_present_position() < self.deg_to_bits(self.theta_6) - 2) or (self.J6.get_present_position() > self.deg_to_bits(self.theta_6) + 2))):
                pass
            self.robot_in_position = True
            self.robot_in_origin = False
            print("Robot is in position.")

            # If the robot is already in offset position, wait for the box to arrive, and stamp
            if self.robot_in_position == True:
                self.time1 = time.time()
                while ((self.time1 - time0_copy) < self.travel_time - 0.5):
                    self.time1 = time.time()
                # Stamp using solenoid
                GPIO.output(26, GPIO.LOW)
                time.sleep(0.1)
                GPIO.output(26, GPIO.HIGH)
    
            print("Stamping is done.")
            
            # After stamping, move robot to the middle location
            self.move_motor_to_default_position_ready()
            GPIO.output(26, GPIO.LOW)
            time.sleep(0.5)

            # Move robot to default upped position
            self.move_motor_to_default_position_add()
            time.sleep(0.5)
            
            # Move robot to the origin
            self.move_motor_to_default_position()
            print("Moving the robot to origin ... ")
            self.robot_in_position = False
            self.robot_in_origin = True
            print("Robot is at origin.")

    # ================================= ROS2 - Subscribers Main Program =================================
    def listener_box_callback(self, msg_):
        self.get_logger().info('I heard x1_bx: "%s"' % msg_.x1_bx)
        self.get_logger().info('I heard y1_bx: "%s"' % msg_.y1_bx)
        self.get_logger().info('I heard x2_bx: "%s"' % msg_.x2_bx)
        self.get_logger().info('I heard y2_bx: "%s"' % msg_.y2_bx)
        self.get_logger().info('I heard x1_bx: "%s"' % msg_.sent_done)
        self.x1box, self.y1box, self.x2box, self.y2box = msg_.x1_bx, msg_.y1_bx, msg_.x2_bx, msg_.y2_bx
        self.ready1 = msg_.sent_done
        self.time0 = time.time()
    
    def listener_red_callback(self, msg_):
        self.get_logger().info('I heard x1_r: "%s"' % msg_.x1_r)
        self.get_logger().info('I heard y1_r: "%s"' % msg_.y1_r)
        self.get_logger().info('I heard x2_r: "%s"' % msg_.x2_r)
        self.get_logger().info('I heard y2_r: "%s"' % msg_.y2_r)
        self.get_logger().info('I heard x1_bx: "%s"' % msg_.sent_done)
        self.x1red, self.y1red, self.x2red, self.y2red = msg_.x1_r, msg_.y1_r, msg_.x2_r, msg_.y2_r
        self.ready2 = msg_.sent_done

    def listener_blue_callback(self, msg_):
        self.get_logger().info('I heard x1_b: "%s"' % msg_.x1_b)
        self.get_logger().info('I heard y1_b: "%s"' % msg_.y1_b)
        self.get_logger().info('I heard x2_b: "%s"' % msg_.x2_b)
        self.get_logger().info('I heard y2_b: "%s"' % msg_.y2_b)
        self.get_logger().info('I heard x1_bx: "%s"' % msg_.sent_done)
        self.x1blue, self.y1blue, self.x2blue, self.y2blue = msg_.x1_b, msg_.y1_b, msg_.x2_b, msg_.y2_b
        self.ready3 = msg_.sent_done

    # ================================= Robot - Move Function =================================
    def deg_to_bits(self, theta):
        return int((theta+150.0)/0.29297)

    def move_motor_from_position(self, X: float,Y: float,Z: float,THETA_DEG: float):
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = utils_robot.get_angle_from_position(X, Y, Z, THETA_DEG)
        self.J1.set_goal_position(self.deg_to_bits(theta_1))
        self.J2.set_goal_position(self.deg_to_bits(theta_2))
        self.J3.set_goal_position(self.deg_to_bits(theta_3))
        self.J4.set_goal_position(self.deg_to_bits(theta_4))
        self.J5.set_goal_position(self.deg_to_bits(theta_5))
        self.J6.set_goal_position(self.deg_to_bits(theta_6))
    
    def move_motor_to_default_position(self):
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = utils_robot.get_angle_from_position(-23, 0, -17, 0)
        self.J1.set_goal_position(self.deg_to_bits(theta_1))
        self.J2.set_goal_position(self.deg_to_bits(theta_2))
        self.J3.set_goal_position(self.deg_to_bits(theta_3))
        self.J4.set_goal_position(self.deg_to_bits(theta_4))
        self.J5.set_goal_position(self.deg_to_bits(theta_5))
        self.J6.set_goal_position(self.deg_to_bits(theta_6))

    def move_motor_to_default_position_ready(self):
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = utils_robot.get_angle_from_position(-13, 12, -10, 0)
        self.J1.set_goal_position(self.deg_to_bits(theta_1))
        self.J2.set_goal_position(self.deg_to_bits(theta_2))
        self.J3.set_goal_position(self.deg_to_bits(theta_3))
        self.J4.set_goal_position(self.deg_to_bits(theta_4))
        self.J5.set_goal_position(self.deg_to_bits(theta_5))
        self.J6.set_goal_position(self.deg_to_bits(theta_6))
    
    def move_motor_to_default_position_add(self):
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = utils_robot.get_angle_from_position(-23, 0, -10, 0)
        self.J1.set_goal_position(self.deg_to_bits(theta_1))
        self.J2.set_goal_position(self.deg_to_bits(theta_2))
        self.J3.set_goal_position(self.deg_to_bits(theta_3))
        self.J4.set_goal_position(self.deg_to_bits(theta_4))
        self.J5.set_goal_position(self.deg_to_bits(theta_5))
        self.J6.set_goal_position(self.deg_to_bits(theta_6))

    def printing(self):
        # Printing rotation of each joint
        print("theta1: ", self.theta_1)
        print("theta2: ", self.theta_2)
        print("theta3: ", self.theta_3)
        print("theta4: ", self.theta_4)
        print("theta5: ", self.theta_5)
        print("theta6: ", self.theta_6)

        # Printing the bit of the joints
        print("bit1: ",self.deg_to_bits(self.theta_1))
        print("bit2: ",self.deg_to_bits(self.theta_2))
        print("bit3: ",self.deg_to_bits(self.theta_3))
        print("bit4: ",self.deg_to_bits(self.theta_4))
        print("bit5: ",self.deg_to_bits(self.theta_5))
        print("bit6: ",self.deg_to_bits(self.theta_6))

        # Printing current's bit position
        print("position now J1:", self.J1.get_present_position())
        print("position now J2:", self.J2.get_present_position() )
        print("position now J3:", self.J3.get_present_position() )
        print("position now J4:", self.J4.get_present_position() )
        print("position now J5:", self.J5.get_present_position() )
        print("position now J6:", self.J6.get_present_position() )

# Perform OOP
def main(args=None):
    rclpy.init(args=args)
    node = MyRunMotor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
