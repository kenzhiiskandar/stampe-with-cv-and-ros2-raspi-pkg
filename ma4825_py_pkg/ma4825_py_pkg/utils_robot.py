"""
Author: Bryan Timothy Galenius
Project: Stamp-e with Conveyer Belt

This program is the inverse kinematics library for the project.
"""

import math
import numpy as np
import sympy as sp

def rad_to_deg(angle):
    return angle/math.pi*180

def deg_to_rad(angle):
    return angle*math.pi/180

def sine_cosine_to_rad(sine_value, cosine_value):
    #Defining the servo angle to range from -180 deg to 180 deg:
    angle_from_quadrant = {(True, True): math.acos(cosine_value),                         #Quadrant 1. Can use either asin() or acos()
                           (True, False): math.acos(cosine_value),                      #Quadrant 2. using acos() will give 2nd quadrant answer
                           (False, False): (-1*math.asin(sine_value))-math.pi,          #Quadrant 3. using asin() will give -theta when it's theta-pi.
                           (False, True): math.asin(sine_value)                         #Quadrant 4. using asin() will give -theta (4th quadrant answer)
                           }

    positive_sine = (sine_value >= 0)
    positive_cosine = (cosine_value >= 0)
    return angle_from_quadrant[(positive_sine, positive_cosine)]


def get_angle_from_position(X: float,Y: float,Z: float,THETA_DEG: float) -> list:
    X, Y = -X, -Y  # When defining the Denavit-Hartenberg frames I accidentally defined it the opposite way :P

    #Defining constants and constraints
    zc1,r1,r2,r3,r4,r5,r6, xc = 4.7, 2.5, 6.8, 14.7, 6.5, 7.5, 5.6, 2.2

    # theta_2 + theta_3 must be bigger than 75 vertical stamping geometric constraint (add 1 degree for safety measures)
    minimum_total_theta_2_3 = deg_to_rad(75.6)

    #Significant figures for output angles
    significant_figures = 1

    ################################# From geometry, obtain theta_1, theta_2, theta_3 #################################
    try:
        theta_1 = math.atan(-X/Y)
    except ZeroDivisionError: # If Y==0
        if X > 0:
            theta_1 = math.pi/2
        elif X < 0:
            theta_1 = -math.pi/2
        else: #X==0
            theta_1 = 0

    #Simultaneous Equations
    # Define symbolic variables
    x1, y1 = sp.symbols('x1 y1') # x1=sin(theta_2), y1=sin(theta_2 + theta_3)
    x2, y2 = sp.symbols('x2 y2') # x2=cos(theta_2), y2=cos(theta_2 + theta_3)

    # Define the equations
    if theta_1 % math.pi < 1e-4:  # if theta_1 is 0 or 180 deg, division by sine is unacceptable. Use cosine only
        eq1 = sp.Eq(r2 * x1 + r3 * y1, (-Y / sp.cos(theta_1)) - r1 - xc * sp.sin(theta_1))
    elif (theta_1 % math.pi) - (
            math.pi / 2) < 1e-4:  # if theta_1 is 90 or 270 deg, division by cosine is unacceptable. Use sine only
        eq1 = sp.Eq(r2 * x1 + r3 * y1, (X / sp.sin(theta_1)) - r1 + xc * sp.cos(theta_1))
    else:
        eq1 = sp.Eq(r2 * x1 + r3 * y1, (X / (2 * sp.sin(theta_1))) - (Y / (2 * sp.cos(theta_1))) - r1 - (xc * (sp.sin(theta_1) - sp.cos(theta_1)) / 2))

    eq2 = sp.Eq(r2 * x2 + r3 * y2, Z-zc1+r4+r5+r6)
    eq3 = sp.Eq(x1*x1 + x2*x2, 1)
    eq4 = sp.Eq(y1*y1 + y2*y2, 1)

    # Solve the equations symbolically
    solution = sp.solve((eq1, eq2, eq3, eq4), (x1, x2, y1, y2))

    if solution==[]:
        raise ValueError("No Solution Found: Specified point is unreachable")

    # theta_2 and theta_3 have 2 possibilities: (theta_2 = a & theta_2+theta_3 = b) or (theta_2 = b & theta_2+theta_3 = a)
    #calculating a and b
    a = sine_cosine_to_rad(solution[0][0], solution[0][1])
    b = sine_cosine_to_rad(solution[0][2], solution[0][3])

    # choose the one where theta_2 < theta_2+theta_3 for better range of motion without much transition
    # also to maximize theta_2+theta+3 such that minimum_total_theta_2_3 is exceeded
    if a<b:
        if b < minimum_total_theta_2_3:
            print(b)
            raise ValueError("Specified coordinate is impossible to achieve when end-effector direction is required to be vertically downwards")
        theta_2 = a
        theta_3 = b-a
    else:
        if a < minimum_total_theta_2_3:
            print(a)
            raise ValueError("Specified coordinate is impossible to achieve when end-effector direction is required to be vertically downwards")
        theta_2 = b
        theta_3 = a - b


    ################################# From HTM, obtain theta_4, theta_5, theta_6 #################################
    #From frame orientation analysis
    R06_matrix = np.array([
        [math.cos(deg_to_rad(THETA_DEG)), math.sin(deg_to_rad(THETA_DEG)), 0],
        [math.sin(deg_to_rad(THETA_DEG)), -1*math.cos(deg_to_rad(THETA_DEG)), 0],
        [0, 0, -1]
    ])

    sin_a, sin_b, sin_c= math.sin(theta_1), math.sin(theta_2), math.sin(theta_3)
    cos_a, cos_b, cos_c = math.cos(theta_1), math.cos(theta_2), math.cos(theta_3)

    R03_matrix = np.array([
        [(sin_a*sin_b*cos_c)+(sin_a*cos_b*sin_c), (sin_a*cos_b*cos_c)-(sin_a*sin_b*sin_c), cos_a],
        [(-cos_a*sin_b*cos_c)-(cos_a*cos_b*sin_c), (cos_a*sin_b*sin_c)-(cos_a*cos_b*cos_c), sin_a],
        [(cos_b*cos_c)-(sin_b*sin_c), (-cos_b*sin_c)-(sin_b*cos_c), 0]
    ])

    R36_matrix = np.dot(np.linalg.inv(R03_matrix), R06_matrix)

    #From HMT we get:
    theta_5 = math.asin(R36_matrix[2][2]) # should be 0
    theta_4 = sine_cosine_to_rad(R36_matrix[1][2]/math.cos(theta_5), R36_matrix[0][2]/math.cos(theta_5))
    theta_6 = sine_cosine_to_rad(-1*R36_matrix[2][1]/math.cos(theta_5), R36_matrix[2][0] / math.cos(theta_5))

    # Change to degree
    theta_1 = rad_to_deg(theta_1)
    theta_2 = rad_to_deg(theta_2)
    theta_3 = rad_to_deg(theta_3)
    theta_4 = rad_to_deg(theta_4)
    theta_5 = rad_to_deg(theta_5)
    theta_6 = rad_to_deg(theta_6)


    #Minor adjustments
    if theta_2 - 7.0 >= 0.5:
        theta_2 = theta_2+(((3/64)*(-Y-7)*(-Y-7))-((11/8)*(-Y-7))-5)

    #Round to the specified significant figures
    theta_1 = round(theta_1, significant_figures)
    theta_2 = round(theta_2, significant_figures)
    theta_3 = round(theta_3, significant_figures)
    theta_4 = round(theta_4, significant_figures)
    theta_5 = round(theta_5, significant_figures)
    theta_6 = round(theta_6, significant_figures)

    

    return(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)