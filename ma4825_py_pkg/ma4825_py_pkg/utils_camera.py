"""
Author: Kenzhi Wong, Reynald Wong
Project: Stamp-e with Conveyer Belt

This program is the coordinate transformation library.
It takes the coordinate (xpix,ypix) input from the pixel camera, 
and returns the coordinate (x,y) wrt to the robot's frame.
"""

import math
import numpy as np
import sympy as sp

# Initializing Value
xrange = 22
yrange = 18
x_edge_cam_to_centre = 20.5
y_edge_cam_to_centre = 8

# Orientation of the box relative to positive x-axis
def get_box_orientation(x1blue, y1blue, x2blue, y2blue, x1red, y1red, x2red, y2red):
    
    # Getting Object's Centre Coordinate in Pixels.
    xbluepixel = x1blue + 0.5*(x2blue-x1blue)   
    ybluepixel = y1blue + 0.5*(y2blue-y1blue)
  
    xredpixel = x1red + 0.5*(x2red-x1red)
    yredpixel = y1red + 0.5*(y2red-y1red)

    xn_red = (xredpixel/640)*xrange
    yn_red = (1 - yredpixel/480)*yrange

    xn_blue = (xbluepixel/640)*xrange
    yn_blue = (1 - ybluepixel/480)*yrange

    front = yn_red - yn_blue
    side = xn_red - xn_blue

    theta_rad = np.arctan(abs(front/side))
    if front > 0 and side < 0 : theta_rad = -theta_rad
    elif front > 0 and side > 0 : theta_rad = theta_rad - 180
    elif front < 0 and side < 0 : theta_rad = theta_rad
    elif front < 0 and side > 0 : theta_rad = 180 - theta_rad
    
    return (np.degrees(theta_rad))

def get_object_coordinate_wrt_centre_of_arm_total(x1box, y1box, x2box, y2box):
    # Getting Object's Centre Coordinate in Pixels.
    xboxpixel = x1box + 0.5*(x2box-x1box)
    yboxpixel = y1box + 0.5*(y2box-y1box)
    
    # Object's Centre Coordinate in cm with reference to the camera's scope
    xN = (xboxpixel/640)*xrange
    yN = (1 - yboxpixel/480)*yrange
    
    # Object Total Coordinate
    x = x_edge_cam_to_centre + xN
    y = y_edge_cam_to_centre + yN

    return x,y

