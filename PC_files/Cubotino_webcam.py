#!/usr/bin/env python
# coding: utf-8


''' 
#############################################################################################################
#  Andrea Favero          rev. 17 January 2023
#
#  Script made to learn computer vision and to improve coding skills
#
#  The script reads the Rubik's cube status, by presenting the cube faces manually to the webcam
#  This script version is used on Cubotino, a small Rubik's cube solver robot https://youtu.be/ZVbVmCKwYnQ
#  This scrip is imported by by Cubotino_GUI.py
#
#  Developped on:
#  --> W10 PC with Python ver: 3.8.12 [MSC v.1916 64 bit (AMD64)] and cv2 ver: 4.5.1
#
#  Verified on:
#  --> W10 PC with Python ver: 3.9.7 |packaged by conda-forge| [MSC v.1916 64 bit (AMD64)] and cv2 ver: 4.5.1
#
#
# Starting from 13 may 2022 the scipy library isn't anymore needed (matrix distance is now based on Numpy)
#
#############################################################################################################
'''


import cv2
import numpy as np
import math
import statistics
import time
import datetime as dt
import json
import os

CALIBRATION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cube_color_calibration.json')
calibrated_colors = {}   # {'white': (B,G,R), ...}, filled by load_color_calibration()/calibrate_cube_colors()

# fallback reference palette (BGR), used for live display classification when no calibration has been done yet
DEFAULT_REF_COLORS = {'white':(255,255,255), 'red':(0,0,204), 'green':(0,132,0),
                       'yellow':(0,245,245), 'orange':(0,128,255), 'blue':(204,0,0)}

CYCLE_COLORS = ['white', 'green', 'yellow', 'orange', 'red', 'blue']   # order a preview cell's color cycles through on click

# the standard color scheme, and unfolded-net layout (col,row in a 4x3 grid), for the 6 scan-order face letters
STD_LETTER_COLOR = {'U':'white', 'R':'red', 'F':'green', 'D':'yellow', 'L':'orange', 'B':'blue'}
NET_LAYOUT = {'U':(1,0), 'L':(0,1), 'F':(1,1), 'R':(2,1), 'B':(3,1), 'D':(1,2)}


try:
    import sys, platform
    print('\n===================  webcam module AF (17 January 2023)  ===========================')
    print(f'Running on: {platform.system()}')         # print to terminal, the platform used
    print(f'Python version: {sys.version}')           # print to termina the python version
    print(f'CV2 version: {cv2.__version__}')          # print to terminal the cv2 version
    print()
except:
    pass


# there are two import attempts for Kociemba solver
try:                                                  # attempt
    import solver as sv                               # import Kociemba solver copied in robot folder
    print('\nimported the installed twophase solver')  # feedback is printed to the terminal
    solver_found = True                               # boolean to track no exception on import the copied solver
except:                                               # exception is raised if no library in folder or other issues
    solver_found = False                              # boolean to track exception on importing the copied solver
    
if not solver_found:                                  # case the library was not in folder
    try:                                              # attempt
        import twophase.solver as sv                  # import Kociemba solver installed
        print('\nimported the copied twophase solver') # feedback is printed to the terminal
        twophase_solver_found = True                  # boolean to track no exception on import the installed solver
    except:                                           # exception is raised if no library in venv or other issues
        twophase_solver_found = False                 # boolean to track exception on importing the installed solver

if not solver_found and not twophase_solver_found:    # case no one solver has been imported
    print('\n(Kociemba) twophase solver not found') # feedback is printed to the terminal
print('====================================================================================\n')









def webcam(cam_num, cam_width=640, cam_height=360):
    ''' Set the camera and its resolution'''
    global camera
    
    cam_num=int(cam_num)          # cam_num =0 is typically the one integrated on laptops
    camera = cv2.VideoCapture(cam_num, cv2.CAP_DSHOW)         # camera object
  
    camera_width_resolution = int(cam_width)                  # laptop camera, width resolution setting
    camera_height_resolution = int(cam_height)                # laptop camera, height resolution setting
    
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width_resolution)    # camera width resolution is set  
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height_resolution)  # camera height resolution is set
    
    width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))         # return the camera reading width 
    height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))       # return the camera reading higth 
    
    if side==0 and debug:
        print(f'Camera resolution: {width} x {height}\n')
    time.sleep(0.5)
#         clear_terminal()
    return camera, width, height







def read_camera():
    ''' Returns the camera reading, and dimensions '''

    ret, frame = camera.read()                     # ret is the boolean if the image array is available, and the image
    if ret==False:
        print('Webcam frame not available: ret variable == False')
    else:
        frame = cv2.flip(frame, 1)                 # horizontal flip, so the feed behaves like a mirror instead of a security cam
        frame, w, h = frame_cropping(frame, width, height)     # frame is cropped in order to limit the image area to analyze
        return frame, w, h







def frame_cropping(frame, width, height):
    ''' Frame cropping, to increase overal speed
    Frame width, and other parameters (i.e. amount of facelets in width), is used to later define the
    min and max area to accept facelet countours for the cube facelets edge detection.'''
    
    global first_cycle, k_kernel, d_iterations, e_iterations, facelets_in_width, min_area, max_area, crop_at_right
    
    if crop_at_right!=0:                      # case is requested to crp pixels at the right frame side
        frame = frame[ : , : -crop_at_right]  # frame is sliced
        w = width-crop_at_right               # frame width
    
    w = width     # frame width
    h = height    # frame height

    if first_cycle==True:        # below part is done once

    # min and max area for facelet contour acceptance, are calculated in proportion to other parameters
        min_area = int(width/facelets_in_width-k_kernel*(1+d_iterations-e_iterations))**2
        max_area = int(width/5.5-k_kernel*(1+d_iterations-e_iterations))**2
#             print(f'Min area:{min_area},   Max area:{max_area}')
        first_cycle=False

    return frame, w, h







def edge_analysis(frame):
    ''' Image analysis that returns a black & white image, based on the colors borders.''' 
        
    global k_kernel, d_iterations, e_iterations
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    # from BGR color space to gray scale
    blurred = cv2.GaussianBlur(gray, (9, 9), 0)       # low pass filter is applied, with a 9x9 gaussian filter
    canny = cv2.Canny(blurred, 10, 30)                # single pixel edges, having intensity gradient between  10 and 30                      
    
    kernel = np.ones((k_kernel, k_kernel), np.uint8)  # the kernel is set at setup() function
    dilated = cv2.dilate(canny, kernel, iterations = d_iterations)  # higher 'iterations' is overall faster
    eroded = cv2.erode(dilated, kernel, iterations = e_iterations)  # smaller 'iterations' keeps the contour apart from the edges  
    
#     cv2.imshow('Frame', frame)
#     cv2.imshow('Gray', gray)
#     cv2.imshow('blurred', blurred)
#     cv2.imshow('Canny', canny) 
#     cv2.imshow('Dilated', dilated)
#     cv2.imshow('Eroded', eroded)
    
    return eroded







def square_check(data):  
    ''' Sanity check if reading a contour with square like shape; Argument is a contour
    Calculates quadrilateral's edge delta lenght: [(max edge - min edge)/average of sides length]
    Calculates the ratio between the 2 diagonals (rhonbus axes): max diagonal / min diagonal
    These parameter are later used to verify if the contour can be considered like a square.'''
    
    edges=[]   # list of the 4 edges of the quadrilateral
    axes=[]    # List of axes of symmetry length of the rhombus
    for i in range(len(data)):
        j = i + 1
        if i==3:
            j=0
        edges.append(math.sqrt((data[j][0]-data[i][0])**2 + (data[j][1]-data[i][1])**2))  # list of the 4 edge's length
        edge_delta = (max(edges)-min(edges))*4/sum(edges)  # max side delta over the mean
    
    for i in range(2):
        j = i + 2
        axes.append(math.sqrt((data[j][0]-data[i][0])**2 + (data[j][1]-data[i][1])**2))  # list of the 2 rhobus axes
        axes_delta = min(axes)/max(axes)
    
    return edge_delta, axes_delta







def inclination_check(data):
    ''' Calculates the facelets inclination from the horizon, usefull while presenting the cube manually to the webcam
    Argument is a contour
    The inclination is calculates on the first square edge (first 2 square points after being ordered)
    This info is later used, to only accept facelets having a contour within an inclination limit from horizon.'''
    

    inclination = 0
    if data[1][1] != data[0][1]: # edge not horizontal (by chance, and giving error on math calculation)
        inclination = - math.atan(((data[1][1]-data[0][1]))/((data[1][0]-data[0][0])))*180/math.pi #inclination in degrees
    else : inclination == 0
    
    return inclination







def cube_inclination(facelets):
    ''' Calculates the average cube inclination from the horizon, in radians.
    Argument is the facelets list with first N contours detected
    The inclination is calculated on the first square edge, the top one, based on the first
    two square points after ordering them CW.'''

    incl_list=[]
    for i in range(len(facelets)):
        # based on the inclination between two point of each facelet
        data=facelets[i].get('cont_ordered')
        if data[1][1] != data[0][1]: # edge not horizontal (by chance, and giving error on math calculation)
            inclination = - math.atan(((data[1][1]-data[0][1]))/((data[1][0]-data[0][0]))) #inclination in radians
        else:
            inclination = 0
        incl_list.append(inclination)
    return statistics.median(incl_list)






def facelet_grid_pos(x, y):
    """returns the face facelet number, based on the coordinates of the contour center, and other parameters.
        This is used to map which in facelets a contour has been detected, and which not."""
    
    # Below dict has the face facelets number as value; keys are facelet coordinate expressed as string combination of column and row
    facelet_number = {'11':0, '21':1, '31':2,
                      '12':3, '22':4, '32':5,
                      '13':6, '23':7, '33':8}
    
    facelet = str(x) + str(y)                 # facelet string combining column (x) and row (y) of the facelet
    if facelet in facelet_number:             # case the facelecet is in the dict
        return facelet_number[facelet]        # facelet number is returned






def median_point(data):
    """Function returns the coordinate of the mediant point out of an arreay of points."""
    return tuple(np.int_(np.median(data, axis=0)))






def median_dist(data, point):
    """Function calculates the distance from a point to a list of coordinates.
        It returns an array of distances, and the median distance."""
    s = np.array(data)
    p = np.array(point)
    dist = np.linalg.norm(p - s, axis=1)
    med_dist = np.median(dist)
    return dist, med_dist






def rotate(points, origin, angle):
    """Function to rotate an array of 2d coordinates arount an origin by an angle in radians."""
    R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle),  np.cos(angle)]])
    o = np.atleast_2d(origin)
    p = np.atleast_2d(points)
#     if debug:     # case the debug variable is set True
#         print("--> points to be rotated", points)
#         print("--> points rotated:", p)
    return np.int_(np.squeeze((R @ (p.T-o.T) + o.T).T))






def estimate_facelets(facelets, angle):
    """Estimates the remaing facelets location, when there are at least N detected facelets.
        This function is interrupted if one row or column is fully empty; In this way the cube width and height
        is well known, enabling a good estimation for the missed facelets position.
    
    
                x_1     x_2     x_3              X
           --|----------------------------------->
             |       |       |       |
        y_1  |   1   |   2   |   3   |   row 1
             |       |       |       |
             |-----------------------
             |       |       |       |
        y_2  |   4   |   5   |   6   |   row 2
             |       |       |       |
             |----------------------- 
             |       |       |       |
        y_3  |   7   |   8   |   9   |   row 3
             |       |       |       |
             |-----------------------
             |
             | clmn1   clmn2   clmn3
             |
             |
           Y v
         
    """

    angle = 1.5*angle     # detected contours tends to be less inclined from horizon than the facelets
    
# current cube face characteristics, based on detected facelets
    cont_xy = []                               # empty list to fill with contours centers x,y coordinates
    cont_x = []                                # empty list to fill with contours centers x coordinates
    cont_y = []                                # empty list to fill with contours centers y coordinates
    cont_area = []                             # empty list to fill with contours areas
    
    for i in range(len(facelets)):             # iteration over the quantity of facelets already detected
        x = facelets[i]['cx']                  # x coordinate for the facelet[i] center
        y = facelets[i]['cy']                  # y coordinate for the facelet[i] center
        cont_xy.append((x, y))                 # all the contours centers x,y coordinates are listed
        cont_x.append(x)                       # all the contours centers x coordinates are listed
        cont_y.append(y)                       # all the contours centers y coordinates are listed
        cont_area.append(facelets[i]['area'])  # all the contours areas are listed
        
    xy_0 = median_point(cont_xy)               # median point from the detected centers
    x0 = xy_0[0]                               # x coordinate of the median point
    y0 = xy_0[1]                               # y coordinate of the median point
        
    if len(cont_xy) != 0:                      # case there are facelets center coordinates
        cont_xy_rot = rotate(cont_xy, xy_0, angle) # center coordinates of the detected facelets are rotated to 'horizon'
        dist, avg_pts_distance = median_dist(cont_xy, xy_0) # distance from median point (array), and median distance value
    
# check if there are contours too far to be part of the cube face
    d_to_exclude = []                          # empty list to be populated with the index of facelets
    for i in range(len(dist)):                 # iteration on the array of distances from the median point
        if dist[i]/avg_pts_distance > 1.5:     # case the distance is more than 1.5 times the median distance
            d_to_exclude.append(i)             # index of that facelet is added to the list
    if len(d_to_exclude)>=1:                   # case facelets to be removed, cause excess distance from median point
        d_to_exclude.sort(reverse=True)        # list order is reversed, making easy to remove
        for i in d_to_exclude:                 # iteration on the list of contour index to be removed
            facelets.pop(i)                    # contour too far are removed from list of potential facelets                      

# search which are the missed facelets, after removing facelerts being too far from the center of detected facelets
    med_a = int(statistics.median(cont_area))  # median area for the facelets in function argument
    cx = cont_x.copy()         # list copy of contours centers x coordinates
    cy = cont_y.copy()         # list copy of contours centers y coordinates
    cont_x.sort()              # sorted list with contours centers x coordinates
    cont_y.sort()              # sorted list with contours centers y coordinates
    
    x_1 = []                   # empty list to fill with x coordinates of first column (smaller x)
    x_2 = []                   # empty list to fill with x coordinates of second column (medium x)
    x_3 = []                   # empty list to fill with x coordinates of third column (larger x)
    y_1 = []                   # empty list to fill with y coordinates of first row (smaller y)
    y_2 = []                   # empty list to fill with y coordinates of second row (medium y)
    y_3 = []                   # empty list to fill with y coordinates of third row (larger y)

    x_low = cont_x[0]          # smaller x coordinate of facelets countours is assigned to the variable x_low
    x_high = cont_x[-1]        # bigger x coordinate of facelets countours is assigned to the variable x_high
    y_low = cont_y[0]          # smaller y coordinate of facelets countours is assigned to the variable y_low
    y_high = cont_y[-1]        # bigger y coordinate of facelets countours is assigned to the variable y_high 
    
    dist = int(max(x_high-x_low, y_high-y_low)/4) # facelets separation distance from min/max detected contours centers
    
    x_1.append(x_low)          # smaller x coordinate of facelets countours is appended to the first column list
    y_1.append(y_low)          # smaller y coordinate of facelets countours is appended to the first row list
    
    for i in range(1, len(facelets)):                                  # iteration on detected facelets contours
        if x_low <= cont_x[i] and cont_x[i] < x_low + dist:            # case the contour center x coordinate is "small"
            x_1.append(cont_x[i])                                      # contour x coordinate is appended to the of first column list (small x)
        elif x_low + dist <= cont_x[i] and cont_x[i] < x_high - dist:  # case the contour center x coordinate is "medium"
            x_2.append(cont_x[i])                                      # contour x coordinate is appended to the of second column list (medium x)
        else:                                                          # case the contour center x coordinate is "large"
            x_3.append(cont_x[i])                                      # contour x coordinate is appended to the of third column list (large x)
        if y_low <= cont_y[i] and cont_y[i] < y_low + dist:            # case the contour center y coordinate is "small"
            y_1.append(cont_y[i])                                      # contour y coordinate is appended to the of first row list (small y)
        elif y_low + dist <= cont_y[i] and cont_y[i] < y_high - dist:  # case the contour center y coordinate is "medium"
            y_2.append(cont_y[i])                                      # contour y coordinate is appended to the of second row list (medium y)
        else:                                                          # case the contour center y coordinate is "large"
            y_3.append(cont_y[i])                                      # contour y coordinate is appended to the of third row list (large y)
    
    if len(x_1)==0 or len(x_2)==0 or len(x_3)==0 or len(y_1)==0 or len(y_2)==0 or len(y_3)==0: # case one or more of the six lists are empty
        return facelets                                                # function returns the already detected facelets
    
    else:                                      # case no one of the six lists is empty
        x1_avg = int(sum(x_1)/len(x_1))        # average x coordinate for the contours on first column (small x)
        x2_avg = int(sum(x_2)/len(x_2))        # average x coordinate for the contours on second column (medium x)
        x3_avg = int(sum(x_3)/len(x_3))        # average x coordinate for the contours on third column (large x)
        y1_avg = int(sum(y_1)/len(y_1))        # average y coordinate for the contours on first row (small y)
        y2_avg = int(sum(y_2)/len(y_2))        # average y coordinate for the contours on second row (medium y)
        y3_avg = int(sum(y_3)/len(y_3))        # average y coordinate for the contours on third row (large y)
    
    dist = int((x_high - x_low + y_high - y_low)/8)   # facelets separation distance from min/max detected contours centers
    detected = []                                     # list for the column row of the face grid
    for i in range(len(facelets)):                    # iteration over the detected facelets
        if cx[i]<x_low+dist:                          # case the facelet contour center x coordinate is on first grid column
            x=1                                       # 1 (as column 1) is assigned
        elif cx[i]>x_low+dist and cx[i]<x_high-dist:  # case the facelet contour center x coordinate is on second grid column
            x=2                                       # 2 (as column 2) is assigned
        elif cx[i]>x_high-dist:                       # case the facelet contour center x coordinate is on third grid column
            x=3                                       # 3 (as column 3) is assigned
        if cy[i]<y_low+dist:                          # case the facelet contour center x coordinate is on first grid row                          
            y=1                                       # 1 (as row 1) is assigned
        elif cy[i]>y_low+dist and cy[i]<y_high-dist:  # case the facelet contour center x coordinate is on second grid row
            y=2                                       # 2 (as row 2) is assigned
        elif cy[i]>y_high-dist:                       # case the facelet contour center x coordinate is on third grid row
            y=3                                       # 3 (as row 3) is assigned
        detected.append(facelet_grid_pos(x, y))       # list with facelet number is populated

    s = set(detected)                                        # list with detected facelets numbers is transformed to set
    missed = [x for x in (0,1,2,3,4,5,6,7,8) if x not in s]  # list with missed facelets numbers
    
    if (len(facelets)+len(missed)) != 9:      # case the total of detected and estimated facelets differ from 9
        return []                             # an empty list is returned

    est = []                                  # list for xy coordinates for the estimated facelet center locations
    for facelet in missed:                    # iteration over the missed facelets numbers
        if facelet == 0:                      # case the missed facelet is 0
            est.append((x1_avg, y1_avg))      # average xy coordinates for column 1 and row 1 are appended
        elif facelet == 1:                    # case the missed facelet is 1
            est.append((x2_avg, y1_avg))      # average xy coordinatees for column 2 and row 1 are appended
        elif facelet == 2:                    # case the missed facelet is 2
            est.append((x3_avg, y1_avg))      # average xy coordinats for column 3 and row 1 are appended
        elif facelet == 3:                    # case the missed facelet is 3
            est.append((x1_avg, y2_avg))      # average xy coordinates for column 1 and row 2 are appended
        elif facelet == 4:                    # case the missed facelet is 4
            est.append((x2_avg, y2_avg))      # average xy coordinates for column 2 and row 2 are appended
        elif facelet == 5:                    # case the missed facelet is 5
            est.append((x3_avg, y2_avg))      # average xy coordinates for column 3 and row 2 are appended
        elif facelet == 6:                    # case the missed facelet is 6
            est.append((x1_avg, y3_avg))      # average xy coordinates for column 1 and row 3 are appended
        elif facelet == 7:                    # case the missed facelet is 7
            est.append((x2_avg, y3_avg))      # average xy coordinates for column 2 and row 3 are appended
        elif facelet == 8:                    # case the missed facelet is 8
            est.append((x3_avg, y3_avg))      # average xy coordinates for column 3 and row 3 are appended
        else:                                 # case that shouldn't exist
            print("error on estimating the missed facelets")  # feedback is printed to the terminal
    
    if len(est) == 0:                  # case there aren't estimated facelets
        return facelets                # the funtion is interrupted
    
    est = rotate(est, (x0,y0), -angle) # center coordinates of the detected + estimated facelets are rotated back 

# contours generation on the estimated facelet(s)
    semi_side = int(0.5*dist/2)                         # half side dimension for the estimated contour square
    if len(missed)==1:                                  # case there is one missed facelet, to be estimated
        tl = [est[0] - semi_side, est[1] - semi_side]   # top left contour coordinate, calculated from the estimated contour center point
        tr = [est[0] + semi_side, est[1] - semi_side]   # top right contour coordinate, calculated from the estimated contour center point
        br = [est[0] + semi_side, est[1] + semi_side]   # bottom right contour coordinate, calculated from the estimated contour center point
        bl = [est[0] - semi_side, est[1] + semi_side]   # bottom left contour coordinate, calculated from the estimated contour center point
        pts=np.array([tl, tr, br, bl], dtype="int32")   # estimated contour coordinates
        contour_tmp = [pts]                             # list is made with the ordered outer contour
        tmp = {'area': med_a, 'cx': est[0], 'cy': est[1], 'contour': pts, 'cont_ordered':pts} # dict with relevant contour info
        facelets.append(tmp)      # estimated facelets relevant info are appended to the detected facelets list
    
    elif len(missed)>1:                                  # case there are more than one missed facelet, to be estimated
        for i, facelet in enumerate(missed):                      # iteration over the missed facelets
            tl = [est[i][0] - semi_side, est[i][1] - semi_side]   # top left contour coordinate, calculated from the estimated contour center point
            tr = [est[i][0] + semi_side, est[i][1] - semi_side]   # top right contour coordinate, calculated from the estimated contour center point
            br = [est[i][0] + semi_side, est[i][1] + semi_side]   # bottom right contour coordinate, calculated from the estimated contour center point
            bl = [est[i][0] - semi_side, est[i][1] + semi_side]   # bottom left contour coordinate, calculated from the estimated contour center point
            pts=np.array([tl, tr, br, bl], dtype="int32")         # estimated contour coordinates      
            contour_tmp = [pts]                                   # list is made with the ordered outer contour
            tmp = {'area': med_a, 'cx': est[i][0], 'cy': est[i][1], 'contour': pts, 'cont_ordered':pts} # dict with relevant contour info
            facelets.append(tmp)   # estimated facelets relevant info are appended to the detected facelets list
        
# check if there are (estimated) contours overlapping the others (too close to the center facelet)
    if len(facelets)==9:                       # 9 contours have cube compatible characteristics
        facelets = order_9points(facelets, new_center=[])  # contours are ordered from top left
        d_to_exclude = distance_deviation(facelets, check='below', delta=-0.15) # facelets to remove due too low inter-distance
        if len(d_to_exclude)>=1:               # case facelets to be removed, cause to less distance from median point
            d_to_exclude.sort(reverse=True)    # list order is reversed, making easy to remove
            for i in d_to_exclude:             # iteration on the list of contour index to be removed
                facelets.pop(i)                # contour too far are removed from list of potential facelets
    
    return facelets     # detected facelets combined with estimated facelets






def find_cube_facelet_squares(frame, offset, background_h):
    ''' Finds the 9 facelet squares of a cube face directly (rather than finding one big outer square and
    subdividing it). Ported from kkoomen/qbr (https://github.com/kkoomen/qbr), a well-tested, widely used
    webcam Rubik's cube scanner: find every square-ish contour of plausible facelet size, then find the one
    contour that has exactly 9 "neighbors" (including itself) sitting at the 8 expected grid positions around
    it - that's necessarily the center facelet, and its 9 neighbors are the 3x3 array. This confirms the real
    3x3 grid structure directly (instead of inferring a face boundary and guessing where the 9 cells are, or
    requiring every sticker pair to have a clean edge between them).
    Returns the 9 (x, y, w, h) facelet rects in reading order (top-left to bottom-right), or [] if not found.'''

    h, w = frame.shape[:2]
    roi = frame[background_h:h, offset:w]
    # Canny on plain grayscale misses edges between two colors that differ a lot in hue but happen to have
    # similar luminance (e.g. a cyan/yellow sticker against a mid-gray background) - run it on each BGR
    # channel separately and combine, so a color-only difference still produces an edge.
    canny = np.zeros(roi.shape[:2], dtype=np.uint8)
    for channel in cv2.split(roi):
        canny |= cv2.Canny(cv2.blur(channel, (3, 3)), 30, 60, 3)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    dilated = cv2.dilate(canny, kernel)
    contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, offset=(offset, background_h))

    expected_w = width/facelets_in_width   # rough facelet size guess, from the 'distance' slider
    min_w, max_w = expected_w*0.35, expected_w*3.0   # generous: the cube rarely matches this guess exactly

    candidates = []
    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.1*peri, True)
        if len(approx) != 4:
            continue
        area = cv2.contourArea(contour)
        x, y, cw, ch = cv2.boundingRect(approx)
        if ch == 0:
            continue
        ratio = cw/float(ch)
        if 0.8 <= ratio <= 1.2 and min_w <= cw <= max_w and area/(cw*ch) > 0.4:
            candidates.append((x, y, cw, ch))

    if debug:
        print(f'facelet candidates: {len(candidates)} (need >=8), expected_w={expected_w:.0f} [{min_w:.0f}-{max_w:.0f}]')
    if len(candidates) < 8:
        return []

    # Fit a 3x3 grid across all candidates by clustering their centers into 3 column x-positions and 3 row
    # y-positions - rather than requiring one specific candidate to literally be the center facelet and have
    # all 8 others sit at its expected neighbor offsets. A real cube's center facelet often carries a small
    # manufacturer logo, whose extra edges can fragment/erase just that one cell's contour even when the
    # other 8 are detected cleanly; row/column clustering finds the grid from whichever cells ARE present,
    # so a single missing slot (most commonly the center) is tolerated and its rect estimated from its row
    # and column neighbors, instead of requiring the true center to be one of the detected candidates.
    avg_cw = sum(c[2] for c in candidates)/len(candidates)
    avg_ch = sum(c[3] for c in candidates)/len(candidates)
    centers = [(x+cw/2, y+ch/2, cw, ch) for (x, y, cw, ch) in candidates]

    def cluster_1d(values, tol):
        values = sorted(values)
        groups = [[values[0]]]
        for v in values[1:]:
            if v - groups[-1][-1] <= tol:
                groups[-1].append(v)
            else:
                groups.append([v])
        return [sum(g)/len(g) for g in groups]

    col_centers = cluster_1d([c[0] for c in centers], avg_cw*0.6)
    row_centers = cluster_1d([c[1] for c in centers], avg_ch*0.6)
    if len(col_centers) != 3 or len(row_centers) != 3:
        return []       # candidates don't resolve into a clean 3x3 layout (wrong count of distinct rows/columns)

    grid = [[None]*3 for _ in range(3)]
    for (cxc, cyc, cw, ch) in centers:
        ci = min(range(3), key=lambda i: abs(col_centers[i]-cxc))
        ri = min(range(3), key=lambda i: abs(row_centers[i]-cyc))
        if grid[ri][ci] is not None:
            return []   # two candidates landed in the same grid slot: not a clean, confident 3x3 read this frame
        grid[ri][ci] = (cxc, cyc, cw, ch)

    missing = [(r, c) for r in range(3) for c in range(3) if grid[r][c] is None]
    if len(missing) > 1:
        return []
    for (r, c) in missing:                      # estimate the one missing cell from its row/column position
        grid[r][c] = (col_centers[c], row_centers[r], avg_cw, avg_ch)

    rects = []
    for r in range(3):
        for c in range(3):
            cxc, cyc, cw, ch = grid[r][c]
            rects.append((int(cxc-cw/2), int(cyc-ch/2), int(cw), int(ch)))
    return rects


def synthetic_facelets_from_rects(rects, inset=6):
    ''' Builds 9 synthetic square contours from the (x, y, w, h) facelet rects found by
    find_cube_facelet_squares(), each shrunk a few pixels in from its detected edges to stay clear of the
    gaps between stickers. Returned in the same contours/hierarchy format cv2.findContours produces, so
    they feed into the existing downstream pipeline (get_approx_contours, get_facelets, order_9points,
    read_color, ...) unchanged.'''

    contours, hierarchy_rows = [], []
    for (x, y, w, h) in rects:
        pts = np.array([[x+inset, y+inset], [x+w-inset, y+inset],
                         [x+w-inset, y+h-inset], [x+inset, y+h-inset]], dtype=np.int32)
        contours.append(pts.reshape((-1, 1, 2)))
        hierarchy_rows.append([-1, -1, -1, -1])   # no child contour, passes the get_facelets() hierarchy check

    hierarchy = np.array([hierarchy_rows])   # shape (1,9,4), matching cv2.findContours' own return shape
    return contours, hierarchy


def detect_in_fixed_zone(frame, w, h):
    ''' Draws a fixed target zone in the frame for the user to hold a cube face against, and looks for the 9
    facelet squares (see find_cube_facelet_squares()) ONLY within that zone - restricting both detection and
    color sampling to a small, known region, rather than hunting the whole frame where background clutter can
    get mistaken for the cube. The zone's border is white by default and turns GREEN as soon as a full 3x3 grid
    is confirmed inside it, giving direct visual feedback that the cube is positioned correctly. Used by both
    the normal scan (read_facelets()) and color calibration (calibrate_cube_colors()).
    background_h/offset keep the zone off the top/bottom text bands and the legend/sketch area.
    Returns the 9 (x, y, w, h) facelet rects in full-frame coordinates (or [] if not confirmed).'''

    zone_edge = max(20, int(width/facelets_in_width))
    zone_w = zone_edge*3
    zx0 = offset + (w - offset - zone_w)//2
    zy0 = background_h + (h - background_h - zone_w)//2

    # search a larger area than the drawn guide box (the box is a rough aiming target, not a strict boundary):
    # the user's cube rarely matches its exact size/position, so search margin*zone_w around it, clipped to frame
    margin = 1.6
    search_w = int(zone_w*margin)
    sx0 = max(offset, zx0-(search_w-zone_w)//2)
    sy0 = max(background_h, zy0-(search_w-zone_w)//2)
    sx1 = min(w, sx0+search_w)
    sy1 = min(h, sy0+search_w)

    zone = frame[sy0:sy1, sx0:sx1]
    rects = find_cube_facelet_squares(zone, 0, 0)                    # search the (larger) area around the guide box
    rects = [(x+sx0, y+sy0, rw, rh) for (x, y, rw, rh) in rects]      # back to full-frame coordinates

    zone_color = (0, 255, 0) if rects else (255, 255, 255)   # green once the cube is confirmed inside the zone
    cv2.rectangle(frame, (zx0, zy0), (zx0+zone_w, zy0+zone_w), zone_color, 3)
    for (x, y, rw, rh) in rects:
        cv2.rectangle(frame, (x, y), (x+rw, y+rh), (255, 0, 0), 1)   # each detected facelet, thin blue square

    return rects


def read_facelets(det_face_time, delay, proceed):
    ''' Shows the fixed target zone (see detect_in_fixed_zone()) and reads the 9 facelets once the cube is
    confirmed inside it (zone border turns green) and the capture delay/spacebar gate has passed.
    The camera view itself stays clean (no text drawn on it) - status/instructions live in the sidebar,
    composed by the caller (cubeAF()) via render_sidebar()/compose_and_show().'''

    wait_time = int(delay-(time.time() - det_face_time)) + 1

    rects = detect_in_fixed_zone(frame, w, h)

    if rects:
        # get_facelets() (downstream) only accepts contours within [min_area, max_area]; that range is normally
        # derived from the 'distance' slider guess of facelet size. Now that the 9 facelets have actually been
        # found and confirmed as a real 3x3 grid, size the acceptance window from their real, measured area
        # instead - directly tied to what's in frame rather than a separate manual guess.
        global min_area, max_area
        avg_area = sum(rw*rh for (_, _, rw, rh) in rects) / len(rects)
        min_area = avg_area*0.4
        max_area = avg_area*2.0

    if (wait_time > 0 and not proceed) or not rects:
        return [], None       # not yet time to capture, or the cube isn't confirmed in the zone: nothing to hand downstream

    return synthetic_facelets_from_rects(rects)




def read_facelets_fixed_grid(det_face_time, delay, proceed):
    ''' Alternative to read_facelets(): instead of hunting for square contours via edge detection,
    draws a fixed 3x3 grid that the user aligns the cube face to, and returns 9 synthetic square
    contours at the grid's exact position once it's time to capture. This feeds the very same
    downstream pipeline (get_facelets, order_9points, read_color, the accept/reject gate, etc.)
    used by read_facelets(), just from a different (non edge-detected) source.'''

    wait_time = int(delay-(time.time() - det_face_time)) + 1

    if wait_time > 0 and not proceed:
        cv2.putText(frame, str(f'Align side {sides[side]} to the grid, capturing in {wait_time} s'),
                    (10, 30), font, fontScale*1.2, fontColor, lineType)
        cv2.putText(frame, str('ESC to escape, spacebar to capture now'), (10, int(h-12)), font, fontScale*1.2, fontColor, lineType)
    else:
        cv2.putText(frame, str(f'Capturing side {sides[side]}'), (10, 30), font, fontScale*1.2, fontColor, lineType)
        cv2.putText(frame, str('ESC to escape'), (10, int(h-12)), font, fontScale*1.2, fontColor, lineType)

    cube_centers_color_ref(frame)           # same reference legend as the auto-detect mode
    plot_colors(BGR_mean, deco_edge, frame, font, fontScale, lineType)

    grid_edge = max(20, int(width/facelets_in_width))    # per-cell size, from the existing 'distance' slider setting
    grid_w = grid_edge*3
    gx0 = offset + (w - offset - grid_w)//2               # horizontally centered in the usable (non-legend) area
    gy0 = background_h + (h - background_h - grid_w)//2   # vertically centered in the usable (non-banner) area

    for row in range(3):                                  # draws the 9-cell grid outline every frame, as alignment guide
        for col in range(3):
            x = gx0 + col*grid_edge
            y = gy0 + row*grid_edge
            cv2.rectangle(frame, (x, y), (x+grid_edge, y+grid_edge), (255, 255, 255), 2)

    if wait_time > 0 and not proceed:       # not yet time to capture: grid is drawn, nothing returned yet
        return [], None

    contours = []                           # 9 synthetic square contours, at the grid's exact pixel position
    hierarchy_rows = []
    for row in range(3):
        for col in range(3):
            cx = gx0 + col*grid_edge + grid_edge//2
            cy = gy0 + row*grid_edge + grid_edge//2
            half = grid_edge//2 - 2         # slightly inset from the drawn outline
            pts = np.array([[cx-half, cy-half], [cx+half, cy-half], [cx+half, cy+half], [cx-half, cy+half]], dtype=np.int32)
            contours.append(pts.reshape((-1, 1, 2)))
            hierarchy_rows.append([-1, -1, -1, -1])   # no child contour, passes the get_facelets() hierarchy check

    hierarchy = np.array([hierarchy_rows])  # shape (1,9,4), matching cv2.findContours' own return shape

    return contours, hierarchy







def get_approx_contours(component):
    ''' Function that simplifies contours (from: https://docs.opencv.org/4.5.3/dd/d49/tutorial_py_contour_features.html)
    Argument is a contour, having at least 4 vertex (contours with less than 4 vertex were previously filtered out)
    Returns approximated contours, having 4 vertex.''' 
    
    contour = component[0]
    hierarchy = component[1][2]
    peri = cv2.arcLength(contour, True)
    contour_convex = cv2.convexHull(contour, False)
    contour = cv2.approxPolyDP(contour_convex, 0.1*peri, True)
    
    return contour, hierarchy, len(contour)







def get_facelets(facelets, contour, hierarchy):
    ''' Contours are analyzed in order to detect the cube's facelets
    Argument are simplified contours
    Returns contours having square characteristics
    
    [parameter to be square like: Area within limits, limited egde lenght deviation, limited diagonals lenght deviation
    (to prevent rhonbus), limited inclination, limited area variation between the 9 facelets,
    limited distante between the 9 facelets centers].''' 
    

    global min_area, max_area
    
    # on April 2022 changed (square_ratio, rhombus_ratio, max_inclination) to very permissive values
    square_ratio=0.4     # considered like a square when [(max_side-min_side)/avg edge  < square_ratio], where 0 = perfect square
    rhombus_ratio=0.6    # considered like a square when [max_diagonal/min_diagonal > rhombus_ratio], where 1 = perfect square
    if estimate_fclts:
        max_inclination=20          # max inclination limit for the 1st facelet vs horizon in degrees
    else:
        max_inclination=50          # max inclination limit for the 1st facelet vs horizon in degrees
    area = cv2.contourArea(contour) # area of each passed contour is retrieved
    
    if hierarchy>=0:                # when contour is an outermost (it has inner contour/contours) 
        area=0                      # contour area is forced to zero to skip that contour
    
    if min_area < area < max_area:                       # filter out too small and too large contours (areas)                                              # that contour isn't  
        contour_squeeze = np.squeeze(contour)                     # flattens out the list of list used by contours
        edges_delta, axes_ratio = square_check(contour_squeeze)   # sanity check on square and ronbhus shapes
        if edges_delta < square_ratio and axes_ratio > rhombus_ratio:      # check if the contour looks like a square
            out_cont_ord, in_cont_ord = order_4points(contour_squeeze)      # vertex of each contour are ordered CW from top left
            inclination = inclination_check(out_cont_ord)         # facelet inclination is measured
            if abs(inclination)<max_inclination:                  # filter out when the (1st) facelets (cube faces) are too inclined
                contour_tmp = [out_cont_ord]                      # list is made with the ordered contour
                cv2.drawContours(frame, contour_tmp, -1, (0, 0, 0), 1)  # a balck polyline is drawn on the contour (1 px thickness)
                # cv2.circle(frame, (contour_ordered[0][0],contour_ordered[0][1]), 5, (0, 0, 0), -1)    # a circle is drawn on the 1st vertex (top left)
                M = cv2.moments(contour)                    # the shape moment (center) of the contour is retrieved
                if M['m00']:                                # compute the center of the contour   
                    cX = int(M['m10'] / M['m00'])           # X value for the contour center
                    cY = int(M['m01'] / M['m00'])           # Y value for the contour center
                
                tmp = {'area': area, 'cx': cX, 'cy': cY, 'contour': contour, 'cont_ordered':in_cont_ord}  # dict with relevant contour info
                
                facelets.append(tmp)                        # list with the dictionary of the potential facelets contrours
                
                N_min = 7
                if len(facelets) >= N_min:                  # when potential contours are at leat equal to N_min
                    a_to_exclude = area_deviation(facelets) # function that analyzes facelets area, and list those eventually with high dev from median
                    if len(a_to_exclude)>=1:                # case when there are facelets to be excluded, due to too different area from median one
                        a_to_exclude.sort(reverse=True)     # list order is reversed, making easy to remove
                        for i in a_to_exclude:              # contour deviating too much on area are removed from list of potential facelets
                            facelets.pop(i)                 # contour deviating too much on area are removed from list of potential facelets                      

                if estimate_fclts and len(facelets) >= N_min:  # case estimation is enabled and still least N_min facelets
                    cube_incl = cube_inclination(facelets)     # cube inclination is retrieved
                    facelets = estimate_facelets(facelets, cube_incl)   # function to estimate remaining facelets     
    
    return facelets   # list of potential facelet's contour is returned







def area_deviation(data):
    ''' Checks whether the areas of 9 facelets are within a certain deviation from the median one
    Aim of this function is to force the user to present the cube with face somehow parallel to the camera
    This function is called when there are a certain amount of potential facelet contours
    Argument is a list of dictionary, wherein the area is one of the dict values
    Returns a list of facelets (index) to be removed from the potential facelets, having area deviating
    too much from the median one.'''
    
    to_exclude = []             # list of the contours index to be removed, due to excess of their area deviation

    delta=0.75                  # 75% of area deviation from the median is set as threshold
    
    area_list=[]                # list to store the contour areas
    for i in range(len(data)):
        area_list.append(data[i]['area'])                  # all the contour areas are listed
    area_median = statistics.median(area_list)             # median area values
    for i in range(len(area_list)):          
        delta_area=(area_list[i]-area_median)/area_median  # area deviation from the median
        if delta_area > delta:                             # filter on area deviating more than threshold
            to_exclude.append(i)                           # list of the contours to exclude is populated
    
    return to_exclude            # returns list of contours to be removed







def distance_deviation(data, check='above', delta=0.25):
    ''' Checks whether the distances between the 9 contours (centers) are within a certain deviation from the median
    In other words, a sanity check if all the 9 facelet are compatible with a 3x3 square array shape
    Aim of this funtion is to exclude contours:
      1) generated outside the cube, due to square-like shapes the webcam detects at the user background, face, cloths.
      2) estimated yet overlapping some of the detected contours (check = 'below' in this case).
    The approach checks indipendently the 6 horizontal distances from the contours center, from the 6 vertical
    Considering the cube can have a certain tilting angle (inclination), Pitagora theorem is used.
    Function return a list with the index of the countours to be removed from the list of potential facelets.'''
    
    d_to_exclude = []        # list of the contour index to be removed, due to excess of distance deviation
    
    distance_list_h = []     # list of the horizontal distance, of each contour from the median one
    distance_list_v = []     # list of the vertical distance, of each contour from the median one

    points_h=[1,2,4,5,7,8]  # points to consider for the distance along the 'horizontal' array 
    for i in points_h:
        j=i-1
        # horizontal distance between the contours centers
        dist=math.sqrt((data[i]['cx']-data[j]['cx'])**2 + (data[i]['cy']-data[j]['cy'])**2)
        distance_list_h.append(dist)  # list with horizontal distance between the contours centers

    points_v=[3,4,5,6,7,8]  # points to consider for the distance along the 'vertical' array 
    for i in points_v:
        k=i-3
        # vertical distance between the contours centers
        dist=math.sqrt((data[i]['cx']-data[k]['cx'])**2 + (data[i]['cy']-data[k]['cy'])**2)
        distance_list_v.append(dist)
    
    dist_median_h = statistics.median(distance_list_h)         # median value for horiz distances
    for i in range(len(distance_list_h)):
        delta_dist_h=(distance_list_h[i]-dist_median_h)/dist_median_h # deviation in horiz distance
        if check == 'above':
            if delta_dist_h > delta:                           # filter if horiz deviation > threshold
                d_to_exclude.append(i) # list with contours indexto exlude, due excess on horiz deviation from median
        elif check == 'below':
            if delta_dist_h < delta:                           # filter if horiz deviation > threshold
                d_to_exclude.append(i) # list with contours indexto exlude, due too small horiz deviation from median

    dist_median_v = statistics.median(distance_list_v)         # median value for vert distances
    for i in range(len(distance_list_v)):
        delta_dist_v=(distance_list_v[i]-dist_median_v)/dist_median_v # deviation in vert distance
        if check == 'above':
            if delta_dist_v > delta and i not in d_to_exclude: # filter if horiz deviation > threshold
                d_to_exclude.append(i)  # list with contours indexto exlude, due excess on vert deviation from median
        elif check == 'below':
            if delta_dist_v < delta and i not in d_to_exclude: # filter if horiz deviation > threshold
                d_to_exclude.append(i)  # list with contours indexto exlude, due too small vert deviation from median
        
    return d_to_exclude







def order_4points(pts):
    ''' Based on: https://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
    Modified on 20220513 to don't use Scipy library (importing 'scipy.spatial distance' isn't anymore needed)
    Orders the 4 vertex of (simplified) contours, so that the first one is top left (CW order)
    Argument is a contour
    Returns a contour with ordered coordinates
    
    0  1
    2  3
    '''
        
    xSorted = pts[np.argsort(pts[:, 0]), :]   # sort the points based on their x-coordinates
    leftMost = xSorted[:2, :]                 # grab the left-most point from the sorted x-coodinate points
    rightMost = xSorted[2:, :]                # grab the right-most point from the sorted x-coodinate points
    
    # sort the left-most according to their y-coordinates, to grab the top-left and bottom-left points, respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost    
    
    #########    used until 20220513, based on scipy library   ###################################################
    # Euclidean distance from top-left and right-most points: the point with largest distance will be bottom-right
    # D = dist.cdist(tl[np.newaxis], rightMost, 'euclidean')[0]
    ##############################################################################################################
    
    #########   starting from 20220513, based on Numpy library (no need for Scipy library)   #####################
    # Euclidean distance from top-left and right-most points: the point with largest distance will be bottom-right
    tl = tl.reshape(1, 2)                                    # reshaping top-left coordinate array
    D = tl[:, np.newaxis, :] - rightMost[np.newaxis, :, :]   # matrix difference, by broadcasting
    D = np.linalg.norm(D, axis=-1)[0]                        # matrix distance by using numpy (instead of scipy)
    tl = tl.reshape(2,)                                      # reshaping top-left coordinate array back
    ##############################################################################################################
    
    # sort the right-most coordinates according to their distance from top-left coordinate
    (br, tr) = rightMost[np.argsort(D)[::-1], :]
        
    inner_pts=np.array([tl, tr, br, bl], dtype='int32')

    # few pixel shift toward the outside, to draw the contour on the outer_pts,
    # without affecting the inner_pts color detection
    gap=3
    tl[0]=tl[0]-gap
    tl[1]=tl[1]-gap
    tr[0]=tr[0]+gap
    tr[1]=tr[1]-gap
    br[0]=br[0]+gap
    br[1]=br[1]+gap
    bl[0]=bl[0]-gap
    bl[1]=bl[1]+gap
    outer_pts=np.array([tl, tr, br, bl], dtype='int32')  # ordered coordinates of the detected contour, sligtly enlarged
    
    return outer_pts, inner_pts  







def order_9points(data, new_center):
    ''' Based on: https://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
    Modified on 20220513 to don't use Scipy library (importing 'scipy.spatial distance' isn't anymore needed)
    Orders the 9 countorus centers, with first one on top left, and order as per below sketch:
    
    0  1  2
    3  4  5
    6  7  8
    '''
    pts=np.zeros([9,2], dtype=int)
    for i in range(len(data)):
        pts[i]=[data[i]['cx'], data[i]['cy']]
        
    xSorted = pts[np.argsort(pts[:, 0]), :]        # sort all the points based on their x-coordinates
    leftMost = xSorted[:3, :]                      # grab the left-most 3 points from the sorted x-coodinate points
    rightMost = xSorted[6:, :]                     # grab the right-most 3 points from the sorted x-coodinate points
    mid = xSorted[3:6, :]                          # remaining 3 points in the x middle
    ySortedMid = mid[np.argsort(mid[:, 1]), :]     # sorts the 3 points in the x middle by the y coordinate
    (tm, mm, bm) = ySortedMid                      # top-middle, middle-middle, bottom-midle points
    
    # sort the 3 left-most points according to their y-coordinates, to grab the top/mid/bottom one respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, ml, bl) = leftMost
    
    #########    used until 20220513, based on scipy library   ###################################################
    # Euclidean distance from top-left and right-most points: the point with largest distance will be bottom-right
    # D = dist.cdist(tl[np.newaxis], rightMost, 'euclidean')[0]
    ##############################################################################################################
    
    #########   starting from 20220513, based on Numpy library (no need for Scipy library)   #####################
    # Euclidean distance from top-left and right-most points: the point with largest distance will be bottom-right
    tl = tl.reshape(1, 2)                                    # reshaping top-left coordinate array
    D = tl[:, np.newaxis, :] - rightMost[np.newaxis, :, :]   # matrix difference, by broadcasting
    D = np.linalg.norm(D, axis=-1)[0]                        # matrix distance by using numpy (instead of scipy)
    tl = tl.reshape(2,)                                      # reshaping top-left coordinate array back
    ##############################################################################################################
    
    # sort the right-most according to their distance from top-left coordinate
    (br, mr, tr) = rightMost[np.argsort(D)[::-1], :]
    
    # ordered coordinates (centers of 9 facelets)   
    ordered_points = np.array([tl, tm, tr, ml, mm, mr, bl, bm, br], dtype='int32')
    
    for coordinates in ordered_points:
        for i in range(len(data)):
            if data[i]['cx'] == coordinates[0] and data[i]['cy'] == coordinates[1]:        
                new_center.append(data[i])     # new_center is a new list with data ordered by xy coordinates
                data.pop(i)                    # used data-element is removed to speed up next iterations
                break                          # inner for loop can be break once the if if found and data appended
    return new_center







def cube_sketch_coordinates(x_start, y_start, edge):
    ''' Generates a list and a dict with the top-left coordinates of each facelet, as per the Kociemba order.
    These coordinates are later used to draw a cube sketch
    The cube sketch (overall a single rectangle with all the facelets in it) starts at x_start, y_start
    Each facelet on the sketch has a square side dimention = edge, defined at start_up() function.'''
    
    d = edge             # edge length
    square_start_pt=[]   # lits of all the top-left vertex coordinate for the 54 facelets
    
    starts={0:(x_start+3*d, y_start), 1:(x_start+6*d, y_start+3*d), 2:(x_start+3*d, y_start+3*d), 3:(x_start+3*d, y_start+6*d),
            4:(x_start, y_start+3*d), 5:(x_start+9*d, y_start+3*d)}
    
    for value in starts.values():
        x_start=value[0]
        y_start=value[1]
        y = y_start
        for i in range(3):
            x = x_start
            for j in range(3):
                square_start_pt.append([x, y])
                x = x+d
                if j == 2: y = y+d
    square_dict = {k:tuple(square_start_pt[k]) for k in range(len(square_start_pt))}
    
    return square_start_pt, square_dict     #square_start_center, dictiony cube<->1st vertex, cube edge







def inner_square_points(square_dict,i,edge):
    ''' Generates the 4 square vertex coordinates, to later color the cube sketch
    These square vertex coordinates are shifted by 1 pixel to the inner side, based on the top-left square vertex (dict of top left
    vertex of the 54 facelets); The dict index defines the facelet number, and the edge is the square side lenght
    The returned array defines the 4 points coordinate, of the area within each of the 54 facelets.'''
    x=square_dict[i][0]+1
    y=square_dict[i][1]+1
    d=edge-2
    return np.array([(x,y),(x+d,y),(x+d,y+d),(x,y+d)])







def cube_centers_color_ref(frame):
    ''' It suggests the faces (center) color as guidance on screen, to present the right cube face to the camera.
    This function fills the center facelets with refence color.'''

    global deco_edge
    edge = deco_edge   # decorative sketches use deco_edge, scaled to fit the actual frame height (see start_up())

    x_start=int(edge/3)
    y_start=int(edge*6)
    square_start_pt, square_dict = cube_sketch_coordinates(x_start,y_start, edge)
    d = edge           # edge lebght for each facelet reppresentation
    m = int(edge/6)    # m=margin around the cube reppresentation

    cv2.rectangle(frame, (x_start-m, y_start-2*edge), (x_start+13*d, int(y_start+9.2*d)), (230,230,230), -1) #gray background
    cv2.putText(frame, 'Reference:', (x_start, y_start-int(0.5*d)), font, fontScale*0.85,(0,0,0),lineType)

    for point in square_start_pt:
        cv2.rectangle(frame, tuple(point), (point[0]+edge, point[1]+edge), (0, 0, 0), 1)

    # color definition in BGR per each of the 6 centers
    center_facelets={4:(255,255,255), 13:(0,0,204), 22:(0,132,0), 31:(0,245,245), 40:(0,128,255), 49:(204,0,0)}
    center_facelet_colors= {4:'white', 13:'red', 22:'green', 31:'yellow', 40:'orange',  49:'blu'}
    fcs_ltrs= {4:'U',13:'R',22:'F',31:'D',40:'L',49:'B'}


    for key, bgr_color in center_facelets.items():
        points=inner_square_points(square_dict,key,edge)  # coordinates for the center facelet on the iterator
        color= bgr_color                                  # BGR color is returned
        cv2.fillPoly(frame, pts = [points], color=color)  # facelet is fille with the BGR color for tha center
        
        x=int(square_start_pt[key][0]-0.7*d)
        y=int(square_start_pt[key][1]+1.7*d)
        cv2.putText(frame, fcs_ltrs[key], (x,y), font, fontScale*1.8,(0,0,0),lineType)







def plot_colors(BGR_mean, edge, frame, font, fontScale, lineType):
    ''' This function plots the detected color of each facelet on the cube sketch.
    This function is called each time a cube face is detected
    The color used on each facelet is the mean BGR calculated on that facelet
    In essence this is a decoration to compare interpreted vs detected colors.'''
    
    x_start=int(edge/3)    # top lef corner of the rectangle where all the cube's faces are plot
    y_start=int(edge*17.5)   # top lef corner of the rectangle where all the cube's faces are plot
    
    square_start_pt, square_dict = cube_sketch_coordinates(x_start, y_start, edge)    # draw the cube frame in blac
    d = edge           # edge lebght for each facelet reppresentation
    m = int(edge/6)    # m=margin around the cube reppresentation  
    
    cv2.rectangle(frame, (x_start-m, y_start-2*edge), (x_start+13*d, int(y_start+9.2*d)), (230,230,230), -1) #gray background
    cv2.putText(frame, 'Detected:', (x_start, y_start-int(0.5*d)), font, fontScale*0.85,(0,0,0),lineType)
    
    for point in square_start_pt:
        cv2.rectangle(frame, tuple(point), (point[0]+edge, point[1]+edge), (0, 0, 0), 1)

    # iteration on the global dict with stored the avg BGR value of each facelet detected so far
    for i in range(len(BGR_mean)):
        # the raw captured BGR is classified to the nearest of the 6 known cube colors, and that reference's
        # own flat color is what gets displayed - never the raw, lighting-tinted color the camera actually saw
        _, (B, G, R) = classify_bgr_to_6(BGR_mean[i])
        points=inner_square_points(square_dict,i,edge)      # coordinates for 'i' facelet at the sketch
        cv2.fillPoly(frame, pts = [points], color=(B,G,R))  # 'i' facelet is colored with its classified color







KOCIEMBA_POSITION = {'U':0, 'R':1, 'F':2, 'D':3, 'L':4, 'B':5}  # each letter's 9-facelet block index in kociemba's fixed U,R,F,D,L,B order
CENTER_COLOR = {4:'white', 13:'red', 22:'green', 31:'yellow', 40:'orange', 49:'blue'}  # each face's center facelet's color - fixed by definition, never classified


def reorder_to_kociemba_position(flat_list, scan_order_letters):
    ''' The 54-item flat_list (BGR or any per-facelet value) was accumulated in SCAN order - whatever order
    'sides' presents the 6 faces to the user in - but the color-interpretation functions below assume the
    fixed kociemba position order U,R,F,D,L,B (they read specific fixed indices directly, e.g. facelet 4 is
    always assumed to be U's center, 13 is R's center, ...). This reorders the 9-facelet blocks from scan
    order into that fixed position order, so the scan can present the 6 faces in any order.'''

    reordered = [None]*54
    for scan_idx, letter in enumerate(scan_order_letters):
        kpos = KOCIEMBA_POSITION[letter]
        reordered[kpos*9:(kpos+1)*9] = flat_list[scan_idx*9:(scan_idx+1)*9]
    return reordered


def cube_colors_interpreted(BGR_detected):
    ''' This function is used to decide wich color belongs to which facelet (cube's side) and related facelet position
    From the mean BGR color, detected per each facelet, the euclidean distance is calculated toward the 6 reference colors (centers).
    The basic principle is to measure the color distance from the cube's center facelets.
    
    1st addition:
    Due to camera vignetting and light conditions, some color interpretation issues were sometimes happening
    (i.e. swapped red and orange).
    To solve this problem the reference colors are averaged with the just interpreted facelets (dinamic color references)
    In this approach the facelets need to be ordered by increasing distance from the reference; This allows to initially choose the more
    certain facelets, and use them to adapt the references.
    After completing this process, for all the facelets, the interpreted colors have to be ordered back to kociemba order.
    This improved version has proved to solve the color interpetation errors from the simpler original function; The verification
    has been done on some BGR_detected cube status logged with errors on interpreted colors
    
    2nd addition:
    1st addition brougth to a success rate of 97%, rather good yet not fully sattisfactory.
    With very bright light conditions the Red/Orange were sometimes swapped on color assigment.
    On this latest improvement the HVS color space is added, and used for facelet's color interpretation, 
    in case the BGR color distance doesn't provide coherent result.'''

    # Step1: dict with BGR_detected and facelet's position as key
    #        dict with HSV (detected color) and facelet's position as key
    BGR_detected_dict={} 
    HSV_detected={}
    for i in range(len(BGR_detected)):
        BGR=BGR_detected[i]
        BGR_detected_dict[i]=BGR
        B,G,R=BGR
        BGR_mean = np.array([[[B,G,R]]], dtype=np.uint8)
        hsv = cv2.cvtColor( BGR_mean, cv2.COLOR_BGR2HSV)
        H=hsv[0][0][0]
        S=hsv[0][0][1]
        V=hsv[0][0][2]
        HSV_detected[i]=(H,S,V)

    if debug:                                      # case the debug variable is set True
        print(f'\nBGR_detected: {BGR_detected}')   # feedback is printed to the terminal
        print(f'\nHSV: {HSV_detected}')            # feedback is printed to the terminal

    # Step2: initial reference colors. A saved calibration (see calibrate_cube_colors(), measured directly
    # off the user's own cube under their own lighting) is preferred when available, since it's far more
    # reliable than trusting whichever 6 center facelets this particular scan happened to detect.
    if len(calibrated_colors) == 6:
        cube_ref_colors = dict(calibrated_colors)
    else:
        cube_ref_colors = {'white':BGR_detected[4], 'red':BGR_detected[13], 'green':BGR_detected[22],
                           'yellow':BGR_detected[31], 'orange':BGR_detected[40], 'blue':BGR_detected[49]}

    # Step3: dictionary with the color distances from the (initial) references
    color_distance={}                                             # empty dict to store all the color distances for all the facelets
    cube_ref_colors_lab={}                                        # empty dictionary to store color refences in Lab color space
    for color, BGR in cube_ref_colors.items():
        B,G,R = BGR                                               # BGR values are unpact from the dict
        cube_ref_colors_lab[color]=tuple(rgb2lab([R,G,B]))        # BGR conversion to lab color space and dict feeding
            
    for facelet, color_measured in BGR_detected_dict.items():         
        B,G,R = color_measured
        lab_meas = rgb2lab([R,G,B])                               # conversion to lab color space (due CIEDE2000 function)
        distance=[]                                               # list with the distance from the 6 references, for each facelet
        for color, lab_ref in cube_ref_colors_lab.items():        # iteration over the 6 reference colors
            distance.append(CIEDE2000(tuple(lab_meas), lab_ref))  # Euclidean distance toward the 6 reference colors
        color_distance[facelet]=distance                          
    
    
    # Step4: Ordering the color distance (the min value per each facelet) by increasing values
    color_distance_copy=color_distance.copy()    # a dict copy is made, to drop items while the analysis progresses
    color_distance_ordered={}                   # empty dictiony to store the (min) color distance by increasing values
    for i in range(len(color_distance_copy)):
        index = color_distance_copy.keys()
        key_min_dist = min(color_distance_copy,key=lambda key:min(color_distance_copy[key]))
        color_distance_ordered[key_min_dist] = color_distance_copy[key_min_dist]
        color_distance_copy.pop(key_min_dist)
    
    
    # Step5: List with facelets position ordered according to the color distance increasing order
    # this is needed to come back later to original kociemba facelets order
    key_ordered_by_color_distance = [x for x in color_distance_ordered.keys()]


    # Step6: Ordering the facelets BGR color values according to color distance from the reference colors
    BGR_ordered={}
    for key in color_distance_ordered.keys():
        BGR_ordered[key]=BGR_detected[key]


    # Step7: Color interpretation
    cube_status_by_color_distance={}          # dict to store the cube status reppresentation wih the interpreted colors
    distance={}                               # dict to store the color distance during each facelet check
#     distance_value=[]                       # list to store the color distance for the selectec color/facelet association
    
    i=0
    for value in BGR_ordered.values():            # iteration on the facelet's BGR values ordered by increasing color distance from ref
        B,G,R =value
        lab_meas = rgb2lab([R,G,B])                                         # conversion to lab color space (due CIEDE2000 function)
        for color, lab_ref in cube_ref_colors_lab.items():                  # iteration over the 6 reference colors
            distance[color]=CIEDE2000(tuple(lab_meas), lab_ref)             # Euclidean distance toward the 6 reference colors
        color = min(distance, key=distance.get)                             # chosem color is the one with min distance from reference
  
        cube_status_by_color_distance[i]=color                              # dict of cube status wih the interpreted colors  
#         distance_value.append(distance[min(distance, key=distance.get)])  # list with the color distance of the chosen facelet's color
        distance.clear()                                                    # distance dict is cleared for the next facelet
        
        B_avg = math.sqrt((B**2+ (cube_ref_colors[color][0])**2)/2)   # average Red color id made from the chosen color and previous reference
        G_avg = math.sqrt((G**2+ (cube_ref_colors[color][1])**2)/2)   # average Green color id made from the chosen color and previous reference
        R_avg = math.sqrt((R**2+ (cube_ref_colors[color][2])**2)/2)   # average Blue color id made from the chosen color and previous reference

        cube_ref_colors[color]=(B_avg, G_avg, R_avg)                    # Color reference dict is updated with the new BGR averaged color
        cube_ref_colors_lab[color]=tuple(rgb2lab([R_avg,G_avg,B_avg]))  # Lab color space reference dict is updated with the new color reference 
        i+=1
    
    
    # Step8: Cube detection status is generated
    cube_status={}
    for i in range(54):
        index = key_ordered_by_color_distance.index(i)
        cube_status[i]=cube_status_by_color_distance[index]

    # A center facelet never moves relative to its own face - on a real cube, U's center IS white, R's IS red,
    # and so on, always, by definition. This isn't a guess to classify like the other 8 facelets on that face;
    # it's a structural fact of the puzzle. Force it, so a bad calibration/lighting reading (which can pull a
    # center's distance-based classification toward the wrong color, e.g. a white center reading as blue) can
    # never produce a self-contradictory cube status where a face's own center isn't that face's color.
    for pos, color in CENTER_COLOR.items():
        cube_status[pos] = color

    # Step9: Cube color sequence for a nicer decoration on interpreted colors (using the HSV color space)
    VS_value={}                            # dict to store the V-S (Value-Saturation) value of all facelets
    Hue={}                                 # dict to store the Hue value of all facelets
    
    i=0
    for H,S,V in HSV_detected.values():           # Hue, Saturation and Value are retrieved from the HSV dict
        VS_value[i]=int(V)-int(S)                 # V-S (value-Saturation) value for all the facelet, populates the related dict
        Hue[i]=int(H)                             # Hue, for all the facelets, populates the related dict
        i+=1
    
    #step 10: function to get the color (sides) order, list of colored center's facelets and white center facelet  
    cube_color_sequence, HSV_analysis = retrieve_cube_color_order(VS_value, Hue)
    if debug:                                     # case the debug variable is set True
        print(f'\nCube_color_sequence: {cube_color_sequence}') # feedback is printed to the terminal
    
    return cube_status, HSV_detected, cube_color_sequence







def retrieve_cube_color_order(VS_value,Hue):
    ''' Determines the cube colors order, meaning the sides order while manually presenting the cube in front of the laptop.
    The function returns a list with the color order, as per kociemba sequence.
    The function also returns a boolean if the HSV analysis is so far coherent.'''
    
    HSV_analysis=True                                # flag used to validate the analysis
    
    centers=[4, 13, 22, 31, 40, 49]                  # facelets of the cube's sides centers 
    cube_color_seq=[4, 13, 22, 31, 40, 49]           # list of center facelets
    cube_color_sequence=['','','','','','']          # list to store the faces colors order
    
    
    if debug:                                        # case the debug variable is set True
        print(f'\nHue centers: {Hue[centers[0]]}, {Hue[centers[1]]} , {Hue[centers[2]]}, {Hue[centers[3]]}, {Hue[centers[4]]}, {Hue[centers[5]]}')
    
    VS_centers=[VS_value[facelet] for facelet in centers]  # V-S value measured on the cube side center under analysis
    Hcenters=[Hue[facelet] for facelet in centers]        # Hue value measured on the cube side center under analysis
    
    
    # white and yellow facelet face (center side facelet number)
    white_center=centers[VS_centers.index(max(VS_centers))]   # white center facelet (max V-S distinguishes the white)
    if white_center<27: 
        yellow_center=white_center+27                # yellow center facelet (yellow is at opposite side of white)
    else:
        yellow_center=white_center-27                # yellow is at opposite side of white
    
    try:
        centers.remove(white_center)                 # white facelet is removed from the list of center's facelets
    except:
        HSV_analysis=False
        if debug:                                    # case the variable debug is set True 
            print(f'\nIssue with the white_center')  # feedback is printed to the terminal
    
    try:
        centers.remove(yellow_center)                # yellow facelet is removed from the list of center's facelets
    except:
        HSV_analysis=False
        if debug:                                    # case the variable debug is set True
            print(f'\nIssue with the yellow_center') # feedback is printed to the terminal
       
    
    # searching for the red and orange cube's side
    for facelet in centers:                   # iteration on the 4 cube's remaining sides centers
        if facelet<=27:        
            opp_facelet=facelet+27            # opposite side center facelet
        elif facelet>27:
            opp_facelet=facelet-27            # opposite side center facelet
        Hc=Hue[facelet]                       # Hue value measured on the cube side center under analysis
        H_opp=Hue[opp_facelet]                # Hue value measured on the cube opposite side center

        if (Hc>150 and H_opp<30) or (Hc<30 and H_opp<30 and Hc<H_opp) or (Hc>160 and H_opp>170 and Hc<H_opp):   # Hue filter for red vs orange
            red_center=facelet                # red center facelet
        elif (Hc<30 and H_opp>150) or (Hc<30 and Hc>H_opp) or (Hc>170 and Hc>H_opp):              # Hue filter for orange vs red
            orange_center=facelet             # orange center facelet
    
    try:
        centers.remove(red_center)            # red facelet is removed from the list of center's facelets
    except:
        HSV_analysis=False
        if debug:                             # feedback is printed to the terminal
            print(f'\nIssue with the red_center')  # feedback is printed to the terminal
    
    try:
        centers.remove(orange_center)         # orange facelet is removed from the list of center's facelets
    except:
        HSV_analysis=False
        if debug:                             # feedback is printed to the terminal
            print(f'\nIssue with the orange_center')  # feedback is printed to the terminal
    
    if HSV_analysis==True:
        # last two colors are blue and green, wherein blue has the highest Hue
        Hc=[Hue[facelet] for facelet in centers]  # Hue value for the last two center's facelets
        blue_center=centers[Hc.index(max(Hc))]    # blue center facelet (higher Hue than green)
        green_center=centers[Hc.index(min(Hc))]   # green center facelet (lower Hue than blue)
        
        for i, value in enumerate(cube_color_seq):
            if value == white_center:
                cube_color_sequence[i] = 'white'
            elif value == yellow_center:
                cube_color_sequence[i] = 'yellow'
            elif value == red_center:
                cube_color_sequence[i] = 'red'
            elif value == orange_center:
                cube_color_sequence[i] = 'orange'
            elif value == blue_center:
                cube_color_sequence[i] = 'blue'
            elif value == green_center:
                cube_color_sequence[i] = 'green'
    else:
        if debug:                                  # feedback is printed to the terminal
            print('\nNot found 6 different colors at center facelets') # feedback is printed to the terminal
        
    return cube_color_sequence, HSV_analysis







def cube_colors_interpreted_HSV(BGR_detected, HSV_detected):
    ''' In case the color interpretation, based on BGR color distance, doesn't give coherent cube status, this function is called.
    Under bright/sunny light condition the colors, chosen via color distance analysys, are sometimes wrongly interpreted
    (=not coherent cube status);
    This function make use on the HSV color space, converted from the avg BGR read on facelets.
    This funcion re-determines the colors assigned to all facelets, based on HSV value measured.
    Baseline:  White has higest V (Value) and lowest S (Saturation) than the other colors; Used '(V-S)' parameter
               Not-white facelets are evaluated only by their H (Hue) value
    Hue (range 0 to 180) average values:  Orange 6, Yellow 32, Green 85, Blue 110, Red 175 (also < 10)
    Note: Depending on the light conditions could be 0< Red_Hue <10 ; In these cases Orange_Hue > Red_Hue
    Note: In some extreme cases orange_hue >175; In these cases Orange_Hue > Red_Hue
    
    This function returns the interpreted facelet colors, on the Kociemba order.
    The function also returns the cube color sequence, to generate a nice decoration on screen.''' 

    VS_value={}                          # dict to store the V-S (Value-Saturation) value of all facelets
    Hue={}                               # dict to store the Hue value of all facelets
    
    i=0
    for H,S,V in HSV_detected.values():  # Hue, Saturation and Value are retrieved from the HSV dict
        VS_value[i]=int(V)-int(S)        # V-S (value-Saturation) delta value for all the facelet, populates the related dict
        Hue[i]=int(H)                    # Hue, for all the facelets, populates the related dict
        i+=1
    if debug:                            # case the variable debug is set True
        print(f'\nV_values: {VS_value}') # feedback is printed to the terminal
        print(f'\nHue: {Hue}')           # feedback is printed to the terminal
    
    
    # function to get the color (sides) order, list of colored center's facelets and white center facelet  
    cube_color_sequence, HSV_analysis = retrieve_cube_color_order(VS_value,Hue)    
    
    while HSV_analysis == True:          # flag on positive analysis is placed on true, and later evenbtually changed
        
        # getting the white center's facelet and the colored facelets
        centers=[4, 13, 22, 31, 40, 49]                              # center facelet numbers
        white_center = centers[cube_color_sequence.index('white')]   # facelet number for the white center
        centers.remove(white_center)                                 # white facelet center is removed from the list of facelets centers
        colored_centers=centers                                      # colored centers (without the white)
        
        if debug:   # case the variable debug is set True
            print(f'\nWhite facelet: {white_center}')                # feedback is printed to the terminal
            print(f'\nCube_color_sequence: {cube_color_sequence}')   # feedback is printed to the terminal
        
        # searching all the 9 white facelets
        Hw,Sw,Vw=HSV_detected[white_center]                          # HSV of the white center
        if debug:   # case the variable debug is set True
            print(f'White Hw,Sw,Vw: {Hw}, {Sw}, {Vw}')               # HSV of the white center
            print(f'Colored center facelets: {colored_centers}')     # colored center facelets

        VSdelta={}                                                   # dict to store the V-S (value-Satuartion) value of all facelets
        i=0
        for H,S,V in HSV_detected.values():                          # Hue, Saturation and Value are retrieved from the HSV dict
            VSdelta[i]=int(V)+int(V)-int(S)-abs(3*(int(H)-int(Hw)))  # V+(V-S)+abs(3*deltaH) value for all the facelets
            i+=1
        
        # white facelets are the brightest, least saturated stickers and can include a logo/print;
        # a simple top-9 V-S ranking is too strict in that case, so we allow small brightness/saturation tolerance
        # and fall back to a brightness ranking if the center-based whitelist does not find 9 candidates.
        white_ref_v = max(100, Vw)
        white_ref_s = max(20, Sw)
        white_candidates = []
        for facelet, (Hf, Sf, Vf) in HSV_detected.items():
            if facelet in colored_centers or facelet == white_center:
                continue
            if Vf >= white_ref_v * 0.72 and Sf <= max(white_ref_s * 2.0, 60):
                white_candidates.append((facelet, int(Vf) - int(Sf)))

        if len(white_candidates) < 9:
            for facelet, (Hf, Sf, Vf) in HSV_detected.items():
                if facelet in colored_centers or facelet == white_center:
                    continue
                white_candidates.append((facelet, int(Vf) - int(Sf)))

        if white_candidates:
            white_candidates = sorted(white_candidates, key=lambda item: item[1], reverse=True)
            white_facelets_list = [facelet for facelet, _ in white_candidates[:9]]
        else:
            # old fallback if no bright white facelets are detected at all
            VSdelta_copy=VSdelta.copy()
            VSdelta_ordered={k: v for k, v in sorted(VSdelta_copy.items(), key=lambda item: item[1])}
            key_ordered_by_VSdelta = [x for x in VSdelta_ordered.keys()]
            white_facelets_list = key_ordered_by_VSdelta[-9:]

        if debug:   # case the variable debug is set True
            print(f'White facelets: {white_facelets_list}')   # feedback is printed to the terminal
            print(f'\nHue dict all facelets {Hue}')           # feedback is printed to the terminal
        
        
        white_facelets={}  # empty dict to store the association of facelet position for the white facelets
        
        # white facelets are removed from Hue dictionary, as meant for colored facelets
        for facelet in white_facelets_list:                # iteration on all the white facelets listed
            try:
                del Hue[facelet]                           # all white facelets are removed from the Hue dictionary
                white_facelets[facelet]='white'            # dictionary with white facelets is populated
            except:
                HSV_analysis = False
                break
        
        if debug:   # case the variable debug is set True
            print(f'\nHue dict colored facelets {Hue}')    # feedback is printed to the terminal)
        
        # facelet's color selection by Hue distance from each center's Hue
        centers=[4,13,22,31,40,49]                                # facelets of centers
        centers.remove(white_center)                              # facelets for colored centers
        cube_color_sequence_no_white=cube_color_sequence.copy()   # cube color sequence copy for list without white
        cube_color_sequence_no_white.remove('white')              # cube color sequence, for colored centers
        
        try:
            Hcenters=[Hue[x] for x in colored_centers]            # list of Hue values for the colored centers
            # an error can happen if the same cube side has been showed twice or more
        except:
            if debug:   # case the variable debug is set True
                print('\nNot found 6 different colors at center facelets')   # feedback is printed to the terminal
            HSV_analysis=False
            break
        
        if debug:   # case the variable debug is set True
            print(f'\nHcenters (no white): {Hcenters}')    # feedback is printed to the terminal
        
        Hc_red=Hue[colored_centers[cube_color_sequence_no_white.index('red')]]     # Hue value for the red center facelet
        Hc_blue=Hue[colored_centers[cube_color_sequence_no_white.index('blue')]]   # Hue value for the blue center facelet
        red_blue_avg=(Hc_red+Hc_blue)//2                                           # mid Hue value between red and blue
        
        # red is supposed to have Hue >160, yet sometime it is below 10
        # distance from the Hue of center facelets is used to decide the facelet's color
        color_facelets={}                                           # dict to store all the colored facelets (NO WHITE)
        for facelet, H in Hue.items():                              # iteration on all the color facelets
            Hdistance=[]                                            # list to store the Hue distance from the 5 colored centers                   
            if sum(value>red_blue_avg for value in Hue.values())<5: # condition suggesting some of the red facelets have Hue < 10
                for Hc in Hcenters:                                 # iteration over the 5 reference Hue colors
                    if H>red_blue_avg:                              # when the facelet's Hue is above mid distance between Red center and Blue center
                        H=0                                         # Hue is force to 0
                    if Hc>red_blue_avg:                             # Red center Hue (when the center facelet is above mid distance between Red center and Blue )
                        Hc=0                                        # Red center Hue is forced to 0
                    dist=min(abs((H-Hc)),180-Hc+H)                  # red can either be 170<Hue<180 as well as <10
                    Hdistance.append(dist)                          # absolute Hue distance toward the 5 colored centers Hue
            else:                                                   # else case is when all the 6 red's facelts Hue are above mid Hue distance blue / red
                for Hc in Hcenters:                                 # iteration over the 5 reference Hue colors
                    dist=min(abs((H-Hc)),180-Hc+H)                  # red can either be 170<Hue<180 as well as <10
                    Hdistance.append(dist)                          # absolute Hue distance toward the 5 colored centers Hue

            color=cube_color_sequence_no_white[Hdistance.index(min(Hdistance))] # right color has min distance from center facelet of same color
            color_facelets[facelet]=color                                       # color is assigned to dictionary


        # Merging the white and color facelets dictionaries, and sorting it by key
        cube_status_detected={**white_facelets, **color_facelets}                                      # dict with white and colored facelets
        cube_status_detected={k: v for k, v in sorted(cube_status_detected.items(), key=lambda item: item[0])}   # dict with white & color ordered by facelets

        # cube can be presented to the laptop camera on a different color order than used by Kociemba solver
        std_cube_color=['white', 'red', 'green', 'yellow', 'orange', 'blue']             # conventional color sequence for kociemba solver
        if cube_color_sequence != std_cube_color:
            cube_status_kociemba={}                                                      # dict with cube status using conventional colors
            i=0
            for color in cube_status_detected.values():                                  # iteration on dict with detected colors
                cube_status_kociemba[i]=std_cube_color[cube_color_sequence.index(color)] # colors are exchanged, and assigned to the dict for solver
                i+=1
            if debug:   # case the variable debug is set True
                print(f'\nCube_status_detected_colors: {cube_status_detected}')          # feedback is printed to the terminal
                print(f'\nCube_status_conventional_colors: {cube_status_kociemba}\n')    # feedback is printed to the terminal
            break # wrong indentation until 22 May 2022
            
        elif cube_color_sequence == std_cube_color:
            cube_status_kociemba=cube_status_detected
            if debug:   # case the variable debug is set True
                print('\nCube colors and orientation according to kociemba order')       # feedback is printed to the terminal
                print(f'\nCube_status_detected_colors: {cube_status_detected}\n')        # feedback is printed to the terminal
            break # wrong indentation until 22 May 2022
    
    if HSV_analysis==False:
        cube_status_kociemba={}
        cube_status_detected={}
    else:
        # see the matching comment in cube_colors_interpreted(): a center facelet's color is a structural fact
        # of the cube (U's center is always white, etc.), not something to classify - force it here too.
        for pos, color in CENTER_COLOR.items():
            cube_status_kociemba[pos] = color

    # cube_status_kociemba uses the conventional colors for kociemba solver
    # cube_status_detected has the detected colors, via the HSV approach. This dict is used for decoration purpose
    return cube_status_kociemba, cube_status_detected, cube_color_sequence







def cube_colors_interpreted_sketch(cube_status, cube_color_sequence, edge, frame, font,                                    fontScale, lineType, color_detection_winner):
    ''' Based on the detected cube status, a sketch of the cube is plot with bright colors.''' 
    
    x_start=int(edge/3)     # top lef corner of the rectangle where all the cube's faces are plot
    y_start=int(edge*6)     # top lef corner of the rectangle where all the cube's faces are plot
    
    _, square_dict = cube_sketch_coordinates(x_start, y_start, edge)
    d = edge                # edge lenght for each facelet reppresentation
    m = int(edge/4)         # m=margin around the cube reppresentation 
    cv2.rectangle(frame, (x_start-m, y_start-2*edge), (x_start+13*d, int(y_start+9.2*d)), (230,230,230), -1) #gray background
    cv2.putText(frame, 'Interpreted:', (x_start, y_start-int(0.5*d)), font, fontScale*0.85,(0,0,0),lineType)
    cv2.putText(frame, color_detection_winner, (x_start+int(6.5*d), y_start+int(2.5*d)), font, fontScale*0.75,(0,0,0),lineType)

    cube_bright_colors = {'white':(255,255,255), 'red':(0,0,204), 'green':(0,132,0), 'yellow':(0,245,245),
                          'orange':(0,128,255), 'blue':(204,0,0)}
    std_color_sequence = list(cube_bright_colors.keys())
    
    i=0
    for color in cube_status.values():
        col=cube_color_sequence[std_color_sequence.index(color)]  # from 'kociemba cube status' color to detected color
        B,G,R = cube_bright_colors[col]                           # decorating a cube reppresentation with bright colors
        start_point=square_dict[i]                                # top-left poiint coordinate
        cv2.rectangle(frame, tuple(start_point), (start_point[0]+edge, start_point[1]+edge), (0, 0, 0), 1) # squre black frame
        inner_points=inner_square_points(square_dict,i,edge)      # array with the 4 square inner vertex coordinates
        cv2.fillPoly(frame, pts = [inner_points], color=(B,G,R))  # inner square is colored with interpreted color           
        i+=1







def rgb2lab(inputColor):
    ''' Convert RGB (not BGR !!!) in L*a*b colors space
    from: https://gist.github.com/manojpandey/f5ece715132c572c80421febebaf66ae (RGB to CIELab color space conversion)
        Step 1: RGB to XYZ
                http://www.easyrgb.com/index.php?X=MATH&H=02#text2
        Step 2: XYZ to Lab
                http://www.easyrgb.com/index.php?X=MATH&H=07#text7
    
    L*a*b color space is a device-independent, 'standard observer' model, is useful in industry
    for detecting small differences in color.'''
    
    num = 0
    RGB = [0, 0, 0]
    for value in inputColor:
        value = float(value) / 255
        if value > 0.04045:
            value = ((value + 0.055) / 1.055) ** 2.4
        else:
            value = value / 12.92
        RGB[num] = value * 100
        num = num + 1
    XYZ = [0, 0, 0, ]
    X = RGB[0] * 0.4124 + RGB[1] * 0.3576 + RGB[2] * 0.1805
    Y = RGB[0] * 0.2126 + RGB[1] * 0.7152 + RGB[2] * 0.0722
    Z = RGB[0] * 0.0193 + RGB[1] * 0.1192 + RGB[2] * 0.9505
    XYZ[0] = round(X, 4)
    XYZ[1] = round(Y, 4)
    XYZ[2] = round(Z, 4)

    # Observer= 2°, Illuminant= D65
    XYZ[0] = float(XYZ[0]) / 95.047         # ref_X =  95.047
    XYZ[1] = float(XYZ[1]) / 100.0          # ref_Y = 100.000
    XYZ[2] = float(XYZ[2]) / 108.883        # ref_Z = 108.883

    num = 0
    for value in XYZ:
        if value > 0.008856:
            value = value ** (0.3333333333333333)
        else:
            value = (7.787 * value) + (16 / 116)
        XYZ[num] = value
        num = num + 1
    Lab = [0, 0, 0]
    L = (116 * XYZ[1]) - 16
    a = 500 * (XYZ[0] - XYZ[1])
    b = 200 * (XYZ[1] - XYZ[2])

    Lab[0] = round(L, 4)
    Lab[1] = round(a, 4)
    Lab[2] = round(b, 4)
    return Lab







def CIEDE2000(Lab_1, Lab_2):
    ''' Calculates CIEDE2000 color distance between two CIE L*a*b* colors
    from: https://github.com/lovro-i/CIEDE2000
    It returns the Euclidean distance between two colors, and it is used to compare each facelet
    toward the 6 centers.'''
    
    C_25_7 = 6103515625 # 25**7

    L1, a1, b1 = Lab_1[0], Lab_1[1], Lab_1[2]
    L2, a2, b2 = Lab_2[0], Lab_2[1], Lab_2[2]
    C1 = math.sqrt(a1**2 + b1**2)
    C2 = math.sqrt(a2**2 + b2**2)
    C_ave = (C1 + C2) / 2
    G = 0.5 * (1 - math.sqrt(C_ave**7 / (C_ave**7 + C_25_7)))
    
    L1_, L2_ = L1, L2
    a1_, a2_ = (1 + G) * a1, (1 + G) * a2
    b1_, b2_ = b1, b2
    
    C1_ = math.sqrt(a1_**2 + b1_**2)
    C2_ = math.sqrt(a2_**2 + b2_**2)
    
    if b1_ == 0 and a1_ == 0: h1_ = 0
    elif a1_ >= 0: h1_ = math.atan2(b1_, a1_)
    else: h1_ = math.atan2(b1_, a1_) + 2 * math.pi
    
    if b2_ == 0 and a2_ == 0: h2_ = 0
    elif a2_ >= 0: h2_ = math.atan2(b2_, a2_)
    else: h2_ = math.atan2(b2_, a2_) + 2 * math.pi

    dL_ = L2_ - L1_
    dC_ = C2_ - C1_    
    dh_ = h2_ - h1_
    if C1_ * C2_ == 0: dh_ = 0
    elif dh_ > math.pi: dh_ -= 2 * math.pi
    elif dh_ < -math.pi: dh_ += 2 * math.pi        
    dH_ = 2 * math.sqrt(C1_ * C2_) * math.sin(dh_ / 2)
    
    L_ave = (L1_ + L2_) / 2
    C_ave = (C1_ + C2_) / 2
    
    _dh = abs(h1_ - h2_)
    _sh = h1_ + h2_
    C1C2 = C1_ * C2_
    
    if _dh <= math.pi and C1C2 != 0: h_ave = (h1_ + h2_) / 2
    elif _dh  > math.pi and _sh < 2 * math.pi and C1C2 != 0: h_ave = (h1_ + h2_) / 2 + math.pi
    elif _dh  > math.pi and _sh >= 2 * math.pi and C1C2 != 0: h_ave = (h1_ + h2_) / 2 - math.pi 
    else: h_ave = h1_ + h2_
    
    T =1-0.17*math.cos(h_ave-math.pi/6)+0.24*math.cos(2*h_ave)+0.32*math.cos(3*h_ave+math.pi/30)-0.2*math.cos(4*h_ave-63*math.pi/180)
    
    h_ave_deg = h_ave * 180 / math.pi
    if h_ave_deg < 0: h_ave_deg += 360
    elif h_ave_deg > 360: h_ave_deg -= 360
    dTheta = 30 * math.exp(-(((h_ave_deg - 275) / 25)**2))
    
    R_C = 2 * math.sqrt(C_ave**7 / (C_ave**7 + C_25_7))  
    S_C = 1 + 0.045 * C_ave
    S_H = 1 + 0.015 * C_ave * T
    
    Lm50s = (L_ave - 50)**2
    S_L = 1 + 0.015 * Lm50s / math.sqrt(20 + Lm50s)
    R_T = -math.sin(dTheta * math.pi / 90) * R_C

    k_L, k_C, k_H = 1, 1, 1
    
    f_L = dL_ / k_L / S_L
    f_C = dC_ / k_C / S_C
    f_H = dH_ / k_H / S_H
    
    dE_00 = math.sqrt(f_L**2 + f_C**2 + f_H**2 + R_T * f_C * f_H)
    
    return dE_00







def decoration(deco_info):
    ''' Plot the cube's status made by a collage of images taken along the facelets color detection
    On the collage it is also proposed the cube's sketches made with detected and interpreted colors
    This picture collage is saved as image, by adding date and time on the file name as reference.'''
    
    fixWindPos, frame, faces, edge, cube_status, cube_color_sequence, kociemba_facelets_BGR_mean = deco_info[:7]
    font, fontScale, lineType, show_time, timestamp, color_detection_winner = deco_info[7:]

    resume_width = int(13*edge)
    resume_height = 29*edge
    cv2.rectangle(frame, (0, 0), (resume_width, resume_height), (230, 230, 230), -1)    # gray background for the cube sketch
    plot_colors(kociemba_facelets_BGR_mean, edge, frame, font, fontScale, lineType)     # plot a cube decoration with detected colors
    
    cube_colors_interpreted_sketch(cube_status, cube_color_sequence, edge, frame, font,
                                   fontScale, lineType, color_detection_winner)       # cube sketch with with (bright) colors interpreted

    # frame slice, with the sketches of detected and interpreted colours
    faces[7] = frame[3*edge:resume_height, :resume_width] 
    
    collage=faces_collage(faces)                     # collage creation function is called
    
    # saving the collage image to a local folder
    import os                                        # os is imported to ensure the folder check/make
    folder = os.path.join('.','CubesStatusPictures') # folder to store the collage pictures
    if not os.path.exists(folder):                   # if case the folder does not exist
        os.makedirs(folder)                          # folder is made
    
    timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S') # date_time variable for file name purpose
    fname = 'cube_collage_'+timestamp+'.png'          # filename with timestamp for the resume picture
    fname = os.path.join(folder, fname)              # folder+filename with timestamp for the resume picture
    
    if color_detection_winner == 'Error':            # case there is a detection Error'
        print('cube status not coherent')            # feedback is printed to the terminal
    elif color_detection_winner == 'BGR':            # case the cube is correctly detected via 'BGR'
        print('cube status detected via BGR color distance')  # feedback is printed to the terminal
    elif color_detection_winner == 'HSV':            # case the cube is correctly detected via 'HSV'
        print('cube status detected via HSV color analysis')  # feedback is printed to the terminal
    print('Cube status image saved at: ', fname, '\n') # feedback is printed to the terminal
    status=cv2.imwrite(fname, collage)               # cube sketch with detected and interpred colors is saved as image
    
    # showing the collage image on screen
    if fixWindPos:                                   # case the fixWindPos variable is set true on __main__ 
        cv2.namedWindow('cube_collage')              # create the collage window
        cv2.moveWindow('cube_collage', 0,0)          # move the collage window to (0,0)
    cv2.imshow('cube_collage', collage)              # starting be status is shown
    key=cv2.waitKey(int(show_time*1000))             # showtime is trasformed from milliseconds to seconds
    if key == 27:                                    # ESC button can be used to escape each window
        cv2.destroyWindow('cube_collage')            # cube window is closed via esc button          

    try: cv2.destroyWindow('cube_collage')           # cube window is closed at function end

    except: pass







def faces_collage(faces):
    ''' This function merges multipe pictures, and it returns a single image.
    The 6 cube faces images, taken while detecting the facelets colors, are used for this collage.
    The Sketch with detected and interpreted cune is also used on this collage.
    The 6 cube faces images are resized to a predefined dimension.
    Gray rectangles are generated and used to complete the picture.
    Once the collage is made, the original dict of images is cleared, to save some memory.'''
    
    face_h = faces[1].shape[0]                                   # height of the cube's face1 image, still on memory
    face_h = min(face_h,250)                                     # if the cube's face1 image height if bigger than 250 then 250 is chosen
    for i in range(1,7):                                         # image of all cube faces (1 to 6 )
        faces[i]=cv2.resize(faces[i], (face_h, face_h), interpolation = cv2.INTER_AREA)  # are resized to square of 300 pixels, or face1 height
    
    empty_face = np.zeros([face_h, face_h, 3],dtype=np.uint8)    # empty array having same dimensions of cube's faces images
    empty_face.fill(230)                                         # array is filled with light gray
    
    # faces[7] is a resume image, still on memory, with detected and interpreted facelets colors
    resume_h = faces[7].shape[0]            # width of faces[7]
    resume_w = faces[7].shape[1]            # height of faces[7]
    resume_ratio=resume_w/resume_h          # image ratio (w/h) is calculated, as this image differs from the cube's faces images
    resume_h=3*face_h                       # resume image height, to occupy 3 cube's faces
    resume_w=int(3*face_h*resume_ratio)     # resume image width is calculated from the imposed height, by keeping aspect ratio
    resume_resized = cv2.resize(faces[7], (resume_w, resume_h), interpolation = cv2.INTER_AREA) # resume is resized to occupy a full 'column'   
    
    # faces[] is keyed by SCAN STEP (1-6, whatever order 'sides' presented the faces in), but this collage's
    # column layout is laid out by KOCIEMBA POSITION (U,R,F,D,L,B) - look up which scan step captured each
    # position, rather than assuming scan step N was always kociemba position N (true only for the original
    # U,R,F,D,L,B scan order; not for any other order sides may now present the faces in).
    step_for_letter = {letter: step for step, letter in sides.items() if step != 0}
    seq = [step_for_letter[letter] for letter in ('U', 'R', 'F', 'D', 'L', 'B')]

    col1=np.vstack([empty_face, faces[seq[4]], empty_face])        # vertical stack of images, to generate 1st column for the pictures collage
    col2=np.vstack([faces[seq[0]], faces[seq[2]], faces[seq[3]]])  # vertical stack of images, to generate 2nd column for the pictures collage
    col3=np.vstack([empty_face, faces[seq[1]], empty_face])        # vertical stack of images, to generate 3rd column for the pictures collage
    col4=np.vstack([empty_face, faces[seq[5]], empty_face])        # vertical stack of images, to generate 4th column for the pictures collage
    
    faces.clear()                                                  # dictionary of images is cleared

    collage = np.hstack([col1,col2,col3,col4,resume_resized])      # horizzontal stack of 5 columns of images, to generate the pictures collage
    collage_ratio = collage.shape[1] / collage.shape[0]            # collage ratio (width/height) is calculated for resizing 
    collage_w=1024                                                 # colleage width is fixed for consistent pictures archiving
    collage_h=int(collage_w/collage_ratio)                         # colleage heigth is calculated to maintain proportions
    collage = cv2.resize(collage, (collage_w, collage_h), interpolation = cv2.INTER_AREA) # resized collage  
    
    return collage







def cube_string(cube_status):
    ''' Generates the cube detected status string, compatible with the solver:
    All uppercase letters indicating the color's initial
    Argument is the cube status generated, whein the values are the facelet colors (full color name).'''
    
    cube_in_letters = {'white':'U', 'red':'R', 'green': 'F', 'yellow':'D', 'orange':'L', 'blue':'B'}  
    string=''
    for color in cube_status.values():         
        string=string+str(cube_in_letters[color])
    return string







def cube_solution(cube_string):
    ''' Calls the Hegbert Kociemba solver, and returns the solution's moves
    from: https://github.com/hkociemba/RubiksCube-TwophaseSolver 
    (Solve Rubik's Cube in less than 20 moves on average with Python)
    The returned string is slightly manipulated to have the moves amount at the start.'''    
    s = sv.solve(cube_string, 20, 2)  # solves with a maximum of 20 moves and a timeout of 2 seconds for example
    solution = s[:s.find('(')]        # solution capture the sequence of manouvre
    
    # solution_text places the amount of moves first, and the solution (sequence of manouvere) afterward
    solution_Text = s[s.find('(')+1:s.find(')')-1]+' moves  '+ s[:s.find('(')] 
    if solution[:5] =='Error':        # solution could start with 'Error' in case of incoherent cusbe string sent to the solver
        solution_Text = 'Error'       # in that case a short error string is returned
    
    return solution, solution_Text







def text_bg(frame, w, h):
    ''' Sets background_h, a small top/bottom margin that keeps the detection zone off the very edges of the
    frame. Instructions/status text now live in the sidebar (see render_sidebar()), not painted over the
    camera view, so this no longer draws anything - it only keeps the margin value other code relies on.'''

    global background_h                                                 # this variable is re-used to reduce the frame to a ROI
    background_h=20                                                     # small margin, kept off the very top/bottom edge







def text_font():
    ''' Sets the (main) text parameters, used on CV2.'''  
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.8
    fontColor = (255,255,255)
    lineType = 2
    return font, fontScale, fontColor, lineType


# ---------------------------------------------------------------------------------------------------------
# Modern window layout: the 'cube' window is a camera-view block (clean video feed, no text burned onto it)
# with a sidebar block to its right holding a title/status line, plain-language instructions, a compact
# per-face capture-progress strip, and clickable buttons - a mouse click on a button is translated to the
# same action a keyboard shortcut would trigger, so both stay in sync automatically.
# ---------------------------------------------------------------------------------------------------------

SIDEBAR_W = 320
TOP_BTN_H = 34                   # slightly shorter than the main 42px action buttons - a visibly distinct,
                                  # secondary-feeling slot for a standalone setup action like "Calibrate colors"
SIDEBAR_BG = (247, 247, 245)     # BGR, light neutral background
TEXT_PRIMARY = (35, 35, 35)
TEXT_SECONDARY = (110, 110, 110)
ACCENT = (200, 120, 30)          # BGR blue accent, for the primary button
ACCENT_ACTIVE = (175, 100, 15)   # darker, for hover/press feedback
DANGER = (60, 60, 210)           # BGR red, for quit
DANGER_ACTIVE = (45, 45, 180)
BTN_SECONDARY_BG = (222, 222, 222)
BTN_SECONDARY_ACTIVE = (200, 200, 200)
DIVIDER = (216, 216, 214)
INSTRUCTION_BLOCK_H = 104   # fixed height reserved for the instructions block: 2 items x up to 2 wrapped lines
                            # each (2*22) + the 8px per-item gap, regardless of how much a given state's text
                            # actually needs - keeps the sidebar layout (and window size) identical across
                            # states even when their instruction text wraps to different line counts

current_button_rects = {}   # {'accept': (x1,y1,x2,y2), ...} in FULL composed-canvas coordinates
mouse_clicked_action = None  # set by on_cube_mouse(), consumed (and cleared) once per main-loop iteration
hover_action = None           # action name currently under the mouse, or None; drives the button hover highlight


try:                              # best-effort real hand cursor on hover (Windows only; harmless no-op elsewhere).
    import ctypes                 # cv2 itself has no per-region cursor API, but the window is a normal Win32
    _user32 = ctypes.windll.user32  # window underneath, so its cursor can still be set directly via user32 -
    _IDC_ARROW, _IDC_HAND = 32512, 32649   # this has to be re-asserted on every mouse move (not just on
except Exception:                          # hover/unhover transitions), since Windows' own window procedure
    _user32 = None                         # resets it to the default arrow in between our calls otherwise.


def _set_windows_hand_cursor(is_hand):
    if _user32 is None:
        return
    try:
        _user32.SetCursor(_user32.LoadCursorW(0, _IDC_HAND if is_hand else _IDC_ARROW))
    except Exception:
        pass


def on_cube_mouse(event, x, y, flags, param):
    ''' Mouse callback for the 'cube' window: a left click inside one of the sidebar's current button
    rectangles (tracked in current_button_rects, refreshed every frame by compose_and_show()) sets
    mouse_clicked_action to that button's action name, for the main loop to act on alongside keypresses.
    Also drives two hover cues: a real Windows hand cursor (_set_windows_hand_cursor(), best-effort - cv2
    itself has no per-region cursor API) and a highlighted button fill (hover_action, drawn by draw_button()
    next frame) as a visible fallback wherever the cursor swap doesn't apply (non-Windows, or if it silently
    fails).'''

    global mouse_clicked_action, hover_action
    if event == cv2.EVENT_LBUTTONDOWN:
        for action, (x1, y1, x2, y2) in current_button_rects.items():
            if x1 <= x <= x2 and y1 <= y <= y2:
                mouse_clicked_action = action
                break
    elif event == cv2.EVENT_MOUSEMOVE:
        hover_action = None
        for action, (x1, y1, x2, y2) in current_button_rects.items():
            if x1 <= x <= x2 and y1 <= y <= y2:
                hover_action = action
                break
        _set_windows_hand_cursor(hover_action is not None)


DISABLED_BTN_BG = (235, 235, 235)   # BGR, near-invisible against the sidebar background
DISABLED_BTN_FG = (190, 190, 190)


def draw_button(img, rect, label, style, font, hovered=False, enabled=True):
    ''' Draws one button (a filled rounded-ish rectangle with a centered label) at rect=(x1,y1,x2,y2).
    style: 'primary' (accent-filled), 'danger' (red-filled), or 'secondary' (light gray).
    hovered: draws it in its active-style shade (same look a real button gives on mouse-over) - the closest
    substitute available for a hand cursor, which cv2 windows can't show per-region.
    enabled=False: draws it faded/inert instead of hiding it - the sidebar's button set stays constant across
    every scanning state (only which ones are clickable changes), so the window never needs to resize or
    have controls appear/disappear as you move between states.'''

    x1, y1, x2, y2 = rect
    if not enabled:
        bg, fg = DISABLED_BTN_BG, DISABLED_BTN_FG
    elif hovered:
        bg = ACCENT_ACTIVE if style == 'primary' else (DANGER_ACTIVE if style == 'danger' else BTN_SECONDARY_ACTIVE)
        fg = (255, 255, 255) if style in ('primary', 'danger') else TEXT_PRIMARY
    else:
        bg = ACCENT if style == 'primary' else (DANGER if style == 'danger' else BTN_SECONDARY_BG)
        fg = (255, 255, 255) if style in ('primary', 'danger') else TEXT_PRIMARY
    cv2.rectangle(img, (x1, y1), (x2, y2), bg, -1)
    cv2.rectangle(img, (x1, y1), (x2, y2), (200, 200, 200), 1, cv2.LINE_AA)
    (tw, th), _ = cv2.getTextSize(label, font, 0.62, 1)
    cv2.putText(img, label, (x1+(x2-x1-tw)//2, y1+(y2-y1+th)//2), font, 0.62, fg, 1, cv2.LINE_AA)


def wrap_text(text, max_chars):
    ''' Simple word-wrap: splits text into lines no longer than max_chars, breaking on spaces.'''

    words = text.split(' ')
    lines, cur = [], ''
    for word in words:
        trial = (cur+' '+word).strip()
        if len(trial) > max_chars and cur:
            lines.append(cur)
            cur = word
        else:
            cur = trial
    if cur:
        lines.append(cur)
    return lines


def compute_face_progress():
    ''' Returns {face_letter: display_bgr} for each cube face captured so far this scan (average of its 9
    facelets, classified to one of the 6 predefined colors), used for the sidebar's progress strip.'''

    progress = {}
    for i in range(len(BGR_mean)//9):
        letter = sides[i+1]
        face_bgrs = BGR_mean[i*9:(i+1)*9]
        avg = tuple(int(sum(c[k] for c in face_bgrs)/9) for k in range(3))
        progress[letter] = classify_bgr_to_6(avg)[1]
    return progress


def render_face_net(img, x0, y0, edge, font, current_letter, captured_letters, face_colors=None, clickable=False):
    ''' Draws the 6 cube faces as their standard unfolded-net layout (U on top, L-F-R-B in a row, D on the
    bottom - the same layout as a real cube's faces adjoin each other), so a face letter maps to a color and
    a position at a glance instead of needing to be memorized. Each cell shows face_colors[letter] if given
    (e.g. the actually captured color, for the post-scan review screen), else its standard reference color
    (U=white, R=red, F=green, D=yellow, L=orange, B=blue). The face currently being requested gets a thick
    accent border; already-captured faces get a small dot badge. clickable=True (the review screen's
    click-to-rescan) also highlights whichever cell is currently under the mouse (see hover_action), the same
    substitute-for-a-hand-cursor treatment draw_button() gives regular buttons.
    Returns (width, height, cell_rects) - cell_rects is {letter: (x1,y1,x2,y2)} in img-local coordinates, for
    callers that want the cells themselves to be clickable (e.g. click-to-rescan on the review screen).'''

    cell_rects = {}
    for letter, (col, row) in NET_LAYOUT.items():
        x, y = x0+col*edge, y0+row*edge
        cell_rects[letter] = (x, y, x+edge, y+edge)
        std_color_name = STD_LETTER_COLOR[letter]
        color = (face_colors or {}).get(letter, DEFAULT_REF_COLORS[std_color_name])
        is_hovered = clickable and hover_action == f'rescan_{letter}'
        if is_hovered:                                # dim the fill slightly: same visual language as a hovered button
            color = tuple(max(0, c-45) for c in color)
        cv2.rectangle(img, (x, y), (x+edge, y+edge), color, -1)
        is_current = letter == current_letter
        border_color = ACCENT if is_current else ((90, 90, 90) if is_hovered else (150, 150, 150))
        border_w = 3 if (is_current or is_hovered) else 1
        cv2.rectangle(img, (x, y), (x+edge, y+edge), border_color, border_w, cv2.LINE_AA)
        dark_bg = std_color_name in ('white', 'yellow')
        txt_color = TEXT_PRIMARY if dark_bg else (255, 255, 255)
        (tw, th), _ = cv2.getTextSize(letter, font, 0.55, 1)
        cv2.putText(img, letter, (x+(edge-tw)//2, y+(edge+th)//2), font, 0.55, txt_color, 1, cv2.LINE_AA)
        if letter in captured_letters:               # small dot badge in the corner, clear of the center letter
            cv2.circle(img, (x+edge-8, y+8), 4, txt_color, -1, cv2.LINE_AA)
            cv2.circle(img, (x+edge-8, y+8), 4, (90, 90, 90), 1, cv2.LINE_AA)

    return 4*edge, 3*edge, cell_rects


def render_sidebar(height, font, title, instructions, buttons, face_progress, preview_face=None, current_letter=None,
                    face_colors=None, clickable_net=False, net_edge=40, top_button=None, editable_preview=False):
    ''' Renders the sidebar panel (SIDEBAR_W x height): title, an optional standalone setup action (top_button
    - visually separated by its own thin divider, since it's a different kind of action from the scan flow
    below it, e.g. "Calibrate colors"), instructions (list of strings, word-wrapped), a 3x3 preview of a
    just-captured face's 9 classified colors (blank placeholder cells when preview_face is None, so this
    block always takes the same space), the 6-face unfolded-net map (see render_face_net() - shows which
    color/position each face letter is, and highlights current_letter as the one to show now), and a stack of
    buttons at the bottom.
    buttons/top_button: (action, label, style) or (action, label, style, enabled) tuples - enabled defaults to
    True; a disabled button is drawn faded and doesn't register a click, but still takes up its normal space,
    so the whole sidebar's layout (and therefore the window size) stays constant across every state - only
    which buttons are usable changes. This also means the button SET passed in should be the same across
    every state a given screen cycles through (e.g. the whole scan loop), just with different enabled flags.
    face_colors: {letter: display_bgr} to show each net cell's actually-captured color instead of the fixed
    reference (used by the post-scan review screen). clickable_net: when True, the net's 6 cells are added to
    the returned rects too, as actions 'rescan_U'/'rescan_R'/etc, so a click on a face triggers rescanning it.
    editable_preview: when True (and preview_face is not None), each of the 9 preview cells is added to the
    returned rects as 'edit_0'..'edit_8', so a click on one facelet can cycle its color without a full rescan.
    Returns (sidebar_img, button_rects) where button_rects is {action: (x1,y1,x2,y2)} in sidebar-local
    coordinates (compose_and_show() offsets these into full-canvas coordinates for click detection).'''

    img = np.full((height, SIDEBAR_W, 3), SIDEBAR_BG, dtype=np.uint8)
    rects = {}

    y = 34
    cv2.putText(img, title, (20, y), font, 0.8, TEXT_PRIMARY, 2, cv2.LINE_AA)
    y += 18
    cv2.line(img, (20, y), (SIDEBAR_W-20, y), DIVIDER, 1, cv2.LINE_AA)
    y += 20

    if top_button is not None:
        action, label, style = top_button[0], top_button[1], top_button[2]
        enabled = top_button[3] if len(top_button) > 3 else True
        rect = (20, y, SIDEBAR_W-20, y+TOP_BTN_H)
        draw_button(img, rect, label, style, font, hovered=(enabled and action == hover_action), enabled=enabled)
        if enabled:
            rects[action] = rect
        y += TOP_BTN_H+10
        cv2.line(img, (20, y), (SIDEBAR_W-20, y), DIVIDER, 1, cv2.LINE_AA)
        y += 20

    # instructions get a FIXED block height (INSTRUCTION_BLOCK_H) regardless of how many lines this particular
    # state's text actually wraps to, so differing text length across states can't shift the layout below it
    # by a few pixels and force yet another window resize - shorter text just leaves blank space beneath it.
    instr_y0 = y
    for line in instructions:
        for wrapped in wrap_text(line, 34):
            cv2.putText(img, wrapped, (20, y), font, 0.55, TEXT_SECONDARY, 1, cv2.LINE_AA)
            y += 22
        y += 8
    y = instr_y0 + INSTRUCTION_BLOCK_H

    y += 6
    label = 'Captured colors (click to fix one)' if editable_preview else 'Captured colors'
    cv2.putText(img, label, (20, y), font, 0.5, TEXT_SECONDARY, 1, cv2.LINE_AA)
    y += 12
    cell = 30
    for row in range(3):
        for col in range(3):
            idx = row*3+col
            cx, cy = 20+col*cell, y+row*cell
            col_bgr = preview_face[idx] if preview_face is not None else SIDEBAR_BG
            action = f'edit_{idx}'
            hovered = editable_preview and preview_face is not None and hover_action == action
            cv2.rectangle(img, (cx, cy), (cx+cell-2, cy+cell-2), col_bgr, -1)
            border_color = (90, 90, 90) if hovered else (170, 170, 170)
            cv2.rectangle(img, (cx, cy), (cx+cell-2, cy+cell-2), border_color, 2 if hovered else 1, cv2.LINE_AA)
            if editable_preview and preview_face is not None:
                rects[action] = (cx, cy, cx+cell-2, cy+cell-2)
    y += 3*cell+14

    y += 2
    cv2.putText(img, 'Face map', (20, y), font, 0.55, TEXT_SECONDARY, 1, cv2.LINE_AA)
    y += 12
    net_w, net_h, net_cell_rects = render_face_net(img, 20, y, net_edge, font, current_letter,
                                                     set(face_progress.keys()), face_colors, clickable_net)
    y += net_h+14

    if clickable_net:
        rects.update({f'rescan_{letter}': rect for letter, rect in net_cell_rects.items()})

    btn_h, btn_gap = 42, 12
    by = height - 20 - len(buttons)*(btn_h+btn_gap) + btn_gap
    for button in buttons:
        action, label, style = button[0], button[1], button[2]
        enabled = button[3] if len(button) > 3 else True
        rect = (20, by, SIDEBAR_W-20, by+btn_h)
        draw_button(img, rect, label, style, font, hovered=(enabled and action == hover_action), enabled=enabled)
        if enabled:
            rects[action] = rect
        by += btn_h+btn_gap

    return img, rects


last_canvas_size = None   # (w, h) of the last canvas shown; compose_and_show() resizes the window when this changes


def compose_and_show(camera_frame, sidebar_img, sidebar_rects):
    ''' Horizontally joins the camera view and the sidebar into one canvas, updates the global
    current_button_rects (offset into full-canvas coordinates, for on_cube_mouse() to hit-test against),
    and shows the result in the 'cube' window. Returns the composed canvas.

    cv2's WINDOW_NORMAL always stretches whatever's shown to exactly fill the window's current rect, with no
    letterboxing. The composed canvas's own height varies by state (the accept/retry screen needs more room
    than plain scanning - see estimate_sidebar_height()), but the window itself only gets resized once at
    startup; on every later state change the canvas no longer matches the window's size, so it gets visibly
    stretched. Fix: whenever the canvas's own pixel size changes, resize the window to match it exactly (1:1,
    no scaling) before showing it. A user who then manually drags the window will still see it stretch until
    the next state change resyncs it - that's an explicit user action, not a bug in what we draw.'''

    global current_button_rects, last_canvas_size
    canvas = np.hstack([camera_frame, sidebar_img])
    cam_w = camera_frame.shape[1]
    current_button_rects = {a: (x1+cam_w, y1, x2+cam_w, y2) for a, (x1, y1, x2, y2) in sidebar_rects.items()}

    canvas_size = (canvas.shape[1], canvas.shape[0])   # (w, h)
    if canvas_size != last_canvas_size:
        try:
            cv2.resizeWindow('cube', canvas_size[0], canvas_size[1])
        except Exception:
            pass
        last_canvas_size = canvas_size

    cv2.imshow('cube', canvas)
    return canvas


MIN_SIDEBAR_H = 250   # hard floor: below this even the lightest state (title + one line) wouldn't fit


def estimate_sidebar_height(instructions, buttons, preview_face, net_edge=40, top_button=None):
    ''' Mirrors render_sidebar()'s own top-down layout arithmetic (title/divider, optional top_button,
    instructions, 3x3 preview, face map, buttons) to compute exactly how tall the sidebar needs to be.
    preview_face no longer changes this (that block is always reserved now, whether or not there's a preview
    to show, precisely so this height - and therefore the window size - stays constant across every state of
    a given screen instead of resizing/jumping around; the parameter is kept only so callers don't need
    updating). If render_sidebar()'s layout constants change, keep these in sync.'''

    y = 34+18+20                                     # title + divider
    if top_button is not None:
        y += TOP_BTN_H+10+20                          # top button + its own divider
    y += INSTRUCTION_BLOCK_H                          # instructions: fixed block, regardless of actual wrap count
    y += 6+12+3*30+14                                # "Captured colors" label + 3x3 grid (cell=30), always reserved
    y += 2+12                                        # "Face map" label
    y += 3*net_edge+14                                # net + margin
    y += 8 + len(buttons)*42 + max(0, len(buttons)-1)*12  # buttons block + its own bottom margin
    return y


def show_cube_frame(camera_frame, cam_h, title, instructions, buttons, preview_face=None, current_letter=None,
                     face_colors=None, clickable_net=False, net_edge=40, top_button=None, editable_preview=False):
    ''' Convenience wrapper for the common case: render the sidebar for the given state (title, optional
    top_button - a standalone setup action visually separated from the scan flow, e.g. "Calibrate colors" -
    instruction lines, buttons, optional 3x3 just-captured-face preview, optional current_letter to highlight
    in the face map), and compose+display it against camera_frame.
    camera_frame/cam_h are taken as explicit arguments (not read off a global) since several callers
    (window_for_cube_rotation(), calibrate_cube_colors()) keep their own local frame/h that never gets
    written back to the module-level globals - reading those here would silently show a stale frame.
    Used at every point in cubeAF() (and the other window-driving functions) that used to just call
    cv2.imshow('cube', frame).'''

    canvas_h = max(cam_h, MIN_SIDEBAR_H, estimate_sidebar_height(instructions, buttons, preview_face, net_edge, top_button))
    if canvas_h != cam_h:            # camera view doesn't match the sidebar's required height: scale it to fit,
        scale = canvas_h/cam_h       # preserving aspect ratio (no distortion) - a slightly-zoomed-in live feed
        new_w = int(camera_frame.shape[1]*scale)         # reads better than a large dead gray bar underneath it
        interp = cv2.INTER_LINEAR if scale > 1 else cv2.INTER_AREA
        camera_frame = cv2.resize(camera_frame, (new_w, canvas_h), interpolation=interp)

    sidebar_img, rects = render_sidebar(canvas_h, font, title, instructions, buttons, compute_face_progress(),
                                         preview_face, current_letter, face_colors, clickable_net, net_edge,
                                         top_button, editable_preview)
    compose_and_show(camera_frame, sidebar_img, rects)


ACTION_TO_KEY = {'accept': 32, 'proceed': 32, 'sample': 32, 'retry': ord('r'), 'quit': 27, 'calibrate': ord('c'), 'cancel': 27}


def poll_key(wait_ms):
    ''' Like cv2.waitKey(wait_ms), but also folds in a pending sidebar button click (see on_cube_mouse()),
    translating it to the same key code the equivalent keyboard shortcut would produce - so every existing
    key==... check elsewhere keeps working unchanged for both input methods. A click takes priority over
    whatever key(if any) was simultaneously pressed, and is consumed (cleared) either way.'''

    global mouse_clicked_action
    key = cv2.waitKey(wait_ms)
    if mouse_clicked_action is not None:
        key = ACTION_TO_KEY.get(mouse_clicked_action, key)
        mouse_clicked_action = None
    return key


def poll_action(wait_ms):
    ''' Like poll_key(), but returns the raw clicked action name too (key, action), for screens with actions
    that have no single keyboard-shortcut equivalent - e.g. the review screen's 6 different 'rescan_<letter>'
    actions, one per clickable face in the map, which poll_key()'s one-action-to-one-key translation can't
    represent.'''

    global mouse_clicked_action
    key = cv2.waitKey(wait_ms)
    action = mouse_clicked_action
    mouse_clicked_action = None
    return key, action


def average_color(image, x, y):
    ''' From: https://sighack.com/post/averaging-rgb-colors-the-right-way
     Averages the pixels within a square defined area on an image
     The average is calculated as the square root of the sum of the squares for the BGR colors
     region centered at (x, y) meant as the square center, with 2*side as quare side lenght in pixels.
     The function return a tuple with the averaged BGR colors
     Pixel by pixel iteration doesn't sound efficient, yet for the small aarea I couldn't notice a time increment.'''
    
    global edge
    # square edge, used for sketching the cube, is used as (half) side of the square to calculate the averaged color
    
    blue=float(0)
    green=float(0)
    red=float(0)
    
    #Iterate through pixels of a bounding box having 2*edge as square side length in pixels
    for i in range(2*edge):         # foor loop used to iterate the colums on the image square 
        j=i-edge                    # iterator j is 'shifted' by half of the square of pixels to analyse
        for i in range(2*edge):     # for loops to iterate trhough the rows of th eimage square
            bgr=image[y+j,x-edge+i]      # gbr of a singele pixel
            b,g,r = bgr                  # bgr components
            b=int(b)                     # from uint8 to integer
            g=int(g)                     # from uint8 to integer
            r=int(r)                     # from uint8 to integer
            
            #Sum the squares of components
            blue=blue+b*b                # progressive sum of the square values for the blue component
            green=green+g*g              # progressive sum of the square values for the green component
            red=red+r*r                  # progressive sum of the square values for the red component
    num=4*edge*edge                      # amount of pixels in the image square under analysis    
    
    # for debug purpose it is drawn the contour of the used area where the facelet's color is averaged 
    if debug:                            # case the debug variable is set True
        tl=(x-edge, y-edge)              # top left coordinate 
        tr=(x+edge, y-edge)              # top right coordinate 
        br=(x+edge, y+edge)              # bottom left coordinate 
        bl=(x-edge, y+edge)              # bottom left coordinate 
        pts=np.array([tl, tr, br, bl])   # array of coordinates
        contour = [pts]                  # list is made with the array of coordinates
        cv2.drawContours(frame, contour, -1, (230, 230, 230), 2)  # a white polyline is drawn on the contour (2 px thickness)
    
    #Return the sqrt of the mean of squared B, G, and R sums
    return (int(math.sqrt(blue/num)), int(math.sqrt(green/num)), int(math.sqrt(red/num)))


def robust_facelet_color(image, x, y):
    ''' Samples a facelet's color from 5 small patches (dead-center plus the 4 diagonal quadrants, all safely
    within the facelet) instead of only dead-center, and returns their per-channel median.
    A cube's center pieces very often carry a small manufacturer logo printed right in the middle of the
    sticker; sampling only dead-center risks averaging in that logo's ink instead of the sticker's actual
    color (this is what made a solved white face's center read as blue). The median of several patches shrugs
    off any one patch being corrupted by a logo, glare, or scratch, without needing to special-case which
    facelets are centers.'''

    global edge
    patch = max(4, edge//2)          # small sampling half-size per patch, so an off-center patch mostly clears a centered logo
    off = edge                       # how far off-center the 4 diagonal patches sit
    offsets = [(0, 0), (-off, -off), (off, -off), (-off, off), (off, off)]

    samples = []
    for dx, dy in offsets:
        region = image[y+dy-patch:y+dy+patch, x+dx-patch:x+dx+patch]
        if region.size == 0:
            continue
        samples.append((float(np.mean(region[:,:,0])), float(np.mean(region[:,:,1])), float(np.mean(region[:,:,2]))))

    return (int(statistics.median(s[0] for s in samples)),
            int(statistics.median(s[1] for s in samples)),
            int(statistics.median(s[2] for s in samples)))


def load_color_calibration():
    ''' Loads a previously saved color calibration from disk, if present, into the global calibrated_colors
    dict. Ported from kkoomen/qbr's calibrate mode: measuring the 6 colors directly off the user's own cube,
    under their own lighting, once - rather than only ever relying on whatever the live scan's center facelets
    happen to read - gives color classification a much more reliable starting point.'''

    global calibrated_colors
    calibrated_colors = {}
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, 'r') as f:
                data = json.load(f)
            calibrated_colors = {color: tuple(bgr) for color, bgr in data.items()}
            print(f'\nLoaded color calibration from {CALIBRATION_FILE}: {calibrated_colors}')
        except Exception as ex:
            print(f'\nCould not load color calibration ({ex}), starting uncalibrated')
            calibrated_colors = {}


def save_color_calibration():
    ''' Persists the global calibrated_colors dict to disk, so it's automatically reloaded next run.'''

    try:
        with open(CALIBRATION_FILE, 'w') as f:
            json.dump({color: list(bgr) for color, bgr in calibrated_colors.items()}, f)
        print(f'\nSaved color calibration to {CALIBRATION_FILE}')
    except Exception as ex:
        print(f'\nCould not save color calibration: {ex}')


def classify_bgr_to_6(bgr):
    ''' Classifies a raw captured BGR color into the nearest of the 6 known cube colors (white, red, green,
    yellow, orange, blue), via CIEDE2000 distance in Lab space against calibrated_colors if available, else
    the DEFAULT_REF_COLORS fallback palette. Returns (color_name, display_bgr).
    display_bgr is always DEFAULT_REF_COLORS[color_name] - a clean, bright, fixed BGR - regardless of which
    reference set decided the match: calibrated_colors is only ever used to decide WHICH of the 6 names is
    closest, never as the color painted on screen. A bad/dim calibration sample would otherwise get painted
    back verbatim on every facelet classified to that color, looking like none of the 6 predefined colors at
    all - exactly what showed up when an early, logo-skewed calibration was in use.'''

    ref = calibrated_colors if len(calibrated_colors) == 6 else DEFAULT_REF_COLORS
    B, G, R = bgr
    lab_meas = rgb2lab([R, G, B])
    best_color, best_dist = None, None
    for color, (rb, rg, rr) in ref.items():
        lab_ref = rgb2lab([rr, rg, rb])
        dist = CIEDE2000(tuple(lab_meas), tuple(lab_ref))
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_color = color
    return best_color, DEFAULT_REF_COLORS[best_color]


def calibrate_cube_colors():
    ''' Interactive calibration: for each of the 6 cube colors in turn, show that color to the camera and
    press SPACE to sample it (the center facelet of whatever 3x3 grid is currently detected). ESC cancels
    without changing the existing calibration. On completion, updates the global calibrated_colors dict and
    persists it to disk. Ported from kkoomen/qbr's calibrate mode.'''

    global calibrated_colors, frame, w, h
    colors_to_calibrate = ['white', 'green', 'yellow', 'orange', 'red', 'blue']   # same order as the scan (sides dict)
    new_calibration = {}

    for step, color_name in enumerate(colors_to_calibrate):
        sampled = False
        while not sampled:
            frame, w, h = read_camera()
            rects = detect_in_fixed_zone(frame, w, h)
            ready = len(rects) == 9
            hint = 'Zone is green - press SAMPLE.' if ready else 'Fill the frame with that color until the box turns green.'
            show_cube_frame(frame, h, f'Calibrate ({step+1}/6): {color_name.upper()}', [hint],
                             [('sample', 'Sample', 'primary' if ready else 'secondary'), ('cancel', 'Cancel', 'danger')])
            key = poll_key(30)
            try:
                window_closed = cv2.getWindowProperty('cube', cv2.WND_PROP_VISIBLE) < 1
            except:
                window_closed = False
            if window_closed or key == 27:
                print('\nColor calibration cancelled')
                return False

            if key == 32 and ready:
                cx, cy, cw, ch = rects[4]       # center facelet, index 4 of the 9 in reading order
                bgr = robust_facelet_color(frame, cx + cw//2, cy + ch//2)
                new_calibration[color_name] = tuple(int(c) for c in bgr)
                sampled = True

    calibrated_colors = new_calibration
    save_color_calibration()

    # on-screen confirmation, not just a terminal print - the terminal isn't always what's being watched
    end_time = time.time() + 1.5
    while time.time() < end_time:
        frame, w, h = read_camera()
        show_cube_frame(frame, h, 'Calibration saved!', [f'Saved to {os.path.basename(CALIBRATION_FILE)}'], [])
        cv2.waitKey(30)

    return True





def read_color(facelets, candidates, BGR_mean, H_mean, wait=20, index=0):
    ''' Reads the average BGR color on the each facelet of the cube face just detected.
    Draw the contour used on each facelect (eventually the facelet number), to feedback on correct facelet reading/ordering
    Wait is the time (in ms) to keep each facelet visible while the remaining frame is forced black
    The fuction returns (or updates) global variables, like BGR_mean, hsv, hue, s, v, H_mean.'''
    
    for facelet in facelets:                              # iteration over the 9 facelets just detedcted
        contour = facelet.get('contour')                  # contour of the facelet under analysis
        candidates.append(contour)                        # new contour is added to the candidates list
        cm_point=facelet['cx'],facelet['cy']                              # contour center coordinate
        bgr_mean_sq = robust_facelet_color(frame, facelet['cx'], facelet['cy'])  # median of several patches, robust to a center logo
        BGR_mean.append(bgr_mean_sq)                          # Initially used a simpler mean to average the facelet color
        b,g,r = bgr_mean_sq                                   # BGR (avg) components are retrieved
        BGR_mean_sq = np.array([[[b,g,r]]], dtype=np.uint8)   # BGR are positioned in cv2 array form
        hsv = cv2.cvtColor( BGR_mean_sq, cv2.COLOR_BGR2HSV)   # HSV color space equilavent values, for the average facelet color
        hue, s, v = cv2.split(hsv)                            # HSV components are retrieved
        H_mean.append((hue[0][0]))                            # the (avg) Hue value is stored on a list

        # a progressive facelet numer, 1 to 9, is placed over the facelets
        cv2.putText(frame, str(index+1), (int(facelet.get('cx'))-12, int(facelet.get('cy'))+6), font, fontScale,(0,0,0),lineType)
        index+=1                







def rotate_image(image, image_center, angle):
    ''' This function rotates an image of a given angle with reference to the given center
    This function is used to align the cropped cube face image (before cropping it).
    Overall scope of this funtion is decorative, to generate an unfoalded cube collage of images.'''
    
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    rotated_img = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return rotated_img







def face_image(frame, facelets, side, faces):
    ''' Slice a frame rectangular portion to temporary store the cube image.
    The cube is initialy cropped from the frame.
    1st vertex of Facelets 0, and 3rd vertedx of facelet 8, are used as reference for the cubre cropping from the frame
    Cube face image is first rotated to be aligned with the horizon
    The function returns a dictionary with the (cropped) images of the 6 cube faces
    This function enables the generation of a cube images collage to be plotted, for decoration purpose
        
    A         B 
      0  1  2
      3  4  5 
      5  7  8  
    D         C
    '''
    

    global frame_width

    ##################################################
    # cube is rotated for a better (visual) cropping #
    ##################################################
    avgAx=0; avgAy=0; avgBx=0; avgBy=0                   # variables for facelets contour's average centers 
    for i in [0,3,6]:                                    # iteration on the left columns facelets
        avgAx=avgAx+int(facelets[i].get('cx'))           # sum cx coordinate for 'i' facelet (left column)
        avgAy=avgAy+int(facelets[i].get('cy'))           # sum cy coordinate for 'i' facelet (left column)
        i=i+2                                            # index increment to the right columns facelets
        avgBx=avgBx+int(facelets[i].get('cx'))           # sum cx coordinate for 'i' facelet (right column)
        avgBy=avgBy+int(facelets[i].get('cy'))           # sum cy coordinate for 'i' facelet (right column)

    p=(avgBx//3-avgAx//3, avgBy//3-avgAy//3)             # average (delta x, delta y) for facelets on right column  
    ang = np.arctan2(*p[::-1])                           # angle (in radians) of the cube
    angle = np.rad2deg(ang%(2*np.pi))                    # angle (in degrees) of the cube
    Ax = int(facelets[0].get('cont_ordered')[0][0])      # x coordinate for the top-left vertex 1st facelet
    Ay = int(facelets[0].get('cont_ordered')[0][1])      # y coordinate for the top-left vertex 1st facelet
    center=(Ax,Ay)  

    frame=rotate_image(frame, center, angle)             # frame is rotated, for a better cropping & view


    #################
    # cube cropping #
    #################
    avgAx=0; avgAy=0; avgBx=0; avgBy=0
    cube_width=0
    for i in [0,3,6]:
        avgAx = int(facelets[i].get('cont_ordered')[0][0])    # avg x coordinate for the top-left vertex 'i' facelet
        avgAy = int(facelets[i].get('cont_ordered')[1][0])    # avg y coordinate for the top-left vertex 'i' facelet
        i=i+2
        avgBx = int(facelets[i].get('cont_ordered')[0][0])    # avg x coordinate for the top-left vertex 'i' facelet
        avgBy = int(facelets[i].get('cont_ordered')[1][0])    # avg y coordinate for the top-left vertex 'i' facelet
        cube_width = cube_width+int(math.sqrt((avgBy-avgAy)**2+(avgBx-avgAx)**2))

    cube_width = cube_width//3
    margin = int(0.1*cube_width)                        # 10% of cube width is used as crop's margin     

    if Ax >= margin:
        Ax = Ax-margin    # shifted coordinate
    if Ay >= margin:
        Ay = Ay-margin    # shifted coordinate
    Cx=Ax+cube_width+margin
    Cy=Ay+cube_width+margin


    # text over the cube face's images, mostly for debug purpose
#         text_x = Ax + int(0.2*(Cx-Ax))       # X coordinate for the text starting location
#         text_y = int((Ay+Cy)/2)              # Y coordinate for the text starting location
#         fontscale_coef = (Cx-Ax)/150         # coefficient to adapt the text size to almost fit the cube
#         cv2.putText(frame, str(f'Side {sides[side]}'), (text_x, text_y), font, fontScale*fontscale_coef, fontColor,lineType)

    faces[side] = frame[Ay:Cy, Ax:Cx]    # sliced image of the cube

    if side==1:
        frame_width = cube_width+2*margin
    elif side>1:
        faces[side] = cv2.resize(faces[side], (frame_width, frame_width), interpolation=cv2.INTER_LINEAR)

    return faces







def window_for_cube_rotation(w, h, side, frame):
    ''' Window on monitor to show the cube rotation (from a side to the following one)
    During these phases the Vision part doesnt search for contours
    This function is also used to
        increment the cube side variable to the next face (side)
        prevent re-capturing already detected facelets one more time, by simply re-freshing
        the frame with a new camera read.'''
    
    side+=1                         # cube side is increased
    
    if debug:    # case the debug variable is set True
        print(f'Rotate the cube to side {sides[side]}')# feedback is printed to the terminal

    # a camera reading now prevents from re-using the previous frame, therefore from re-capturing the same facelets twice
    frame, w, h = read_camera()

    show_cube_frame(frame, h, f'Next: side {sides[side]}', ['Turn the cube to show the highlighted face.'],
                     [('quit', 'Close', 'danger')], current_letter=sides[side],
                     top_button=('calibrate', 'Calibrate colors', 'secondary'))

    return side                     # return the new cube side to be anayzed







def window_for_cube_solving(solution_Text, w, h, side, frame):
    ''' This function keeps the webcam active and a window on monitor
    The window allows the user to see the Kociemba solution on screen, and to keep life the cube resolution.
    When this function is active, facelets contours aren't searched ! '''
    
    while quitting == False:
        frame, w, h=read_camera()            # video stream and frame dimensions

        if solution_Text == 'Error':
            title, instr = 'Error', ['Incoherent cube detection.', 'Recalibrate colors and rescan.']
        elif solution_Text == '0 moves  ':
            title, instr = 'The cube is solved!', ['Nothing to do.']
        else:
            title, instr = 'Solution', [solution_Text]

        show_cube_frame(frame, h, title, instr,
                         [('proceed', 'Continue', 'primary'), ('quit', 'Close', 'danger')],
                         top_button=('calibrate', 'Calibrate colors', 'secondary'))
        key=poll_key(10)                 # frame is refresched every 10ms, until keyboard/sidebar-button click

        if cv2.getWindowProperty('cube', cv2.WND_PROP_VISIBLE) <1 or (key == 27): # X on top bar or ESC button
            quit_func()                  # quitting function
            break

        if key in (ord('c'), ord('C')):  # enters interactive color calibration
            calibrate_cube_colors()

        if key == 32:               # spacebar is used to move on to the next cube's side
#             clear_terminal()        # cleares the terminal
            side=0                  # cube side is set to zero to start a new cycle
            BGR_mean.clear()        # empties the dict previoously filled with 54 facelets colors
            H_mean.clear()          # empties the dict previoously filled with 54 facelets Hue







def clear_terminal():
    ''' Removes all the text from the terminal and positions the cursors on top left.'''

    import subprocess, platform
    if platform.system()=='Windows':
        try:
            subprocess.Popen('cls', shell=True).communicate() 
        except:
            pass
        try:
            clear_output()
        except:
            pass

    else: #Linux and Mac
        print('\033c', end='')







def quit_func():
    ''' Quitting function, that properly closes stuff:
        Camera is closed
        Open cv windows are closed.'''
    
    global quitting
    
    quitting = True                  # flag when quitting process (to prevent further camera reading while quitting)
    
    try:                             # tentative
        cv2.destroyAllWindows()      # all cv2 windows are removed
        cv2.waitKey(1)               # on Windows, destroyAllWindows() needs a following waitKey to actually flush,
                                      # otherwise the last frame can stay stuck on screen looking like a frozen app
    except:                          # case of exception raised
        pass                         # do nothing
    
    try:                             # tentative
        close_camera()               # webcam is close
    except:                          # case of exception raised
        pass                         # do nothing







def camera_opened_check():
    ''' Verifies if the camera is opened (if it provides a feedback)
    Funtion returns a boolean.'''

    return camera.isOpened()         # checks if webcam is responsive or not







def close_camera():
    ''' Closes the camera object
    It's important to close the camera, if the script runs again.'''
    
    try:                             # tentative
        camera.release()             # if the program gets stuk it's because the camera remained open from previour run
    except:                          # case of exception raised
        pass                         # do nothing







def start_up(cam_num, cam_width, cam_height, cam_crop_at_right, cam_facelets):
    ''' Start up function, that aims to run (once) all the initial settings needed.'''
    
    # global variables
    global font, fontScale, fontColor, lineType, camera, width, height, sv, quitting
    global sides, side, BGR_mean, H_mean, kociemba_facelets_BGR_mean, edge, deco_edge, offset, faces, w, h, background_h
    global clear_output, first_cycle, plt, k_kernel, d_iterations, e_iterations, facelets_in_width, crop_at_right
    global solution_Text


    side=0                           # set the initial cube side (cube sides are 1 to 6, while zero is used as starting for other setting)
    first_cycle=True                 # variable to be used only at first program loop

    # webcam relevant info are returned after cropping, resizing, etc
    camera, width, height = webcam(cam_num, cam_width, cam_height)
    crop_at_right = cam_crop_at_right  # picture cropping at the right side
    edge = 14                       # edge dimension of each facelet used on cube sketches, and half-side of the color sampling box
    # deco_edge scales the on-screen Reference/Detected/Interpreted sketches (and the end-of-scan collage) so their
    # fixed layout (29*deco_edge tall) always fits the actually negotiated camera height, instead of getting silently
    # clipped by cv2 below the visible frame on shorter resolutions. It never affects color sampling (that stays 'edge').
    deco_edge = max(8, min(edge, int(height/29)))
    # scan order: white(U), green(F), yellow(D), orange(L), red(R), blue(B) - a more natural physical roll
    # (top, tilt-to-front, tilt-to-bottom, then the 3 remaining sides) than the raw kociemba U,R,F,D,L,B order.
    # reorder_to_kociemba_position() re-sorts the captured data back into kociemba's expected position order
    # once all 6 are in, so this can be any order without affecting solving correctness.
    sides={0:'Empty', 1:'U', 2:'F', 3:'D', 4:'L', 5:'R', 6:'B'}

    # general parameters for facelet's edges detection
    k_kernel=5                      # parameter for the edge detection
    d_iterations=10                 # d=Dilated (the higher the faster), for edge detection
    e_iterations=4                  # e=eroded (the larger the bigger the distance from real edge), for edge detection              
    
    # Amount of facelets in frame width, to determin the min/max acceptable contour's area
    facelets_in_width = cam_facelets

    
    quitting = False
    font, fontScale, fontColor, lineType = text_font()         # setting text font paramenters
    BGR_mean=[]                      # empty list to be filled with with 54 facelets colors while reading cube status
    H_mean=[]                        # empty list to be fille18d with with 54 facelets HUE value, while reading cube status
    kociemba_facelets_BGR_mean=[]    # empty list to be filled with with 54 facelets colors, ordered according KOCIEMBA order
    faces={}                         # dictionary that store the image of each face
    offset=int(13 * deco_edge)       # left part of the frame not usable for cube facelet detection, as used to depict the cube sketches
    solution_Text = ''               # solution_Text is set to empty string
    load_color_calibration()         # loads a previously saved color calibration from disk, if any




def rescan_one_face(letter):
    ''' Re-captures just one face (identified by its kociemba letter, e.g. 'F'), replacing its 9 entries in
    BGR_mean at their correct scan-order position - used by the post-scan review screen's click-to-rescan, so
    a single wrong face doesn't require redoing the whole 6-face scan. Reuses the same detection/accept-gate
    building blocks the main scan loop uses. Returns True once a face is accepted, False if cancelled/quit.'''

    global frame, w, h, BGR_mean

    scan_order_letters = [sides[i] for i in range(1, 7)]
    step = scan_order_letters.index(letter) + 1   # 1-based scan-step whose BGR_mean slice this letter occupies

    det_face_time = time.time()
    while True:
        key = poll_key(100)
        try:
            window_closed = cv2.getWindowProperty('cube', cv2.WND_PROP_VISIBLE) < 1
        except:
            window_closed = False
        if window_closed or key == 27:
            return False

        frame, w, h = read_camera()
        contours, hierarchy = read_facelets(det_face_time, 1, True)   # proceed=True: no countdown, capture as soon as ready

        if hierarchy is None:
            show_cube_frame(frame, h, f'Rescan side {letter}',
                             ['Hold that face inside the frame.', 'Wait for the box to turn green.'],
                             [('cancel', 'Cancel', 'danger')], current_letter=letter)
            continue

        hierarchy = hierarchy[0]
        facelets = []
        for component in zip(contours, hierarchy):
            contour, hier, corners = get_approx_contours(component)
            if corners == 4:
                facelets = get_facelets(facelets, contour, hier)
        if len(facelets) != 9:
            det_face_time = time.time()
            continue
        facelets = order_9points(facelets, new_center=[])

        candidates, new_bgr, new_hue = [], [], []
        read_color(facelets, candidates, new_bgr, new_hue)
        preview_face = [classify_bgr_to_6(bgr)[1] for bgr in new_bgr]

        retry = False
        while True:
            show_cube_frame(frame, h, f'Side {letter} rescanned',
                             ['Check the colors, then accept or retry.'],
                             [('accept', 'Accept', 'primary'), ('retry', 'Retry', 'secondary'), ('cancel', 'Cancel', 'danger')],
                             preview_face, current_letter=letter)
            key = poll_key(150)
            try:
                window_closed = cv2.getWindowProperty('cube', cv2.WND_PROP_VISIBLE) < 1
            except:
                window_closed = False
            if window_closed or key == 27:
                return False
            if key == 32:
                BGR_mean[(step-1)*9:step*9] = new_bgr    # replaces this face's slice in place, order preserved
                return True
            if key in (ord('r'), ord('R')):
                retry = True
                break

        if retry:
            det_face_time = time.time()
            continue


def review_scan():
    ''' Shown once all 6 faces are captured, before solving: a large clickable face map (each cell shows that
    face's actually captured color) lets the user rescan any face that looks wrong, rather than only being
    able to accept-or-restart-from-scratch. Loops until Accept (returns True) or Quit (returns False).'''

    global frame, w, h

    net_edge = 40   # same size used throughout scanning, so the layout doesn't visibly jump when review starts
    review_buttons = [('accept', 'Accept', 'primary'), ('quit', 'Close', 'danger')]
    while True:
        frame, w, h = read_camera()

        face_colors = {}
        for i in range(6):
            letter = sides[i+1]
            face_bgrs = BGR_mean[i*9:(i+1)*9]
            avg = tuple(int(sum(c[k] for c in face_bgrs)/9) for k in range(3))
            face_colors[letter] = classify_bgr_to_6(avg)[1]

        canvas_h = max(h, estimate_sidebar_height(['Click a face below to rescan it,', 'or Accept to continue.'],
                                                    review_buttons, None, net_edge))
        if canvas_h > h:
            pad = np.full((canvas_h-h, frame.shape[1], 3), (60, 60, 60), dtype=np.uint8)
            padded_frame = np.vstack([frame, pad])
        else:
            padded_frame = frame
        sidebar_img, rects = render_sidebar(canvas_h, font, 'Scan complete',
                                             ['Click a face below to rescan it,', 'or Accept to continue.'],
                                             review_buttons, compute_face_progress(), None, None,
                                             face_colors, clickable_net=True, net_edge=net_edge)
        compose_and_show(padded_frame, sidebar_img, rects)

        key, action = poll_action(150)
        try:
            window_closed = cv2.getWindowProperty('cube', cv2.WND_PROP_VISIBLE) < 1
        except:
            window_closed = False
        if window_closed or key == 27 or action == 'quit':
            return False
        if key == 32 or action == 'accept':
            return True
        if action and action.startswith('rescan_'):
            rescan_one_face(action[len('rescan_'):])







def cubeAF():
    ''' This function is substantially the main function.
        It covers all the different phases after the initial settings:
        1) Camera setting for 1st side and remaining
        2) Keeps interrogating the camera
        3) Cube status detection
        4) Cube solver call.'''
    
    global font, fontScale, fontColor, lineType, cap, width, height, h, w, sides, side
    global BGR_mean, H_mean, kociemba_facelets_BGR_mean, offset, frame, cube
    global facelets, faces, start_time, servo, camera, fixWindPos, solution_Text
    
    cube_color_sequence = []            # cube_color_sequence is set to empty list
    cube_status_string = ''             # cube_status_string is set to empty string
    
    if not camera_opened_check():       # case the camera is not responsive
        print('\nCannot open camera')   # feedback is printed
        quit_func()                     # script is closed
    
    frame, w, h = read_camera()         # video stream and frame dimensions
    if fixWindPos:                      # case the fixWindPos variable is set true on __main__
        # WINDOW_AUTOSIZE (cv2's default) locks the window to the frame's pixel size with no way for the
        # user to resize it, which clips off-screen on smaller/scaled displays with no way to recover. WINDOW_NORMAL
        # makes it a normal, user-resizable window instead.
        cv2.namedWindow('cube', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('cube', on_cube_mouse)   # sidebar buttons become clickable
        cv2.moveWindow('cube', 0,0)     # move the cube window to (0,0)
        try:                             # best-effort: cap the initial window size to the actual screen, so it
            import ctypes                # doesn't open larger than the display with nothing yet telling the user
            user32 = ctypes.windll.user32     # it's now draggable-resizable (Windows only; harmless no-op elsewhere)
            screen_w, screen_h = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)
            # cv2's WINDOW_NORMAL stretches whatever's shown to fill the window rect exactly (no letterboxing),
            # so this must match the ACTUAL composed canvas size show_cube_frame() will display - not the raw
            # camera height - or the image gets non-uniformly squashed/stretched to fit a mismatched window.
            # Sized for the normal scanning state (2 instruction lines, 2 buttons, no preview) rather than the
            # minimal 'Starting...' screen, so the window doesn't need to immediately resize once scanning begins.
            composed_h = max(h, estimate_sidebar_height(['line one', 'line two'], [('a', 'a', 'a'), ('b', 'b', 'b')], None))
            init_h = min(composed_h, screen_h - 80)  # leaves room for the window's title bar and the taskbar
            init_w = int((w+SIDEBAR_W) * init_h/composed_h)   # composed canvas is the camera view plus the sidebar
            cv2.resizeWindow('cube', init_w, init_h)
        except Exception:
            pass
        show_cube_frame(frame, h, 'Starting...', ['Getting the camera ready.'], [('quit', 'Close', 'danger')])
        for _ in range(5):              # lets the window actually finish opening/painting on screen
            cv2.waitKey(30)             # a freshly created window can briefly (mis)report itself as not-visible,
                                         # which the window-closed checks below would otherwise mistake for the user having clicked X
    window_open_time = time.time()      # window-closed checks are only trusted after this warm-up grace period


    if side==0:                         # side zero is used as starting phase, cube faces are numbered 1 to 6 
        faces.clear()                   # empties the dict of images (6 sides) recorded during previous solving cycle 
        show_time = 12                  # showing time of the unfolded cube images (its initial status)
        timestamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')   # date_time variable is assigned, for file name and log purpose
        start_time = time.time()        # initial time is stored
        det_face_time = time.time()     # reference time for starting faceletes detection
        proceed = False                 # proceed variable is set False (variable to jump into facelet reading mode)
        

    def scan_state_text():
        ''' Title/instructions for the sidebar, reflecting the current prepare-countdown/reading state.'''
        wait_time = int(delay-(time.time()-det_face_time))+1
        if wait_time > 0 and not proceed:
            return (f'Prepare side {sides[side]} - {wait_time}s',
                    ['Hold the cube face inside the frame.', 'Press SPACE to start early.'])
        return (f'Reading side {sides[side]}',
                ['Hold the cube face inside the frame.', 'Wait for the box to turn green.'])

    # one constant button SET for the whole scan loop (accept/retry/quit) - which ones are enabled changes by
    # state, but the set itself never does, so the sidebar layout (and window size) stays constant instead of
    # buttons appearing/disappearing and the window resizing as the loop moves between states. Calibrate lives
    # in its own separated top_button slot instead - a different kind of action from the scan flow below it.
    def scan_buttons(accept_retry_enabled):
        return [('accept', 'Accept', 'primary', accept_retry_enabled),
                ('retry', 'Retry', 'secondary', accept_retry_enabled),
                ('quit', 'Close', 'danger', True)]
    calibrate_top_button = ('calibrate', 'Calibrate colors', 'secondary')

    while quitting == False:            # substantially the main loop, it can be interrupted by quit_func()

        key = poll_key(150)             # refresh time, and time to check keyboard/sidebar-button clicks
        try:
            window_closed = time.time()-window_open_time > 0.5 and cv2.getWindowProperty('cube', cv2.WND_PROP_VISIBLE) < 1
        except:
            window_closed = False
        if window_closed:
            quit_func()
            return cube_color_sequence, cube_status_string
        if key == 32:                   # spacebar moves from preparing the cube to read facelets
            if solution_Text != 'Error':  # case the solution_Text differs from 'Error'
                proceed = True          # proceed variable is set True
        elif key == 27:                 # ESC button method to close CV2 windows
            quit_func()                 # quit function is called
            return cube_color_sequence, cube_status_string   # function is closed
        elif key in (ord('c'), ord('C')):  # enters interactive color calibration (6 colors, SPACE to sample each)
            calibrate_cube_colors()
            det_face_time = time.time()    # calibration paused the scan for a while: restart the capture delay

        frame, w, h = read_camera()     # video stream and frame dimensions

        if key in (ord('s'), ord('S')):     # saves the raw camera frame to disk, for offline debugging
            debug_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       f'debug_frame_{dt.datetime.now().strftime("%Y%m%d_%H%M%S")}.png')
            cv2.imwrite(debug_path, frame)
            print(f'\nSaved raw frame to {debug_path}')

        text_bg(frame, w, h)            # generates a rectangle as backgroung for text in Frame

        if side==0:                     # case side equals zero
            side = window_for_cube_rotation(w, h, side, frame) # keeps video stream while suggesting which cube's face to show
            
        if scan_mode == 'grid':          # case the fixed-grid scanning mode is selected
            (contours, hierarchy)=read_facelets_fixed_grid(det_face_time, delay, proceed)  # grid-aligned synthetic contours
        else:                            # case the auto-detect (contour) scanning mode is selected
            (contours, hierarchy)=read_facelets(det_face_time, delay, proceed)  # reads cube's facelets and returns the contours
        
        candidates = []                        # empties the list of potential contours
        if hierarchy is None:                  # no candidate contours this frame (e.g. no cube face found yet):
            title, instr = scan_state_text()
            show_cube_frame(frame, h, title, instr, scan_buttons(False), current_letter=sides[side], top_button=calibrate_top_button)   # still refresh the window, otherwise it keeps showing a stale frame
        else:                                   # analyze the contours in case these are previously retrieved
            hierarchy = hierarchy[0]           # only top level contours (no childs)
            facelets = []                      # empties the list of contours having cube's square characteristics

            for component in zip(contours, hierarchy):  # each contour is analyzed

                if key == 27:                  # ESC button method to close CV2 windows
                    quit_func()                # quit function is called
                    return cube_color_sequence, cube_status_string  # function is closed

                contour, hierarchy, corners = get_approx_contours(component)   # contours are approximated

                title, instr = scan_state_text()
                show_cube_frame(frame, h, title, instr, scan_buttons(False), current_letter=sides[side], top_button=calibrate_top_button)   # shows the frame
                key=poll_key(20)               # refresh time set to 20ms (real time is longher)

                # this loop can run many iterations per frame (one per candidate contour), so the window-closed
                # (X button) check needs to happen here too, not just at the top of the outer loop, otherwise a
                # click on X during a busy frame gets missed and the next imshow() call re-draws the window
                try:
                    window_closed = cv2.getWindowProperty('cube', cv2.WND_PROP_VISIBLE) < 1
                except:
                    window_closed = False
                if window_closed:
                    quit_func()                # quit function is called
                    return cube_color_sequence, cube_status_string  # function is closed

                if time.time() < det_face_time + delay: # case the delay time is not elapsed yet
                    if key == 32:              # case spacebar is pressed
                        proceed = True         # proceed is set true (from preparing the cube to read facelets)
                    if not proceed:            # cese proceed variable is False
                        break                  # for loop is interrupted 
                
                if corners==4:                 # case contours has 4 corners
                    facelets = get_facelets(facelets, contour, hierarchy) # returns a dict with cube compatible contours
                
                if len(facelets)==9:           # 9 contours have cube compatible characteristics
                    facelets = order_9points(facelets, new_center=[])  # contours are ordered from top left
                    d_to_exclude = distance_deviation(facelets, check='above') # facelets to remove due excess of inter-distance
                    if len(d_to_exclude)>=1:               # check if any contour is too far to be part of the cube
                        d_to_exclude.sort(reverse=True)    # reverse the contours order, for easier eliminations
                        for i in d_to_exclude:             # remove the contours too faar far to be part of the cube
                            facelets.pop(i)                # facelet is removed

                
                # case having 9 contours with right characteristics              
                if len(facelets)==9:
                    read_color(facelets, candidates, BGR_mean, H_mean)   # each facelet is read for color, decoration for the viewer is made
                    kociemba_facelets_BGR_mean = BGR_mean                # facelets are ordered as per kociemba order
                    faces = face_image(frame, facelets, side, faces)     # image of the cube side is taken for later reference
                    det_face_time=time.time()          # face detection time stored as reference for the next one
                    proceed = False                    # proceed is set false, fto force a delay on facelets detection at cube face changing

                    # accept/reject gate: user reviews the just-captured face before it counts. Each preview
                    # cell is also clickable ('edit_0'..'edit_8'), cycling that one facelet through the 6
                    # canonical colors in place, so a single misread doesn't force a full rescan of the face.
                    accepted = False
                    retry_face = False
                    preview_face = [classify_bgr_to_6(bgr)[1] for bgr in BGR_mean[-9:]]
                    while not accepted and not retry_face:
                        show_cube_frame(frame, h, f'Side {sides[side]} captured',
                                        ['Check the colors look right,', 'then accept or retry.'],
                                        scan_buttons(True),
                                        preview_face, current_letter=sides[side], top_button=calibrate_top_button,
                                        editable_preview=True)
                        key, action = poll_action(150)  # refresh time, and time to check keyboard/sidebar-button clicks
                        try:
                            window_closed = cv2.getWindowProperty('cube', cv2.WND_PROP_VISIBLE) < 1
                        except:
                            window_closed = False
                        if window_closed or key == 27:  # ESC button, or window closed
                            quit_func()                 # quit function is called
                            return cube_color_sequence, cube_status_string   # function is closed
                        elif key == 32:                 # spacebar accepts this face
                            accepted = True
                        elif key in (ord('r'), ord('R')):  # 'r' rejects this face, to retry it
                            retry_face = True
                        elif action is not None and action.startswith('edit_'):
                            idx = int(action[len('edit_'):])
                            cur_name = classify_bgr_to_6(BGR_mean[-9+idx])[0]
                            next_name = CYCLE_COLORS[(CYCLE_COLORS.index(cur_name)+1) % len(CYCLE_COLORS)]
                            BGR_mean[-9+idx] = DEFAULT_REF_COLORS[next_name]   # exact reference BGR: classifies back to next_name unambiguously
                            preview_face[idx] = DEFAULT_REF_COLORS[next_name]

                    if retry_face:                     # case the user asked to retry this face
                        del BGR_mean[-9:]               # this face's 9 facelets are removed from the running list
                        del H_mean[-9:]                 # same for the parallel Hue list
                        break                           # for loop is interrupted, to re-start facelet search for the same side

                    if on_face_captured is not None:   # case a callback has been provided (i.e. by the GUI)
                        on_face_captured(side, preview_face)  # the just-accepted face's 9 classified colors

                    if side < 6:  # actions when a face has been completely detected, and there still are other to come
                        show_cube_frame(frame, h, f'Side {sides[side]} accepted', ['Get the next side ready.'], scan_buttons(False), top_button=calibrate_top_button)
                        key=poll_key(20)               # delay for viewer to realize the face is aquired
                        if key == 27:                  # ESC button method to close CV2 windows
                            quit_func()                # quit function is called
                            return cube_color_sequence, cube_status_string   # function is closed
                        side = window_for_cube_rotation(w, h, side, frame) # image stream while viewer has time to positione the cube for next face
                        break                          # with this break the process re-starts from contour detection at the next cube face


                    if side == 6:  # case last cube's face is acquired

                        if not review_scan():          # lets the user rescan any face before solving; False = quit
                            quit_func()
                            return cube_color_sequence, cube_status_string
                        kociemba_facelets_BGR_mean = BGR_mean   # review may have replaced some faces' entries

                        # BGR_mean/kociemba_facelets_BGR_mean was accumulated in SCAN order (whatever order
                        # 'sides' presents the 6 faces in), but cube_colors_interpreted() assumes the fixed
                        # kociemba position order U,R,F,D,L,B (it reads specific fixed indices directly, e.g.
                        # facelet 4=U's center, 13=R's center, ...) - reorder into that fixed order so any
                        # scan order works correctly, not just one that happens to already be U,R,F,D,L,B.
                        scan_order_letters = [sides[i] for i in range(1, 7)]
                        kociemba_facelets_BGR_mean = reorder_to_kociemba_position(kociemba_facelets_BGR_mean, scan_order_letters)

                        # cube string status with colors detected
                        cube_status, HSV_detected, cube_color_sequence = cube_colors_interpreted(kociemba_facelets_BGR_mean)
                        
                        cube_status_string = cube_string(cube_status)  # cube string for the solver
                        solution, solution_Text = cube_solution(cube_status_string)   # Kociemba solver is called to have the solution string
                        color_detection_winner='BGR'      # variable used to log which method gave the solution
                        
                        if debug:                         # case the debug variable is set True
                            print(f'\nCube status (via BGR color distance): {cube_status_string}\n')

                        if solution_Text == 'Error':      # if colors interpretation on BGR color distance fail an attempt is made on HSV
                            if debug:                     # case the debug variable is set True
                                print(f'Solver return: {solution}\n')
                            
                            # cube string status with colors detected 
                            cube_status, cube_status_HSV, cube_color_sequence = cube_colors_interpreted_HSV(kociemba_facelets_BGR_mean,HSV_detected)
                            
                            cube_status_string = cube_string(cube_status)  # cube string for the solver
                            solution, solution_Text = cube_solution(cube_status_string)   # Kociemba solver is called to have the solution string
                            color_detection_winner='HSV'            # variable used to log which method give the solution
                            
                            if solution_Text == 'Error':            # in case color color detection fail also with HSV approach
                                color_detection_winner='Error'      # the winner approach goes to error, for log purpose
                            else: 
                                if debug:   # case the debug variable is set True
                                    # nice information to print at terminal, sometime useful to copy
                                    print(f'\nCube status (via HSV color distance): {cube_status_string}')
                                    print(f'\nCube solution: {solution_Text}')
                    
                        elif solution_Text != '0 moves  ':                 # case of interest, the cube isn't already solved
                            
                            if debug:       # case the debug variable is set True
                                print(f'\nCube solution: {solution_Text}') # nice information to print at terminal, sometime useful to copy 


                        if solution_Text == 'Error':   # still an error after HSV color analysis
                            show_time_ = show_time     # time to show on screen the decorative info (cube collage)
                            deco_info = (fixWindPos, frame, faces, deco_edge, cube_status, cube_color_sequence,\
                                         kociemba_facelets_BGR_mean, font, fontScale, lineType, show_time_,\
                                         timestamp, color_detection_winner)
                            
                            cv2.destroyWindow('cube')   # cube window is closed
                            decoration(deco_info)       # Cube images as seen + sketch with recognized and interpreted colors          
                            window_for_cube_solving(solution_Text, w, h, side, frame)  # image stream while user manually solves the cube
                            side = 0                    # side is set to zero, to 
                            det_face_time = time.time() # reference time for starting faceletes detection
                            break                       # foor loop is interrupted
                        
                        elif solution_Text != 'Error':  # no errors BGR or HSV color analysis
                            show_time_ = 2              # time to show on screen the decorative info (cube collage)
                            deco_info = (fixWindPos, frame, faces, deco_edge, cube_status, cube_color_sequence,\
                                         kociemba_facelets_BGR_mean, font, fontScale, lineType, show_time_,\
                                         timestamp, color_detection_winner)
                            
                            cv2.destroyWindow('cube')   # cube window is closed   
                            decoration(deco_info)       # Cube images as seen + sketch with recognized and interpreted colors
                            
                            if debug:                   # case the debug variable is set True
                                print(f'cube_status_string: {cube_status_string}') # feedback is printed to the terminal
                            return cube_color_sequence, cube_status_string         # main cube info are returned                                    
                 
                # shows the frame
                title, instr = scan_state_text()
                show_cube_frame(frame, h, title, instr, scan_buttons(False), current_letter=sides[side], top_button=calibrate_top_button)

    return cube_color_sequence, cube_status_string







def cube_status(cam_num, cam_width, cam_height, cam_crop_at_right, cam_facelets, c_debug, c_estimate_fclts, c_delay,
                 c_on_face_captured=None, c_scan_mode='auto'):
    ''' This function:
        starts up the CV part with related settings
        acquires the facelets status
        checks if the cube status is coherent
        returns the cube status if coherent
        shows the detected and interpreted colors if cube status not coherent.'''

    global debug, estimate_fclts, fixWindPos, delay, on_face_captured, scan_mode

    debug = c_debug                       # flag to enable/disable the debug related prints
    estimate_fclts = c_estimate_fclts     # flag to enable/disable the estimation on facelets position/contour
    delay = c_delay                       # delay for facelets detection at faces change
    fixWindPos = True                     # flag to fix the CV2 windows position, starting from coordinate 0,0,0
    on_face_captured = c_on_face_captured # optional callback(side, bgr9), called once a face is accepted by the user
    scan_mode = c_scan_mode               # 'auto' (contour detection) or 'grid' (fixed on-screen grid)
    try:
        start_up(cam_num, cam_width, cam_height, cam_crop_at_right, cam_facelets) # starts Webcam and other settings
        ccs, cube_status_string = cubeAF()    # cube reading/solving function returning cube color sequence an cube status
        return ccs, cube_status_string        # function return
    finally:
        quit_func()                           # always close the camera and OpenCV windows






if __name__ == '__main__':
    ''' This function is used for debug purpose, to test the webcam application.'''
    
    global debug, estimate_fclts, fixWindPos, delay
    
    debug = False #True       # flag to enable/disable the debug related prints
    estimate_fclts = True     # flag to enable/disable the estimation on facelets position/contour
    fixWindPos = True         # flag to fix the CV2 windows position, starting from coordinate 0,0
    delay = 5                 # delay time to start reading the facelets from a cube face change
    
    
    if debug:                 # case the debug variable is set True, in this __main__ function
        print('Debug prints activated')   # feedback is printed to the terminal
    
    if estimate_fclts:        # case the estimate_fclts variable is set True, in this __main__ function
        print('Facelets position estimation activated')  # feedback is printed to the terminal
    
    if fixWindPos:            # case the fixWindPos variable is set True, in this __main__ function
        print('CV2 windows forced to top-left screen corner')  # feedback is printed to the terminal
    
    print(f'Delay of {delay} to start reading the facelets after a cube face change')   
    
    print('====================================================================================\n')

    cam_num=0                 # cam 0 is tipically the integrated one
    cam_width=640             # camera width 
    cam_height=360            # camera height
    cam_crop_at_right=0       # picture cropping at the right side
    cam_facelets=11           # number of facelets per camera width, to set at wich distance facelets are detected
    
    start_up(cam_num, cam_width, cam_height, cam_crop_at_right, cam_facelets)  # starts Webcam and other settings
    ccs, cube_status_string = cubeAF()  # cube reading/solving function returning cube color sequence and cube status
    ccs=tuple(ccs)                      # cube color sequence list is convered to tuple
    quit_func()                         # quitting function is called

