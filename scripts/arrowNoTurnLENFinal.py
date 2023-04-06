#!/usr/bin/env python3
import cv2 as cv
import numpy as np
from coordinate_finder import Camera
from swarm.srv import FireData
import rospy

def take_distance(tup1, tup2):
    return np.sqrt((tup1[0]-tup2[0])**2 + (tup1[1]-tup2[1])**2)

def angle(pnt1, pnt2):
    """
    DEPRECATED

    Hard coded. Takes the angle between the line of two points and horizontal line, pnt1 is
    thought of as the arrow mass center and pnt2 is thought of as the hull mass center. By calculating the angle
    between pnt2-pnt1 and positve x and taking the angle in counter clockwise direction we obtain the angle of
    rotation we need.

    :param pnt1: the mass center of the arrow
    :param pnt2: the mass center of the hull
    :return: angle between the lines pnt2-pn1 and +x in radians
    """

    if ((pnt1[0] > pnt2[0]) and (pnt1[1] > pnt2[1])):
        return np.arctan((pnt1[1] - pnt2[1])/(pnt1[0] - pnt2[0]))
    elif ((pnt1[0] < pnt2[0]) and (pnt1[1] > pnt2[1])):
        return np.pi - np.arctan((pnt1[1] - pnt2[1])/(pnt2[0] - pnt1[0]))
    elif ((pnt1[0] < pnt2[0]) and (pnt1[1] < pnt2[1])):
        return np.arctan((pnt2[1] - pnt1[1])/(pnt2[0] - pnt1[0]))+np.pi
    elif ((pnt1[0] > pnt2[0]) and (pnt1[1] < pnt2[1])):
        return 2*np.pi - np.arctan((pnt2[1] - pnt1[1])/(pnt1[0] - pnt2[0]))
    elif ((pnt1[0] > pnt2[0]) and (pnt1[1] == pnt2[1])):
        return 0
    elif ((pnt1[0] < pnt2[0]) and (pnt1[1] == pnt2[1])):
        return np.pi
    elif ((pnt1[0] == pnt2[0]) and (pnt1[1] > pnt2[1])):
        return np.pi/2
    elif ((pnt1[0] == pnt2[0]) and (pnt1[1] < pnt2[1])):
        return 3*np.pi/2

def line_finder(pnt_arrow, pnt_hull):
    """
    A function to find slope-intercept representation of the line constituted by convex hull's
    center of mass and contours center of mass
    :param pnt_arrow: mass center of contour in tuple of form (x, y)
    :param pnt_hull: mass center of convex hull in tuple of form (x, y)
    :return: returns m (slope of the line) b (y-intercept)
    """

    m = (pnt_arrow[1]-pnt_hull[1])/(pnt_arrow[0]-pnt_hull[0])
    b = pnt_arrow[1] - m*pnt_arrow[0]
    return m, b

def find_arrow(frame, degrees=1):

    """
    A function that take a frame, uses grayscale thresholding to mask arrow, 
    uses contour arond arrow and convex hull of the contour to find mass centers,
    then uses these mass centers to find angle between line of two mass centers
    and horizontal, tip of the arrow and base of the arrow, and returns these values
    
    :param frame: frame to be analyzed
    :param degrees: 1=angle in degrees, 0=angle in radians
    :return: angle in degrees, mass center of convex hull, tip of the arrow in tuple of form (x,y), and base of the arrow in tuple of form (x, y)
    """

    #masking arrow in gray is easier and more efficient
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    #create a mask with the upper and lower bounds of the arrow shape, (USED GIMP TO FIND BOUNDS)
    lower_white = 205
    upper_white = 255
    mask_white = cv.inRange(gray, lower_white, upper_white)
    
    #1 iteration of erotion and dilation to get rid of small points
    kernel = np.ones((4, 4), dtype=np.uint8)
    eroded = cv.erode(mask_white, kernel, iterations=1)
    eroded = cv.dilate(eroded, kernel, iterations=1)
    
    #create the mask
    frame = cv.bitwise_and(frame, frame, mask=eroded)

    #find contours around the arrow, take the one with max area, and take convex hull of that contour
    contours,hierarchy = cv.findContours(mask_white, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cnt = max(contours, key=cv.contourArea)
    hull = cv.convexHull(cnt)
    

    """
    center of mass of the arrow is always more close to the tip of the arrow compared to 
    the center of mass of the hull. Taking the angle between the line of these two points and 
    the horizontal line we can rotate the image to always have the arrow facing right.
    """

    #take the mass center of the arrow
    M = cv.moments(cnt)
    mass_center_arrow_int = (np.uint16(M["m10"] / M["m00"]), np.uint16(M["m01"] / M["m00"]))
    mass_center_arrow = (M["m10"] / M["m00"], M["m01"] / M["m00"])


    #take the mass center of the hull
    M = cv.moments(hull)
    mass_center_hull_int = (np.uint16(M["m10"] / M["m00"]), np.uint16(M["m01"] / M["m00"]))
    mass_center_hull = (M["m10"] / M["m00"], M["m01"] / M["m00"])

    m, b = line_finder(mass_center_arrow, mass_center_hull)
    func_tip = lambda x: (x[0][1]-m*x[0][0]-b)**2
    tip = min(hull, key=func_tip)[0]
    
    func_base = lambda x: ((tip[0]-x[0][0])**2+(tip[1]-x[0][1])**2)
    base = max(hull, key=func_base)[0]

    # calculate the angle between two points and the horizontal line. Opencv wants the angle in degrees.
    angle = 1

    return angle, mass_center_hull_int, tip, base

def find_fire(frame, mass_center):
    blured = cv.blur(frame, (9, 9))
    hsv = cv.cvtColor(blured, cv.COLOR_BGR2HSV)
    # blue mask
    lower_red1 = np.array([160, 50, 50])
    upper_red1 = np.array([180, 255, 255])

    # red mask
    lower_red2 = np.array([0, 50, 50])
    upper_red2 = np.array([20, 255, 255])

    # create masks
    mask_red1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv.inRange(hsv, lower_red2, upper_red2)
    mask = cv.bitwise_or(mask_red1, mask_red2)
    
    kernel = np.ones((7, 7), dtype=np.uint8)
    eroded = cv.erode(mask, kernel, iterations=1)
    dilated = cv.dilate(eroded, kernel, iterations=5)

    contours, _ = cv.findContours(dilated, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    cv.drawContours(frame, contours, -1, [0, 0, 255], 3)
    
    cnt = max(contours, key=cv.contourArea)
    M = cv.moments(cnt)
    mass_center_fire_int = (np.uint16(M["m10"] / M["m00"]), np.uint16(M["m01"] / M["m00"]))
    mass_center_fire = (M["m10"] / M["m00"], M["m01"] / M["m00"])

    fire_min_y = min(cnt, key=lambda x: x[0][1])[0, 1]
    fire_max_y = max(cnt, key=lambda x: x[0][1])[0, 1]//10*10
    fire_min_x = min(cnt, key=lambda x: x[0][0])[0, 0]
    fire_max_x = max(cnt, key=lambda x: x[0][0])[0, 0]//10*10
    fire_min_y_off = (fire_min_y-mass_center[1])//10*10
    fire_min_x_off = (fire_min_x-mass_center[0])//10*10
    minboxno = (fire_min_x_off//10 + 1, fire_min_y_off//10 - 1)
    bounds = (fire_min_x//10*10, fire_max_x, fire_min_y//10*10, fire_max_y)

    height, width, _ = frame.shape

    cv.line(frame, (fire_min_x, height), (fire_min_x, 0), (255, 0, 0), 3)
    cv.line(frame, (fire_max_x, height), (fire_max_x, 0), (255, 0, 0), 3)
    cv.line(frame, (width, fire_min_y), (0, fire_min_y), (255, 0, 0), 3)
    cv.line(frame, (width, fire_max_y), (0, fire_max_y), (255, 0, 0), 3)
    print(minboxno) 
    fire_frame = dilated[bounds[2]:bounds[3], bounds[0]:bounds[1]]

    return mass_center_fire_int, minboxno, fire_frame, bounds

offset = 10
def boxes_finder(img, frame, minboxno):
    height, width = frame.shape
    LEN_BOX = 20
    max_box_x = width//LEN_BOX
    max_box_y = height//LEN_BOX
    num_boxes = (max_box_x, max_box_y)
    list_boxes = []
    bool_grid = np.zeros((max_box_y+offset, max_box_x+offset), dtype=np.uint8)

    for iii in range(max_box_y):
        for jjj in range(max_box_x):
            box = frame[iii*LEN_BOX:iii*LEN_BOX+LEN_BOX, jjj*LEN_BOX:jjj*LEN_BOX+LEN_BOX]
            if (255 in box):
                list_boxes.append((jjj+minboxno[0]+2, iii+minboxno[1]+2))
                bool_grid[iii+offset//2,jjj+offset//2] = 1
                cv.circle(img, (jjj*LEN_BOX+20+318+minboxno[0]*10, iii*LEN_BOX+20+140+minboxno[1]*10), 1, (0, 0, 0), -1)
    #array_bool = np.array(dtype=)
    return bool_grid, list_boxes, num_boxes

if __name__ == "__main__":
    rospy.init_node("camera")
    #take image/video path and distance from tip to base of arrow from console
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-i", "--image", help = "path to the image file")
    # ap.add_argument("-d", "--distance", help = "distance in centimers from the base to tip")
    # args = vars(ap.parse_args())
    # img_path = args["image"]
    # dist = float(args["distance"])

    int_old = np.array([[888.62458621,  0.,             406.78936403],
                        [0.,            883.3167022,    279.32739954],
                        [0.,            0.,             1.]])
    int_new = np.array([[902.24310303,      0.,            408.50777145],
                         [0.,               886.68945312,  276.85191197],
                         [0.,               0.,            1.        ]])
    distor = np.array([[ 0.01671327,  0.35190008, -0.00838681,  0.00533049,  0.04647637]])
    roi = (8, 9, 783, 580)
    specified_camera=Camera(int_old, int_new, distor, roi)

    cam = cv.VideoCapture("/dev/video2")
    ret, img = cam.read()
    
    angle, mass_center, tip, base = find_arrow(img)
    
    #take fire place's point in original image and then rotate it 
    fire, minboxno, fire_frame, bounds = find_fire(img, mass_center)
    
    bool_grid, list_boxes, num_boxes = boxes_finder(img, fire_frame, minboxno)
    bool_grid = np.array(bool_grid)
    bool_grid = abs(1-bool_grid)
    print(bool_grid)
    bool_grid = bool_grid.flatten()
    bool_grid = bool_grid.tolist()
    

    m, b = line_finder(tip, mass_center)
    m_ort = -1/m
    b_ort = mass_center[1] - m_ort*mass_center[0]

    height, width, _ = img.shape
    linepnt1 = (width, int(m*width + b))
    linepnt2 = (0, int(b))
    linepnt3 = (int(height/m_ort-b_ort/m_ort), height)
    linepnt4 = (int(-b_ort/m_ort), 0)

    coordinate_mass_center = specified_camera.find_coord(0, 0, 240, mass_center[0], mass_center[1], 0, 0, 0)
    coordinate_tip = specified_camera.find_coord(0, 0, 240, tip[0], tip[1], 0, 0, 0)
    dist_tip_mass = take_distance(coordinate_mass_center, coordinate_tip)
    print(dist_tip_mass)
    coordinate_topleft = specified_camera.find_coord(0, 0, 240, bounds[0], bounds[2], 0, 0, 0)
    coordinate_topright = specified_camera.find_coord(0, 0, 240, bounds[1], bounds[2], 0, 0 ,0)
    dist_top_left_right = take_distance(coordinate_topleft, coordinate_topright)
    
    coordinate_bottomleft = specified_camera.find_coord(0, 0, 240, bounds[0], bounds[3], 0, 0 ,0)
    startx, starty = np.array(coordinate_topleft) - np.array(coordinate_mass_center)
    width_ros = take_distance(coordinate_topleft, coordinate_topright)
    height_ros = take_distance(coordinate_bottomleft, coordinate_topleft)

    ros_list = (startx, starty, width_ros, height_ros, num_boxes[0], num_boxes[1], bool_grid)
    
    pixel = dist_top_left_right/(bounds[1]-bounds[0])
    print(f"pixel = {pixel}")

    rospy.wait_for_service("firedata")
    client=rospy.ServiceProxy("firedata", FireData)
    client.call(startx, starty, width_ros+(offset*pixel*20), height_ros+(offset*pixel*20), num_boxes[0]+offset, num_boxes[1]+offset, bool_grid)
    
#[[0, 3, 0], [2.0, 4.399999999999999, 0], [5.0, 5.0, 0], [7.0, 5.0, 0], [7.0, 2.0, 0]]
