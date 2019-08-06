# =============================================================================
"""
Code Information:
	Programmer: John Betancourt G.
	Phone: +57 (311) 813 7206
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision & Ai Team
sources:
Description:
    Utile functions for almost all processes
"""

# =============================================================================
# LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPEN
# =============================================================================
from __future__ import print_function
import numpy as np
import math
import cv2
import sys

# =============================================================================
def get_projection_point_dst(pt_src, M):
    """  Gets the coordinate equivalent in surface projection space from original 
         view space 
    Args:
        coords_src: `numpy.darray`  coordinate in the original image space
        M: `numpy.darray` rotation matrix 
    Returns:
        coords_src: `numpy.darray`  projected coordinate in original view space
    """

    pt_dst = np.matmul(M, pt_src)
    pt_dst = pt_dst / pt_dst[2]
    pt_dst = [int(pt_dst[0]), int(pt_dst[1])]

    return pt_dst

def get_projection_point_src(coords_dst, INVM):
    """  Gets the coordinate equivalent in original view space from surface 
         projection space
    Args:
        coords_src: `numpy.darray`  coordinate in the original image space
        INVM: `numpy.darray` inverse rotation matrix 
    Returns:
        coords_src: `numpy.darray`  projected coordinate in original view space
    """

    # Calculate new coordinate
    coords_src = np.matmul(INVM, coords_dst)
    coords_src = coords_src / coords_src[2]
    coords_src = [int(coords_src[0]), int(coords_src[1])]

    return coords_src

def get_undistor_point(pt, mtx, dist):
    """  Get the coordinate pt in undistortion image
    Args:
        pt: `numpy.darray`  point in original image
        mtx: `numpy.narray` camera's distortion matrix
        dist: `numpy.narray` camera's distortion vector
    Returns:
        Undistorted point
    """

    test = np.zeros((1,1,2), dtype=np.float32)
    test[0,0,0]=pt[0]
    test[0,0,1]=pt[1]

    xy_undistorted = cv2.undistortPoints(test, mtx, dist)

    x_undistor = xy_undistorted[0,0,0]* mtx[0,0] + mtx[0,2];  
    y_undistor = xy_undistorted[0,0,1]* mtx[1,1] + mtx[1,2];  

    return int(x_undistor), int(y_undistor)

def get_distor_point(pt, mtx, dist):
    """  Gets the coordinate equivalent in original view space from surface 
         projection space
    Args:
        pt: `numpy.darray`  point in undistorted image
        mtx: `numpy.narray` camera's distortion matrix
        dist: `numpy.narray` camera's distortion vector
    Returns:
        distor_point: `numpy.darray` coordinate in image with distortion
    http://answers.opencv.org/question/148670/re-distorting-a-set-of-points-af
    ter-camera-calibration/
    """
    test = np.zeros((1, 1, 2), dtype = np.float32)
    test[0,0,0]=pt[0]
    test[0,0,1]=pt[1]

    rtemp = ttemp = np.array([0, 0, 0], dtype = 'float32')

    # Normalize the points to be independent of the camera matrix using undistorted points with no distortion matrix
    xy_normalized = cv2.undistortPoints(test, mtx, None)

    # Convert them to 3d points 
    ptsTemp = cv2.convertPointsToHomogeneous(xy_normalized)

    # Project them back to image space using the distortion matrix
    output = cv2.projectPoints(ptsTemp, rtemp, ttemp, mtx, dist, xy_normalized)

    x_undistor = output[0][0, 0, 0]
    y_undistor = output[0][0, 0, 1]

    distor_point = int(round(x_undistor)), int(round(y_undistor))

    return distor_point

def CalculateProjectionMatrix(src_pts, dst_pts):  
    """  
    Args:
    Returns:
    """
  
    # Calculate rotation matrix from surface from original source image to 
    # projected four points surfaces
    src_points = np.array(src_pts, dtype=np.float32) 
    dst_points = np.array(dst_pts, dtype=np.float32)
    M = cv2.getPerspectiveTransform(src=src_points, dst=dst_points)
    INVM = np.linalg.inv(M)

    return M, INVM

def dotline(src, p1, p2, color, thickness, Dl):
    """  draws a doted line on input image
    Args:
        src: `cv2.mat` source image
        p1: `tuple` line's first point [pix, pix]
        p2: `tuple` line's second point [pix, pix]
        color: `tuple` lines' color RGB [B, G, R] [int]
        thickness: `int` lines' thickness [pix]
        Dl: `int` distance in pixels between every point
    Returns:
        src: `cv2.mat` image with doted line drawn
    """

    # Get a number of intermediate points
    segments = discrete_contour((p1, p2), Dl)

    # Draw doted line 
    for segment in segments:
        cv2.circle(src, segment, thickness, color, -1) 

    # Return result
    return src

def discrete_contour(contour, Dl):
    """  Takes contour points to get a number of intermediate points
    Args:
        contour: `List` contour or list of points to get intermediate points
        Dl: `int` distance to get a point by segment
    Returns:
        new_contour: `List` new contour with intermediate points
    """

    # If contour has less of two points is not valid for operations
    if len(contour) < 2:
        print("Error: no valid segment")
        return contour

    # New contour variable
    new_contour = []

    # Iterate through all contour points
    for idx, cordinate in enumerate(contour):

        # Select next contour for operation
        if not idx == len(contour)-1:
            next_cordinate = contour[idx+1]
        else:
            next_cordinate = contour[0]

        # Calculate length of segment
        segment_lenth = math.sqrt((next_cordinate[0] - cordinate[0])**2 +\
                        (next_cordinate[1] - cordinate[1])**2)
        
        divitions = segment_lenth/Dl # Number of new point for current segment
        dy = next_cordinate[1] - cordinate[1]    # Segment's height
        dx = next_cordinate[0] - cordinate[0]    # Segment's width
        
        if not divitions:
            ddy = 0 # Dy value to sum in Y axis
            ddx = 0 # Dx value to sum in X axis
        else:
            ddy = dy/divitions  # Dy value to sum in Y axis
            ddx = dx/divitions  # Dx value to sum in X axis
        
        # get new intermediate points in segments
        for idx in range(0, int(divitions)):
            new_contour.append((int(cordinate[0] + (ddx*idx)), 
                                int(cordinate[1] + (ddy*idx))))    

    # Return new contour with intermediate points
    return new_contour

def organize_points(src_pts, src_size):
    """ Organizes points in the next order p1 as left superior, p2 right superior
        p3 as right inferior and p4 as left inferior
    Args:
        src_pts: `list`  list of four points (X, Y)
        src_size: `tuple` Original video streaming size
    Returns:
        src_pts: `list`  sorted points as p1, p2, p3, and p4
    """

    p1, p2, p3, p4 = src_pts

    # for Left and superior  surface projection  point
    dmin = max(src_size[0], src_size[1])
    for pt in src_pts:
        d = math.sqrt((pt[0]-0)**2 + (pt[1])**2)
        if d < dmin:
            p1 = pt
            dmin = d

    # Right and superior  surface projection  point
    dmin = max(src_size[0], src_size[1])
    for pt in src_pts:
        d = math.sqrt((pt[0]-src_size[0])**2 + (pt[1])**2)
        if d < dmin:
            p2 = pt
            dmin = d

    # Right and Inferior  surface projection  point
    dmin = max(src_size[0], src_size[1])
    for pt in src_pts:
        d = math.sqrt((pt[0]-src_size[0])**2 + (pt[1]-src_size[1])**2)
        if d < dmin:
            p3 = pt
            dmin = d

    # Left and Inferior  surface projection  point
    dmin = max(src_size[0], src_size[1])
    for pt in src_pts:
        d = math.sqrt((pt[0]-0)**2+(pt[1] - src_size[1])**2)
        if d < dmin:
            p4 = pt
            dmin = d

    # Result to return
    src_pts = [p1, p2, p3, p4]

    return src_pts

def closest_point(pts, pt):
    """ Return the closet point to 'pt' in the list of points 'pts'
    Args:
        pts: `list`  list of tuples with points (X, Y)
        pt: `tuple` point to find closest point in list of points
    Returns:
        cl_pt: `pt`  closest point to pt in list of points
    """

    cl_pt = None
    min_dist = sys.maxint
    for pts_pt in pts:
        dist=np.sqrt(pow(pts_pt[0]-pt[0], 2)+pow(pts_pt[1]-pt[1], 2))
        if dist<min_dist:
            min_dist=dist
            cl_pt=pts_pt

    return cl_pt 

def print_list_text(img_src, str_list, origin=(0, 0), color=(0, 255, 255), 
    line_break=20, thickness=2, left_origin=False, fontScale=0.45):

    for idx, strprint in enumerate(str_list):
        cv2.putText(img = img_src,
                    text = strprint,
                    org = (origin[0], origin[1] + (line_break * idx)),
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX, 
                    fontScale = fontScale, 
                    color = (0, 0, 0), 
                    thickness = thickness+3, 
                    lineType = cv2.LINE_AA,
                    bottomLeftOrigin = left_origin)
        cv2.putText(img = img_src,
                    text = strprint,
                    org = (origin[0], origin[1] + (line_break * idx)),
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX, 
                    fontScale = fontScale, 
                    color = color, 
                    thickness = thickness, 
                    lineType = cv2.LINE_AA,
                    bottomLeftOrigin = left_origin)

    return img_src

# =============================================================================