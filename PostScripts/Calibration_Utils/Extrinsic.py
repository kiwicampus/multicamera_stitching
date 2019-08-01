# =============================================================================
"""
Code Information:
	Programmer: John Betancourt G.
	Phone: +57 (311) 813 7206
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision & Ai Team
sources:
    https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
"""
# =============================================================================
import numpy as np
import copy
import yaml
import cv2
import os

from Utils import dotline
from Utils import organize_points
from Utils import closest_point
from Utils import CalculateProjectionMatrix

# =============================================================================
# Mouse events 
MOUSE_CPOS = (0, 0)
MOUSE_SRC_PTS=list()
MOUSE_EVENT = 0
CALIBRATION_EVENT = False
def click_and_crop(event, x, y, flags, param):
    """ description
    Args:
        variable_name: `type` description
    Returns:
    """

    global MOUSE_CPOS, MOUSE_SRC_PTS, CALIBRATION_EVENT
    
    # If mouse pressed
    if event==1:
        if len(MOUSE_SRC_PTS)<4: MOUSE_SRC_PTS.append((x, y))
        else: 
            cl_pt=closest_point(pts=MOUSE_SRC_PTS, pt=(x, y))
            MOUSE_SRC_PTS.remove(cl_pt)
            MOUSE_SRC_PTS.append((x, y))
        if len(MOUSE_SRC_PTS)==4: CALIBRATION_EVENT=True
    elif event==2:
        if len(MOUSE_SRC_PTS):
            cl_pt=closest_point(pts=MOUSE_SRC_PTS, pt=(x, y))
            MOUSE_SRC_PTS.remove(cl_pt)
    MOUSE_CPOS = (x, y)
    MOUSE_EVENT = event

# =============================================================================
def perform_extrinsic_calibration(img_src, WIN_NAME="extrinsic_calibration"):
    """ Create a window to perform a extrinsic calibration with mouse
    Args:
        img_src: `cv2.math` source image
        WIN_NAME: `string` local window name
    Returns:
        extrinsic_calibration: `dictionary`
            M: `np.array` rotation and translation matrix to perspective transformation
            INVM: `np.array` rotation and translation inverse matrix from perspective transformation
            src_pts: `list` list of tuple with source points
            dts_pts: `list` list of tuple with destination points
            src_size: `tuple` size of source image
            dst_size: `tuple` size of destination(transformation) image
            ppmy: `float` pixel per meter relation in y axis
            ppmx: `float` pixel per meter relation in x axis
    """

    global CALIBRATION_EVENT, MOUSE_SRC_PTS
    DST_SIZE = (300, 200)

    # Create window and set mouse event
    cv2.namedWindow(WIN_NAME)
    cv2.setMouseCallback(WIN_NAME, click_and_crop)

    # Create calibration object
    extrinsic_calibration={"M":None, "INVM":None, "src_pts":None, "dts_pts":None, 
        "src_size":None, "dst_size":None, "ppmy":None, "ppmx":None}
    img_dst = None

    while True:
        
        MOUSE_SRC_PTS = organize_points(src_pts=MOUSE_SRC_PTS, src_size=(img_src.shape[1], 
            img_src.shape[0])) if len(MOUSE_SRC_PTS)==4 else MOUSE_SRC_PTS

        # Copy image and draw gui
        img_src_cp = draw_extrinsic(img_src=img_src.copy(), src_pts=MOUSE_SRC_PTS)


        if CALIBRATION_EVENT: # True when four points
            CALIBRATION_EVENT=False

            # Calculate projection matrix and find affine perspective
            src_pts = organize_points(src_pts=MOUSE_SRC_PTS, src_size=(img_src.shape[1], img_src.shape[0])) 
            dst_pts = [(0, 0), (DST_SIZE[0], 0), (DST_SIZE[0], DST_SIZE[1]), (0, DST_SIZE[1])]
            M, INVM =CalculateProjectionMatrix(src_pts=src_pts, dst_pts=dst_pts)
            img_dst = cv2.warpPerspective(src=img_src.copy(), M=M, dsize=DST_SIZE)

            # Re-assing parameters
            extrinsic_calibration["M"]=M
            extrinsic_calibration["INVM"]=INVM
            extrinsic_calibration["src_pts"]=src_pts
            extrinsic_calibration["dts_pts"]=dst_pts
            extrinsic_calibration["src_size"]=(img_src.shape[1], img_src.shape[0])
            extrinsic_calibration["dst_size"]=DST_SIZE
            extrinsic_calibration["ppmy"]=None
            extrinsic_calibration["ppmx"]=None
            
        # Show image and capture user key
        cv2.imshow(WIN_NAME, img_src_cp)
        if img_dst is not None: cv2.imshow(WIN_NAME+"_projection", img_dst)
        key = cv2.waitKey(10)

        if key==113: # (Q) If pressed then break while
            break
        elif key!=-1:  # No key command
            print("Command or key action no found: {}".format(key))
        cv2.destroyWindow(winname=WIN_NAME+"_projection")

    return extrinsic_calibration

def draw_extrinsic(img_src, src_pts):
    """ draws points and geometries in image 
    Args:
        img_src: `cv2.math` image to draw components
        src_pts: `list` of spacial transform
    Returns:
        img_src: `cv2.math` image with components drawn
    """

    # Draw current mouse position
    dotline(src=img_src, p1=(MOUSE_CPOS[0], 0), p2=(MOUSE_CPOS[0], img_src.shape[0]), 
        color=(0, 0, 255), thickness=2, Dl=5)
    dotline(src=img_src, p1=(0, MOUSE_CPOS[1]), p2=(img_src.shape[1], MOUSE_CPOS[1]), 
        color=(0, 0, 255), thickness=2, Dl=5)

    # Draw contour
    if len(src_pts)>1:
        cv2.drawContours(image=img_src, contours=np.array([src_pts]), 
            contourIdx=-1, color=(0, 255, 0), thickness=1)

    # Draw click points
    for pt in src_pts:
        cv2.circle(img=img_src, center=tuple(pt), radius = 10, 
            color = (255, 0 ,0), thickness = 2)
        cv2.circle(img=img_src, center=tuple(pt), radius = 2, 
            color = (0, 0 ,255), thickness = -1)

    if len(src_pts)==4:
        dotline(src=img_src, p1=(0, 0), p2=src_pts[0], 
            color=(255, 255, 255), thickness=1, Dl=5)
        dotline(src=img_src, p1=(img_src.shape[1], 0), p2=src_pts[1], 
            color=(255, 255, 255), thickness=1, Dl=5)
        dotline(src=img_src, p1=(img_src.shape[1], img_src.shape[0]), p2=src_pts[2], 
            color=(255, 255, 255), thickness=1, Dl=5)
        dotline(src=img_src, p1=(0, img_src.shape[0]), p2=src_pts[3], 
            color=(255, 255, 255), thickness=1, Dl=5)

    return img_src

def save_extrinsic_calibration(file_path, file_name, extrinsic_calibration):
    """ saves extrinsic calibration in file
    Args:
        file_path: `string` path to save file
        file_name: `string` name of the extrinsic calibration file
            M: `np.array` rotation and translation matrix to perspective transformation
            INVM: `np.array` rotation and translation inverse matrix from perspective transformation
            src_pts: `list` list of tuple with source points
            dts_pts: `list` list of tuple with destination points
            src_size: `tuple` size of source image
            dst_size: `tuple` size of destination(transformation) image
            ppmy: `float` pixel per meter relation in y axis
            ppmx: `float` pixel per meter relation in x axis
    Returns:
    """
    
    # Absolute path to file
    abs_path = os.path.join(file_path, file_name)

    # Prepare data to save
    data = dict(
        M=extrinsic_calibration["M"].tolist(), 
        INVM=extrinsic_calibration["INVM"].tolist(),
        src_pts=extrinsic_calibration["src_pts"],
        dts_pts=extrinsic_calibration["dts_pts"],
        src_size=extrinsic_calibration["src_size"],
        dst_size=extrinsic_calibration["dst_size"],
        ppmy=extrinsic_calibration["ppmy"],
        ppmx=extrinsic_calibration["ppmx"],
        )

    try: # Save the calibration data in file
        with open(abs_path, 'w') as outfile: 
            yaml.dump(data, outfile, default_flow_style=False)
        print("[INFO][EXTRINSIC]: ({}) Camera extrinsic calibration saved at {}".format(
            file_name, file_path)) 
    except IOError as e: # Report any error saving de data
        print("[ERROR][EXTRINSIC]: Problem saving camera extrinsic calibration: {}".format(e))

def load_extrinsic_calibration(abs_path):
    """ Loads extrinsic parameters from file
    Args:
        abs_path: `string` absolute path where calibration file will be saved
    Returns:
            M: `np.array` rotation and translation matrix to perspective transformation
            INVM: `np.array` rotation and translation inverse matrix from perspective transformation
            src_pts: `list` list of tuple with source points
            dts_pts: `list` list of tuple with destination points
            src_size: `tuple` size of source image
            dst_size: `tuple` size of destination(transformation) image
            ppmy: `float` pixel per meter relation in y axis
            ppmx: `float` pixel per meter relation in x axis
    """

    try: # load the calibration data in file
        with open(abs_path) as f:
            data_loaded = yaml.load(f)
        M = np.array(data_loaded["M"])
        INVM = np.array(data_loaded["INVM"])
        src_pts = data_loaded["src_pts"]
        dts_pts = data_loaded["dts_pts"]
        src_size = data_loaded["src_size"]
        dst_size = data_loaded["dst_size"]
        ppmy = data_loaded["ppmy"]
        ppmx = data_loaded["ppmx"]
        print("[INFO][EXTRINSIC]: Camera extrinsic calibration loaded")

    except IOError as e: # Report any error saving de data
        print("[ERROR][EXTRINSIC]: Problem loading camera extrinsic calibration: {}".format(
            (os.path.basename(abs_path))))
        return {"M":None, "INVM":None, "src_pts":None, "dts_pts":None, 
        "src_size":None, "dst_size":None, "ppmy":None, "ppmx":None}

    return {"M":M, "INVM":INVM, "src_pts":src_pts, "dts_pts":dts_pts, 
        "src_size":src_size, "dst_size":dst_size, "ppmy":ppmy, "ppmx":ppmx}

# =============================================================================
if __name__ == '__main__':
    
    # Perform extrinsic calibration from image gallery
    extrinsic_calibration=perform_extrinsic_calibration(img_src=cv2.imread("sample.jpg"))
    
    # Saves extrinsic calibration from image gallery
    save_extrinsic_calibration(file_path="", file_name="extrinsic.yaml", 
                extrinsic_calibration=extrinsic_calibration)

    # Loads extrinsic calibration from file
    load_extrinsic_calibration("extrinsic.yaml")

