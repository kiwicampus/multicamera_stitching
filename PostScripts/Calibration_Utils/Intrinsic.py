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
import yaml
import cv2
import os

# =============================================================================
def perform_intrinsic_calibration(abs_path="./Gallery", n_x=6, n_y=4):
    """ From chess board pictures gallery find the coefficient and distortion of 
        the camera and other calibration parameters
    Args:
        abs_path: `string` absolute path where image for calibration are saved
        n_x: `int` number rows of the chessboard used to calibrate
        n_y: `int` number columns of the chessboard used to calibrate
    Returns:
        intrinsic_calibration: `dictionary` intrinsic calibration variables
            ret: `float` description
            mtx: `numpy.ndarray` Output 3x3 floating-point camera matrix 
            dist: `numpy.ndarray` Output vector of distortion coefficients (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements
            rvecs: `list` Output vector of rotation vectors 
            tvecs: `list` Output vector of translation vectors estimated for each pattern view.
            newcameramtx: `numpy.ndarray` Output new camera matrix.
            roi: `tuple` Image size after rectification. By default,it is set to imageSize
            img_width: `int` images width
            img_height: `int` images height
    """
    
    # setup object points variables
    objp = np.zeros((n_y * n_x, 3), np.float32)
    objp[:, :2] = np.mgrid[0:n_x, 0:n_y].T.reshape(-1, 2)
    image_points = [];  object_points = []; fail_boards = 0 
    win_name = 'Chess_boards'

    # loop through images in provided absolute path 
    print("[INFO][INTRINSIC]: Finding chess boards")
    for idx, image_file in enumerate(os.listdir(abs_path)):
        if image_file.endswith("jpg"):

            # read images and turn them in gray scale
            img_gray = cv2.cvtColor(cv2.imread(os.path.join(abs_path, image_file)), 
                cv2.COLOR_RGB2GRAY)   

            # Find chess boards in image
            found, corners = cv2.findChessboardCorners(image=img_gray, 
                patternSize=(n_x, n_y)) 

            if found: # If chess board was found in image

                # Shows chess board corner detection
                img_chess = cv2.drawChessboardCorners(image=img_gray, 
                    patternSize=(n_x, n_y), corners=corners, patternWasFound=found)
                            
                # Show detected chess boards 
                cv2.imshow(winname=win_name, mat=img_chess)
                cv2.waitKey(100)                              

                # make fine adjustments to the corners so higher precision can
                #  be obtained before appending them to the list
                corners2 = cv2.cornerSubPix(image=img_gray, corners=corners, 
                    winSize=(11, 11), zeroZone=(-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + 
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)) 
                
                # Include chess board corners to list
                image_points.append(corners2)
                
                # Include chess board objects to list
                object_points.append(objp)    

            else : # If chess board was not found from picture                       
                fail_boards+=1     

    # Calibration parametres
    ret, mtx, dist, rvecs, tvecs, newcameramtx, roi, img_width, img_height = [None]*9
    
    print("[INFO][INTRINSIC]: {} chess boards found of {}".format(idx-fail_boards, idx))
    if len(object_points): # If corners intersection were found then calibrate

        print("[INFO][INTRINSIC]: Performing calibration please wait ...")
        # perform the calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPoints=object_points,
            imagePoints=image_points, imageSize=img_gray.shape[::-1], 
            distCoeffs=None, cameraMatrix=None)

        # Find optimum camera matrix to Undistorted images
        img_height,  img_width = img_chess.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix=mtx, 
            distCoeffs=dist, imageSize=(img_width, img_height), alpha=1, 
            newImgSize=(img_width, img_height))

        """
        Re-projection error gives a good estimation of just how exact is the 
        found parameters. This should be as close to zero as possible. Given the
        intrinsic, distortion, rotation and translation matrices, we first transform
        the object point to image point using cv2.projectPoints(). Then we calculate
        the absolute norm between what we got with our transformation and the corner
        finding algorithm. To find the average error we calculate the arithmetical
        mean of the errors calculate for all the calibration images.
        """
        mean_error = 0; tot_error=0
        for i in xrange(len(object_points)):
            imgpoints2, _ = cv2.projectPoints(object_points[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.normalize(object_points[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error
        print("[INFO][INTRINSIC]: Total error: {}".format(mean_error/float(len(object_points))))
        print("[INFO][INTRINSIC]: Camera calibrated") 
    else:
        print("[ERROR][INTRINSIC]: Camera calibration no found, check your gallery")

    # Destroy window
    cv2.destroyWindow(win_name)

    return {"ret":ret, "mtx":mtx, "dist":dist, "rvecs":rvecs, "tvecs":tvecs, 
        "newcameramtx":newcameramtx, "roi":roi, "img_width":img_width, 
        "img_height":img_height}

def save_intrinsic_calibration(dest_path, intrinsic_calibration):
    """ Saves intrinsic parameters in file
    Args:
        intrinsic_calibration: `dictionary` intrinsic calibration variables
            ret: `float` description
            mtx: `numpy.ndarray` Output 3x3 floating-point camera matrix 
            dist: `numpy.ndarray` Output vector of distortion coefficients (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements
            rvecs: `list` Output vector of rotation vectors 
            tvecs: `list` Output vector of translation vectors estimated for each pattern view.
            newcameramtx: `numpy.ndarray` Output new camera matrix.
            roi: `tuple` Image size after rectification. By default,it is set to imageSize
            img_width: `int` images width
            img_height: `int` images height
    Returns:
    """

    # Prepare data to save
    data = dict(
        ret=intrinsic_calibration["ret"], 
        mtx=intrinsic_calibration["mtx"].tolist(), 
        dist=intrinsic_calibration["dist"].tolist(), 
        roi=intrinsic_calibration["roi"], 
        rvecs=np.asarray(intrinsic_calibration["rvecs"]).tolist(),
        tvecs=np.asarray(intrinsic_calibration["tvecs"]).tolist(), 
        newcameramtx=intrinsic_calibration["newcameramtx"].tolist(), 
        img_width=intrinsic_calibration["img_width"], 
        img_height=intrinsic_calibration["img_height"])

    try: # Save the calibration data in file
        file_name = 'cam_intrinsic_{}X{}.yaml'.format(intrinsic_calibration["img_width"], 
            intrinsic_calibration["img_height"])
        with open(os.path.join(dest_path, file_name), 'w') as outfile: 
            yaml.dump(data, outfile, default_flow_style=False)
        print("[INFO][INTRINSIC]: ({}) Camera intrinsic calibration saved at {}".format(
            file_name, dest_path)) 
        return file_name
    except IOError as e: # Report any error saving de data
        print("[ERROR][INTRINSIC]: Problem saving camera intrinsic calibration: {}".format(e))
        return ""

def load_intrinsic_calibration(abs_path, file_name):
    """ Load intrinsic parameters from file
    Args:
        abs_path: `string` absolute path where calibration file is saved
        file_name: `string` intrinsic calibration file name
    Returns:
        intrinsic_calibration: `dictionary` intrinsic calibration variables
            ret: `float` description
            mtx: `numpy.ndarray` Output 3x3 floating-point camera matrix 
            dist: `numpy.ndarray` Output vector of distortion coefficients (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements
            rvecs: `list` Output vector of rotation vectors 
            tvecs: `list` Output vector of translation vectors estimated for each pattern view.
            newcameramtx: `numpy.ndarray` Output new camera matrix.
            roi: `tuple` Image size after rectification. By default,it is set to imageSize
            img_width: `int` images width
            img_height: `int` images height
    """

    try: # load the calibration data in file
        with open(os.path.join(abs_path, file_name)) as f:
            data_loaded = yaml.load(f)
        ret = data_loaded["ret"]
        mtx = np.array(data_loaded["mtx"])
        dist = np.array(data_loaded["dist"])
        rvecs = data_loaded["rvecs"]
        tvecs = data_loaded["tvecs"]
        newcameramtx = np.array(data_loaded["newcameramtx"])
        roi = data_loaded["roi"]
        img_width = data_loaded["img_width"]
        img_height = data_loaded["img_height"]
        print("[INFO][INTRINSIC]: Camera intrinsic calibration loaded")

    except IOError as e: # Report any error saving de data
        print("[ERROR][INTRINSIC]: Problem loading camera intrinsic calibration: {}".format(e))
        return {"ret":None, "mtx":None, "dist":None, "rvecs":None, "tvecs":None, 
        "newcameramtx":None, "roi":None, "img_width":None, "img_height":None}

    return {"ret":ret, "mtx":mtx, "dist":dist, "rvecs":rvecs, "tvecs":tvecs, 
        "newcameramtx":newcameramtx, "roi":roi, "img_width":img_width, 
        "img_height":img_height}

def validate_intrinsic_calibration(abs_path, intrinsic_calibration):
    """ Validates and show calibration in image gallery
    Args:
        abs_path: `string` absolute path where image for calibration are saved
        intrinsic_calibration: `dictionary` intrinsic calibration variables
            ret: `float` description
            mtx: `numpy.ndarray` Output 3x3 floating-point camera matrix 
            dist: `numpy.ndarray` Output vector of distortion coefficients (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements
            rvecs: `list` Output vector of rotation vectors 
            tvecs: `list` Output vector of translation vectors estimated for each pattern view.
            newcameramtx: `numpy.ndarray` Output new camera matrix.
            roi: `tuple` Image size after rectification. By default,it is set to imageSize
            img_width: `int` images width
            img_height: `int` images height
    Returns:
    """

    win_name = 'Chess_boards'

    # loop through images in provided absolute path 
    for idx, image_file in enumerate(os.listdir(abs_path)):
        if image_file.endswith("jpg"):

            # read images and turn them in gray scale
            img = cv2.imread(os.path.join(abs_path, image_file))

            # 1. Using undistort
            img_dst = cv2.undistort(src=img, cameraMatrix=intrinsic_calibration["mtx"], 
                distCoeffs=intrinsic_calibration["dist"])

            # 2. Using remapping
            # mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix=mtx, distCoeffs=dist, 
            #     R=None, newCameraMatrix=newcameramtx, size=(640, 360), m1type=5)
            # img_dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

            # Show detected chess boards 
            cv2.imshow(winname=win_name, mat=img_dst); cv2.waitKey(100)          

    # Destroy window
    cv2.destroyWindow(win_name)

# =============================================================================
if __name__ == '__main__':
    
    # Perform intrinsic calibration from image gallery
    intrinsic_calibration = perform_intrinsic_calibration(abs_path="./Gallery", n_x=6, n_y=4)

    # Saves intrinsic calibration from image gallery
    file_name=save_intrinsic_calibration(dest_path="", intrinsic_calibration=intrinsic_calibration)

    # Loads intrinsic calibration from file
    intrinsic_calibration = load_intrinsic_calibration(abs_path="", file_name=file_name)

    """
        Weird results?: Important making sure that the calibration grid fills as
        much of the image as possible is important, or at least have some pictures 
        of the checkerboard where it is near the boundaries. This issue is that 
        distortion is small near the center and large at the boundaries. A large 
        distortion therefore doesn't affect the center of the image very much. 
        Equivalently, a small error in estimating the distortion from the center of 
        the image leads to a perhaps very wrong overall estimate of the distortion. 
        Redundant images matter "a little"; it is effectively making those images 
        (and therefore, the location of the checkerboard pattern in those images) 
        more important.
    """
    # Validate calibration
    validate_intrinsic_calibration(abs_path="./Gallery", intrinsic_calibration=intrinsic_calibration)
