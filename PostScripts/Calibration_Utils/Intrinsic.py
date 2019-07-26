# =============================================================================
"""
Code Information:
	Programmer: John Betancourt G.
	Phone: +57 (311) 813 7206
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision & Ai Team
"""

# =============================================================================
def intrisic_calibration(abs_path="./Gallery", n_x=6, n_y=4):

    """ From chess board pictures gallery find the coefficient and distortion of 
        the camera and other calibration parameters
    Args:
        path: `string` absolute path where image for calibration are saved
        n_x: `int` number rows of the chessboard used to calibrate
        n_y: `int` number columns of the chessboard used to calibrate
    Returns:
        None: `None type`  No returns
    """
    
    # setup object points variables
    objp = np.zeros((n_y * n_x, 3), np.float32)
    objp[:, :2] = np.mgrid[0:n_x, 0:n_y].T.reshape(-1, 2)
    image_points = [];  object_points = []; fail_boards = 0 

    # loop through images in provided absolute path 
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
                            
                # Print on image info
                cv2.putText(img_chess, "img_{}".format(idx), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, 8)
                cv2.putText(img_chess, "img_{}".format(idx), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1, 8)

                # Show detected chess boards 
                cv2.imshow(winname='Chess_boards', image=img_chess)
                cv2.waitKey(500)                              

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

    if len(object_points): # If corners intersection were found then calibrate

        # perform the calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPoints=object_points,
            imagePoints=image_points, point_counts=img_gray.shape[::-1], 
            imageSize=None, cameraMatrix=None)

        # Find optimum camera matrix to Undistorted images
        h,  w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix=mtx, 
            distCoeffs=dist, imageSize=(w, h), alpha=1, new_camera_matrix=(w, h))

    return ret, mtx, dist, rvecs, tvecs, newcameramtx, roi, img_width, img_height

def save_intrisic_calibration(ret, mtx, dist, rvecs, tvecs, newcameramtx, roi, 
    img_width, img_height):
    pass

def validate_calibration(abs_path, ret, mtx, dist, rvecs, tvecs, newcameramtx, roi):
    pass

# =============================================================================
if __name__ == '__main__':
    
    # Perform intrinsic calibration from image gallery
    ret, mtx, dist, rvecs, tvecs, newcameramtx, roi, img_width, img_height = intrisic_calibration(
        abs_path="./Gallery", n_x=6, n_y=4)

    # # Save intrinsic calibration from image gallery
    # save_intrisic_calibration(dest_path="", ret=ret, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs, 
    #     newcameramtx=newcameramtx, roi=roi, img_width=img_width, img_height=img_height)

    # # Validate calibration
    # validate_calibration(abs_path="./Gallery", ret=ret, mtx=mtx, dist=dist, rvecs=rvecs, 
    #     tvecs=tvecs, newcameramtx=newcameramtx, roi=roi, img_width=img_width, 
    #     img_height=img_height)
