#!/usr/bin/env python
# =============================================================================
"""
Code Information:
	Programmer: John Betancourt G.
	Phone: +57 (311) 813 7206
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision & Ai Team
"""
# =============================================================================
import numpy as np
import rospy
import cv2
import sys
import os

from extended_rospylogs import Debugger, update_debuggers, loginfo_cond, logerr_cond
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

from easy_memmap import MultiImagesMemmap, EasyMemmap
from utils.cameras import CamerasSupervisor

from std_msgs.msg import Bool

from Intrinsic import validate_intrinsic_calibration
from Intrinsic import perform_intrinsic_calibration
from Intrinsic import save_intrinsic_calibration
from Intrinsic import load_intrinsic_calibration

from Extrinsic import perform_extrinsic_calibration
from Extrinsic import save_extrinsic_calibration
from Extrinsic import load_extrinsic_calibration

# =============================================================================
def setProcessName(name):
    if sys.platform in ['linux2', 'linux']:
        import ctypes
        libc = ctypes.cdll.LoadLibrary('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    else:
        raise Exception("Can not set the process name on non-linux systems: " + 
            str(sys.platform))

# =============================================================================
def main():

    # Enable(1)/Disable(0) local run
    LOCAL_RUN=int(os.getenv(key="LOCAL_RUN", default=0))

    # Start rosnode
    rospy.init_node('video_mapping_node', anonymous=True) # Node initialization
    rospy.set_param('/video_mapping/debug', 0) # Set node debugger
    setProcessName("video_mapping_node") # Set video mapping process name
    r = rospy.Rate(hz=30) # Set node rate

    # Initiate CameraSupervisor Class that handles the threads that reads the cameras
    cameras_supervisor = CamerasSupervisor()
    cam_labels = cameras_supervisor.cameras_labels.keys()

    # video mapping for raw images
    video_map = MultiImagesMemmap(memmap_path=os.getenv(key="MEMMAP_PATH", default="/tmp"), 
      labels=cam_labels, name="main_stream", axis=1, mode="w")

    # Setting debuggers
    main_debugger = Debugger() # Debugger that logs properly in stdout
    main_debugger.debugger(DEBUG_LEVEL_0, "Initiating main loop")

    # Calibration variables
    intrinsic_calibration = load_intrinsic_calibration(abs_path=os.path.dirname(
        os.getenv(key="CAM_PORTS_PATH")), file_name="cam_intrinsic_{}X{}.yaml".format(
            int(os.environ.get("VIDEO_WIDTH", 640)), int(os.environ.get("VIDEO_HEIGHT", 360))))
    extrinsic_calibrations = {key:{"M":None, "INV":None} for key in cam_labels}
    stitcher_config = {"intrisic":intrinsic_calibration}

    # Local launch variables
    local_cam_idx = 0; local_intrinsic=True; LOCAL_WIN_NAME="Local_visualizer"
    while not rospy.is_shutdown():

        # Read into a list all images read from the threads
        images = list(map(lambda o: o.image, cameras_supervisor.camera_handlers))

        # Concatenate all list images in one big 3D matrix and write them into memory
        video_map.write(np.concatenate(images, axis=1)) 

        # Suspend execution of R expressions for a specified time interval. 
        r.sleep()

        # ---------------------------------------------------------------------
        # Visual debugging - Visual debugging - Visual debugging - Visual debug
        if LOCAL_RUN:
            # for idx, img in enumerate(images):
            #     cv2.imshow("CAM{}".format(cam_labels[idx]), img)

            img = images[local_cam_idx]
            # If intrinsic calibration available
            if intrinsic_calibration["mtx"] is not None and local_intrinsic:

                # Undistord the image
                img=cv2.undistort(src=img, cameraMatrix=intrinsic_calibration["mtx"], 
                    distCoeffs=intrinsic_calibration["dist"])
            
                # If extrinsic calibration available 
                if (extrinsic_calibrations[cam_labels[local_cam_idx]]["M"] is not None 
                    and local_extrinsic):
                    pass

            cv2.putText(img, "{}".format(cam_labels[local_cam_idx]), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, 8)
            cv2.putText(img, "{}".format(cam_labels[local_cam_idx]), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1, 8)

            cv2.imshow(LOCAL_WIN_NAME, img); key = cv2.waitKey(10)
            if   key==173: # (-) If pressed go to previous camera
                if local_cam_idx!=0: local_cam_idx-=1
            elif key==171: # (+) If pressed go to next camera
                if local_cam_idx<len(images)-1: local_cam_idx+=1
            elif key==115: # (S) If pressed save image current capture
                re_path = os.getenv(key="CALIBRATION_PATH"); pic_idx = 0
                abs_path = "{}/picture_{}({}).jpg".format(re_path, cam_labels[local_cam_idx], pic_idx)
                while os.path.isfile(abs_path):
                    pic_idx+=1
                    abs_path="{}/picture_{}({}).jpg".format(re_path, cam_labels[local_cam_idx], pic_idx)
                try:
                    cv2.imwrite(filename=abs_path, img=images[local_cam_idx]) 
                    main_debugger.debugger(DEBUG_LEVEL_0, "Image saved at {}".format(
                        abs_path), log_type="info")
                except:
                    main_debugger.debugger(DEBUG_LEVEL_0, "Saving image ar {}".format(
                        abs_path), log_type="err")               
            elif key==113: # (Q) If pressed then quit/restrat node
                exit()
            elif key==105: # (I) If pressed perform intrinsic camera calibration

                # Perform intrinsic calibration from image gallery
                intrinsic_calibration = perform_intrinsic_calibration(
                    abs_path=os.getenv(key="CALIBRATION_PATH"), n_x=6, n_y=4)
                
                # Saves intrinsic calibration from image gallery
                file_name=save_intrinsic_calibration(dest_path=os.path.dirname(os.getenv(key="CAM_PORTS_PATH")), 
                    intrinsic_calibration=intrinsic_calibration)
                
                intrinsic_calibration=load_intrinsic_calibration(abs_path=os.path.dirname(
                    os.getenv(key="CAM_PORTS_PATH")), file_name=file_name)

                # Validate calibration
                validate_intrinsic_calibration(abs_path=os.getenv(key="CALIBRATION_PATH"), 
                    intrinsic_calibration=intrinsic_calibration)
            elif key==101: # (E) If pressed perform Extrinsic camera calibration
                extrinsic_calibration=perform_extrinsic_calibration(
                    img_src=img, WIN_NAME=LOCAL_WIN_NAME)
                save_extrinsic_calibration(file_path=os.path.dirname(os.getenv(key="CAM_PORTS_PATH")), 
                    file_name="{}_extrinsic.yaml".format(cam_labels[local_cam_idx]), 
                    extrinsic_calibration=extrinsic_calibration)
            elif key==100: # (D) If pressed start/Stop data capture
                local_data_capture_pub = rospy.Publisher("MotionTestTrack/data_capture/capture", Bool, queue_size=2)
                local_data_capture_pub.publish(True)
            elif key!=-1:  # No key command
                print("Command or key action no found: {}".format(key))
            
        # ---------------------------------------------------------------------

# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# =============================================================================