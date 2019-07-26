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

    # Local launch variables
    cam_idx = 0

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
            cv2.putText(images[cam_idx], "{}".format(cam_labels[cam_idx]), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, 8)
            cv2.putText(images[cam_idx], "{}".format(cam_labels[cam_idx]), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1, 8)
            cv2.imshow("Local_visualizer", images[cam_idx]); key = cv2.waitKey(10)
            if   key==173: # (-) If pressed go to previous camera
                if cam_idx!=0: cam_idx-=1
            elif key==171: # (+) If pressed go to next camera
                if cam_idx<len(images)-1: cam_idx+=1
            elif key==115: # (S) If pressed save image current capture
                re_path = os.getenv(key="CALIBRATION_PATH"); pic_idx = 0
                abs_path = "{}/picture_{}({}).jpg".format(re_path, cam_labels[cam_idx], pic_idx)
                while os.path.isfile(abs_path):
                    pic_idx+=1
                    abs_path="{}/picture_{}({}).jpg".format(re_path, cam_labels[cam_idx], pic_idx)
                try:
                    cv2.imwrite(filename=abs_path, img=images[cam_idx]) 
                    main_debugger.debugger(DEBUG_LEVEL_0, "Image saved at {}".format(
                        abs_path), log_type="info")
                except:
                    main_debugger.debugger(DEBUG_LEVEL_0, "Saving image ar {}".format(
                        abs_path), log_type="err")               
            elif key==113: # (Q) If pressed then quit/restrat node
                exit()
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