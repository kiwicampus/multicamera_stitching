#!/usr/bin/env python
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

    while not rospy.is_shutdown():

        # Read into a list all images read from the threads
        images = list(map(lambda o: o.image, cameras_supervisor.camera_handlers))

        # ---------------------------------------------------------------------
        # Visual debugging - Visual debugging - Visual debugging - Visual debug
        if LOCAL_RUN:
            for idx, img in enumerate(images):
                cv2.imshow("CAM{}".format(cam_labels[idx]), img)
            key = cv2.waitKey(10)
            if   key==113: # (Q) If press q then quit
                exit()
            elif key!=-1:  # No key command
                print("Command or key action no found: {}".format(key))
        # ---------------------------------------------------------------------

        # Concatenate all list images in one big 3D matrix and write them into memory
        video_map.write(np.concatenate(images, axis=1)) 

        # Suspend execution of R expressions for a specified time interval. 
        r.sleep()

# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# =============================================================================