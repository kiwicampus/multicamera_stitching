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

    # Get video memmap absolute path
    video_path = os.getenv(key="MEMMAP_PATH", default="/tmp") 

    # Get axis for concatenating all thread images: Axis=1 showed almost 10x 
    # better performance than axis=2, and slightly better than axis=0
    axis_concat = int(os.getenv(key="MMAP_AXIS_CONCAT", default=1))

    rospy.init_node('video_mapping_node', anonymous=True) # Node initialization
    rospy.set_param('/video_mapping/debug', 0) # Set node debugger
    setProcessName("video_mapping_node") # Set video mapping process name
    r = rospy.Rate(hz=30) # Set node rate

    # Initiate CameraSupervisor Class that handles the threads that reads the 
    # cameras. Dont take last label, since it doesn't correspond to a physical 
    # device
    cameras_labels = ["L", "C", "R"] # define camera labels order for memory mapping
    cameras_supervisor = CamerasSupervisor( #mtx, dist 
        None, None, cameras_labels=cameras_labels)
        
    # video mapping for raw images
    video_map = MultiImagesMemmap(
        memmap_path=video_path, labels=cameras_labels, name="main_stream", 
        axis=axis_concat, mode="w")

    # Setting debuggers
    main_debugger = Debugger() # Debugger that logs properly in stdout
    # Used to update logging level. DEPRECATED since reading rosparam in main loop is SLOW
    list_debuggers = [cameras_supervisor, main_debugger]+cameras_supervisor.camera_handlers
    
    main_debugger.debugger(DEBUG_LEVEL_0, "Initiating main loop")
    while not rospy.is_shutdown():
        # Read into a list all images read from the threads
        images = list(map(lambda o: o.image, cameras_supervisor.camera_handlers))

        #Show video streams
        for idx, cam_label in enumerate(cameras_labels):
            cv2.imshow("CAM{}".format(cam_label), images[idx]); cv2.waitKey(10)

        # Concatenate all list images in one big 3D matrix
        data = np.concatenate(images, axis=axis_concat)
        video_map.write(data) # Write into memory

        # Suspend execution of R expressions for a specified time interval. 
        r.sleep()

# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# =============================================================================