#!/usr/bin/env python
# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus, Computer Vision & Ai Team
"""

# =============================================================================
import numpy as np
import time
import sys
import csv
import os

import ntplib
import rospy
import cv2
import yaml
import subprocess
import binascii

import datetime
from glob import glob

from extended_rospylogs import Debugger, update_debuggers, loginfo_cond, logerr_cond
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

from std_msgs.msg import Bool
from video_mapping.srv import GetMoreCameraStatus
from data_capture.msg import Status

from easy_memmap import MultiImagesMemmap
from threading import Timer

from video_mapping.utils.cameras import read_cam_ports

# =============================================================================
class DataCapture(Debugger):
    def __init__(self, csv_file, dest_folder):
        """ initializes class
        Args:
            csv_file: `string` csv absolute path to save images
            dest_folder: `string` destination folder to save images
            data_capture: `boolean` Enable/Disable data capture process
        Returns:
        """

        # inherit from other classes
        super(DataCapture, self).__init__()

        self.csv_file = csv_file # csv absolute path to save images
        self.dest_folder = dest_folder # Destination folder to save images
        self.quality = int(os.getenv(key='IMG_QUALITY', default=80)) # [0-100] Quality to save images
        self.capture_id = 0 # Current data capture identifier for the csv file
        self.images = [] # List of images to write
        self.recording = False # Enable/Disable data recording
        self.desired_fps = int(os.getenv(key='DATA_CAPTURE_FPS', default=15))
        self.fps = 0. # Frame rate to capture data
        
        # TODO: Camera status service to check state of cameras 
        # self.camera_status_service = rospy.ServiceProxy(
        #     name='video_mapping/get_cameras_status_verbose', 
        #     service_class=GetMoreCameraStatus)

        # Subscriber to enable/disable data capturing
        rospy.Subscriber(name="cumbia_rover/data_capture/capture", data_class=Bool, 
            callback=self.capture_cb, queue_size=2)

        # check if usb is right mounted and with space left
        self.space_left = 100.

    def capture_cb(self, data):
        """ Callback function to update data capture class
        Args:
            data: `Bool` data from message
        Returns:
        """

        # check if usb is right mounted and with space left
        self.space_left = space_left(device_path=self.dest_folder, percentage=True)

        # -----------------------------------------------------------------
        # Check possible error cases
        # 1 - Check for space in storing device
        if self.space_left <= float(os.getenv('MIN_USB_SPACE', 3)):
            self.debugger(DEBUG_LEVEL_0, "Can't record video, USB FULL", log_type = 'warn')
        
        # 2 - Check if the usb device is connected
        elif self.dest_folder is None:
            self.debugger(DEBUG_LEVEL_0, "Can't record video, USB is not mounted", log_type = 'warn')

        # 3 - Check if there's no csv file
        elif self.csv_file is None:
            self.debugger(DEBUG_LEVEL_0, "Can't record video, some problems with the USB", log_type = 'warn')
 
        # -----------------------------------------------------------------
        self.recording = not self.recording
        if self.recording: 
            self.debugger(DEBUG_LEVEL_0, "Data recording {} started".format(self.capture_id), log_type = 'info')
        else: 
            self.debugger(DEBUG_LEVEL_0, "data recording {} stopped".format(self.capture_id), log_type = 'info')
            self.capture_id += 1 # Increment capture identifier
                
# =============================================================================
def write_images(images, dest, quality, multiple_images=False, img_format="jpg"):
    """ saves a list of images in destination folder
    Args:
        images: `list` list of images to write in memory
        dest: `string` absolute path where images will be saved
        quality: `int` [%] percentage of quality to save images
    Returns:
        timestamp: `time` current time
        name_l: `string` saved image file of left camera
        name_c: `string` saved image file of center camera
        name_r: `string` saved image file of right camera
    """

    timestamp = int(time.time()*1000)
    prefix = binascii.b2a_hex(os.urandom(2))
    
    name_l = '{}-{}_{}.{}'.format(prefix, timestamp, "l", img_format)
    name_c = '{}-{}_{}.{}'.format(prefix, timestamp, "c", img_format)
    name_r = '{}-{}_{}.{}'.format(prefix, timestamp, "r", img_format)

    if multiple_images:
        cv2.imwrite(os.path.join(dest, name_l), images[0], [cv2.IMWRITE_JPEG_QUALITY, quality])
        cv2.imwrite(os.path.join(dest, name_c), images[1], [cv2.IMWRITE_JPEG_QUALITY, quality])
        cv2.imwrite(os.path.join(dest, name_r), images[2], [cv2.IMWRITE_JPEG_QUALITY, quality])
    else:
        cv2.imwrite(os.path.join(dest, name_c), images[0][0], [cv2.IMWRITE_JPEG_QUALITY, quality])

    return timestamp, name_l, name_c, name_r

def check_usb(msg, bot_data, main_debugger, dest, device):
    """ Reads from disk available space in usb and number of images recorded 
        msg, bot, main_debugger -> objects passed by reference dest is actual 
        folder from today for recording device is where the usb is mounted
    Args:
        msg: `string` debugger message
        bot: `DataCapture` data capture variable
        main_debugger: `Debugger` debugger to report messages
        dest: `string` absolute path to destination folder
        device: `type` absolute path to mounted device
    Returns:
    """

    msg.number_images = get_number_files(dest)
    msg.space_left = space_left(device_path=device, percentage = True)
    bot_data.space_left = msg.space_left

    if (msg.number_images == -1 
        or msg.space_left == -1 
        or msg.space_left <= float(os.getenv('MIN_USB_SPACE', 3))):
        bot_data.csv_file = None
        bot_data.recording = False
    
    if bot_data.csv_file is None:
        try:
            csv_file, log_msg = create_folder_csv_4data_capture(dest)
        except Exception as e:
            csv_file, log_msg = None, "USB may have been disconnected"
    
    main_debugger.debugger(DEBUG_LEVEL_1, 
        'Reading Usb Space: {}% empty; Captured images: {}'.format(
        msg.space_left, msg.number_images))

def space_left(device_path, percentage=True):
    """ calculates left space in device
    Args:
        device_path: `string` absolute path to device
        percentage: `boolean` return value in percentage
    Returns:
        _: `float` total of space in device in bytes or percentage
    """
    if device_path is None:
        return 100
    try:
        disk = os.statvfs(device_path)
    except Exception as e:
        logerr_cond(True, "Error reading USB {}".format(e))
        return -1  
    totalBytes = float(disk.f_bsize * disk.f_blocks)
    totalAvailSpace = float(disk.f_bsize * disk.f_bfree)
    if percentage:
        return totalAvailSpace/totalBytes * 100.
    else:
        return totalAvailSpace / 1024. / 1024. / 1024.

def get_number_files(folder):
    """ calculates amount of files in folder
    Args:
        folder: `string` absolute path 
    Returns:
        _: `int`  amount of files in folder
    """
    if folder is None: return 0
    try:
        return (len([name for name in os.listdir(folder) if os.path.isfile(os.path.join(folder, name))]) - 1)
    except Exception as e:
        logerr_cond(True, "Error reading USB {}".format(e))
        print("[ERROR] - Error reading USB {}".format(e))
        return -1

def create_folder_csv_4data_capture(dest_folder):
    """ creates data.csv headers if it not exists and Creates folder for 
        capturing data
    Args:
        dest_folder: `string` destination folder of csv file
    Returns:
        csv_file: `string` csv file absolute path
        message: `string` message for debugger
    """
    
    # Create folder if does not exits
    if os.path.exists(dest_folder):
        message = 'Folder for datacapture exists'
    else:
        message = 'Folder for datacapture does not exist'
        os.mkdir(dest_folder)

    # Path to csv file
    csv_file = os.path.join(dest_folder,'data.csv')

    # If csv file does not exits in destination path
    if not os.path.isfile(csv_file):
        with open(csv_file, 'a') as fd:
            writer = csv.writer(fd)
            row = [ 'capture_id', 'timestamp', 'camera_label', 'image_file']
            writer.writerow(row)

    return csv_file, message

def get_usb_devices():
    sdb_devices = map(os.path.realpath, glob('/sys/block/sd*'))
    usb_idx = None
    for idx, dev in enumerate(sdb_devices):
        if 'usb' in dev: usb_idx = idx
    if usb_idx is not None:
        usb_devices = {sdb_devices[usb_idx][-1], sdb_devices[usb_idx]}
        return usb_devices
    else:
        return []

def get_mount_points(devices=None):
    devices = devices or get_usb_devices() # if devices are None: get_usb_devices
    output = subprocess.check_output(['mount']).splitlines()
    is_usb = lambda path: any(dev in path for dev in devices)
    usb_info = (line for line in output if is_usb(line.split()[0]))
    return [(info.split()[0], info.split()[2]) for info in usb_info]

def setProcessName(name):
    """ sets name fot current process
    Args:
        name: `string` name of process
    Returns:
    """
    
    if sys.platform in ['linux2', 'linux']:
        import ctypes
        libc = ctypes.cdll.LoadLibrary('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    else:
        raise Exception("Can not set the process name on non-linux systems: " + str(sys.platform))

# =============================================================================
def main():

    # create and set debugger: 0-DEBUG, 1-INFO, 2-WARNING, 3-ERROR, 4-FATAL_ERROR
    main_debugger = Debugger()
    
    # Initialize data capture ros node
    rospy.init_node('data_capture_node', anonymous=True)
    rospy.set_param('/data_capture/debug', 0)
    setProcessName("data_capture_node")

    rate = 30 #args.rate
    r = rospy.Rate(hz=rate) # Set ros node rate

    # Get current date
    date = datetime.date.today().strftime("%m-%d-%y")

    # Get base path
    base_path = None; device = None; sub_folder_path = "data"
    usb_devices = get_mount_points() # Get connected usb devices
    if len(usb_devices):
        base_path = usb_devices[-1][-1] # Get absolute path to root in usb device
        device = usb_devices[-1][0] # Get usb device 
        base_path = os.path.join(base_path,"data_capture-{}".format(date))
        main_debugger.debugger(DEBUG_LEVEL_0, "USB Detected: {}".format(device), log_type='info')

    # If path already exits then rename destination folder with new index
    if base_path is not None:
        if os.path.isdir(base_path):
            i=1; axu_dest=base_path+"({})".format(i)
            while(os.path.isdir(axu_dest)):
                i+=1; axu_dest=base_path+"({})".format(i)
            base_path=axu_dest
        # Create subfolder to save images
        sub_folder_path = os.path.join(base_path, sub_folder_path)
        if not os.path.isdir(sub_folder_path):
            os.makedirs(str(sub_folder_path))

    try: # Create data.csv headers if it not exists
        if base_path is not None:
            main_debugger.debugger(DEBUG_LEVEL_0, "Destination folder: {}".format(base_path), log_type='info')
            csv_file, log_msg = create_folder_csv_4data_capture(dest_folder=base_path,)
        else:
            csv_file, log_msg = None, "USB device no found or it may have been disconnected"
            main_debugger.debugger(DEBUG_LEVEL_0, log_msg, log_type='warn')
    except Exception as err:
        csv_file, log_msg = None, "Error creating csv file"
        main_debugger.debugger(DEBUG_LEVEL_0, err, log_type='err')

    # Initialize memmap variable and wait for data
    video_map = MultiImagesMemmap(mode="r", name="main_stream", memmap_path=os.getenv("MEMMAP_PATH", "/tmp"))
    video_map.wait_until_available() #initialize and find video data
    main_debugger.debugger(DEBUG_LEVEL_0, "Memmap video data ready!", log_type='info')

    MotionTestTrack = DataCapture(csv_file=csv_file, dest_folder=base_path)

    cam_labels=read_cam_ports(os.environ.get("CAM_PORTS_PATH"))

    # Init ros node cycle
    while not rospy.is_shutdown():

        # images = [video_map.read(cam_label) for cam_label in cam_labels.keys()]
  
        # # If data capture then do
        # if MotionTestTrack.recording and base_path is not None: 
        #     pass
        #     if first_time: start_time = time.time() # Get time if first iteration
        #     start_time_local = time.time() # Get local start time
        #     i = 0; is_same_image = True 
            
        #     while is_same_image:
        #         if i >= 1000: #more than one sec and same image, 
        #             main_debugger.debugger(DEBUG_LEVEL_1, 
        #             "Could not grab a valid frame in more than 1 sec", 
        #             log_type = 'warn')
        #             break
        #         images = []
        #         if bot_data_capture.is_sim: # If environment is simulated
        #             if(bot_data_capture.multiple_cameras): 
        #                 images.append(bot_data_capture.left_image)
        #                 images.append(bot_data_capture.center_image)
        #                 images.append(bot_data_capture.right_image)
        #             else:
        #                 images.append([bot_data_capture.center_image])
        #             images = np.array(images, dtype=np.uint8)
        #         else: 
        #             images = capture_images_memory(video_map)
                
        #         # Check if current capture change
        #         is_same_image = np.array_equal(c_image_old, images[0]) if (bot_data_capture.is_sim and 
        #             not bot_data_capture.multiple_cameras) else np.array_equal(c_image_old, images[1])
        #         if not is_same_image: 
        #             c_image_old = np.array(images[0], dtype=np.uint8).copy() if(bot_data_capture.is_sim and 
        #                 not bot_data_capture.multiple_cameras) else images[1].copy()
        #         time.sleep(0.001); i +=1 # Wait and increment capture time index

        #     if not is_same_image and bot_data_capture.throttle > 0.0:
        #     # if not is_same_image:

        #         # Show captured images to record
        #         # for idx, img in enumerate(images):
        #         #     cv2.imshow("test_data_capture_{}".format(idx), img); cv2.waitKey(10)
                
        #         # Write images in destination
        #         bot_data_capture.images = images
        #         timestamp, name_l, name_c, name_r = write_images(
        #             images=bot_data_capture.images, 
        #             dest=sub_folder_path, 
        #             quality=bot_data_capture.quality,
        #             multiple_images=bot_data_capture.multiple_cameras)
        #         names = [name_l, name_c, name_r]

        #         # Structure: 'capture', 'timestamp','camera','filename', 'steering
        #         rows = [[bot_data_capture.capture_id, timestamp, cam_label, file_name, 
        #             bot_data_capture.steering_angle] for file_name, cam_label in zip([name_l, 
        #             name_c, name_r], ["L", "C", "R"])]

        #         # Write data and variables in csv file
        #         with open(bot_data_capture.csv_file, 'a') as fd:
        #             writer = csv.writer(fd); writer.writerows(rows)

        #         first_time = False; msg.recording = True

        #         # Update frame rate
        #         bot_data_capture.fps = 1.0/(time.time()-start_time)
        #         fps = 1.0/(time.time()-start_time_local)

        #         # If time left for next capture then wait
        #         start_time = time.time()
        #         time2sleep = 1.0/bot_data_capture.desired_fps - 1.0/fps - 1.0/rate
        #         main_debugger.debugger(DEBUG_LEVEL_3, "Capture FPS: {}, time2sleep: {}".format(
        #             bot_data_capture.fps, time2sleep), log_type='info')
        #         if time2sleep >= 0:
        #             time.sleep(time2sleep)

        # else:
        #     msg.recording = False; first_time = True
        #     bot_data_capture.fps = 0; time.sleep(0.1)

        r.sleep()

# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except:
        logerr_cond(True, sys.exc_info())

# =============================================================================