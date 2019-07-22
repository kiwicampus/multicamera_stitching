#!/usr/bin/env python
# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Phone: +57 (350) 283 51 22
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer Vision Team
"""

# =============================================================================
import numpy as np
import time
import sys
import csv
import os

from glob import glob
import ntplib
import rospy
import cv2

from extended_rospylogs import Debugger, update_debuggers, loginfo_cond, logerr_cond
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Bool

from video_mapping.srv import GetMoreCameraStatus
from data_capture.msg import Status

import subprocess
import binascii

from easy_memmap import MultiImagesMemmap
from threading import Timer

# =============================================================================
class RoscppTimer(object):
    '''
    Class for handling Timer, beeing able to reset them. Similar API to Roscpp Timers. TODO: update to have same API
    Constructor: period in seconds of handler, function to callback

    Methods: start() to start the timer
            stop() to stop the timer
    '''

    def __init__(self, interval, f, *args, **kwargs):
        self.interval = interval
        self.f = f
        self.args = args
        self.kwargs = kwargs

        self.timer = None

    def _callback(self):
        self.f(*self.args, **self.kwargs)
        self._start()

    def _start(self):
        self.timer = Timer(self.interval, self._callback)
        self.timer.daemon=True # finish if father finishes
        self.timer.start()

    def stop(self):
        # only if there is a timer cancel the thread
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def start(self):
        # only if there is no timer yet launch the thread
        if self.timer is None:
            self._start()

class DataCapture(Debugger):
    def __init__(self, bot_id, csv_file, dest_folder, jpg_quality,
        capture_topic="cumbia_rover/data_capture/capture",
        mavros_topic = "cumbia_rover/twist_cmd",
        queue=2, data_capture=True):
        """ initializes class
        Args:
            bot_id: `string` Bot's uniq identifier
            csv_file: `string` csv absolute path to save images
            dest_folder: `string` destination folder to save images
            jpg_quality: `int` [0-100] Quality to save images
            capture_topic: `string` name of data capture topic
            queue: `int` size of queue for topics
            data_capture: `boolean` Enable/Disable data capture process
        Returns:
        """

        super(DataCapture, self).__init__()

        self.bot_id = bot_id # Bot's uniq identifier
        self.csv_file = csv_file # csv absolute path to save images
        self.dest_folder = dest_folder # Destination folder to save images
        self.quality = jpg_quality # [0-100] Quality to save images
        self.data_capture = data_capture # Enable/Disable data capture process

        self._capture_topic = capture_topic # Topic name for data capture
        self._mavros_topic = mavros_topic # Mavros topic name to subscribe
        self.image_topic_center = "cumbia_rover/camera/center/image_raw"
        self.image_topic_left = "cumbia_rover/camera/left/image_raw"
        self.image_topic_right = "cumbia_rover/camera/right/image_raw"
        self.is_sim = int(os.getenv(key='IS_SIM', default=1))  # is in simulation mode?
        #self.multiple_cameras = os.getenv(key='MULTIPLE_CAMS', default=True) #when in simulation, use one or 3 cameras?
        self.multiple_cameras = True
        self.bridge = CvBridge() # Cv bridge to convert image topics into cv images

        self.subscribers = [] # Subscribers list
        self.init_subscribers(queue) # initializes subscribers to save data in csv

        self.recording = False # Enable/Disable data recording
        self.desired_fps = int(os.getenv(key='DATA_CAPTURE_FPS', default=15))
        self.fps = 0. # Frame rate to capture data
        
        # csv file data
        self.steering_angle = 0.0 # Bot's steering angle [rad]
        self.throttle = 0.0 # Bot's throttle [0-1]
        self.capture_id = 0 # Current data capture identifier

        self.images = [None, None, None] # List of images to write

        # Camera status service to check state of cameras 
        self.camera_status_service = rospy.ServiceProxy(
            name='video_mapping/get_cameras_status_verbose', 
            service_class=GetMoreCameraStatus)

        # Subscriber to enable/disable data capturing
        rospy.Subscriber(name=self._capture_topic, data_class=Bool, 
            callback=self.capture_cb, queue_size=queue)

        # See if usb is right mounted and with space left
        self.space_left = space_left(device_path=self.dest_folder, percentage=True)

        # shutdown subscribers until recording it requested (save CPU usage)s
        self.unregister_subscribers()
        
    def init_subscribers(self, queue=2):
        """ initializes class subscribers
        Args:
            queue: `int` size of queue for topics
        Returns:
        """

        self.subscribers.append(rospy.Subscriber(name=self._mavros_topic, 
            data_class=TwistStamped, callback=self.mavros_cb, queue_size=queue))
         
        if self.is_sim:
            self.subscribers.append(rospy.Subscriber(name="cmd_vel", 
            data_class=Twist, callback=self.twist_cb, queue_size=queue))

            self.subscribers.append(rospy.Subscriber(name=self.image_topic_center, 
                data_class=Image, callback=self.image_center_cb, queue_size=queue))

            if(self.multiple_cameras):
                self.subscribers.append(rospy.Subscriber(name=self.image_topic_left, 
                data_class=Image, callback=self.image_left_cb, queue_size=queue))

                self.subscribers.append(rospy.Subscriber(name=self.image_topic_right, 
                data_class=Image, callback=self.image_right_cb, queue_size=queue))

    def unregister_subscribers(self):
        """ Unresgister subscribers in subscribers list
        Args:
        Returns:
        """

        for sub in self.subscribers:
            sub.unregister()

    def mavros_cb(self, data):
        """ Callback function - updates bot's steering and throttle value
        Args:
            data: `TwistStamped` data from message
        Returns:
        """

        self.steering_angle = data.twist.angular.z
        self.throttle = data.twist.linear.x
        self.debugger(DEBUG_LEVEL_1, "Steering_angle {}, Throttle: {}".format(
            self.steering_angle, self.throttle))
    
    def twist_cb(self, data):
        """ Callback function - updates bot's steering and throttle value
        Args:
            data: `TwistStamped` data from message
        Returns:
        """

        self.steering_angle = data.angular.z
        self.throttle = data.linear.x
        self.debugger(DEBUG_LEVEL_1, "Steering_angle {}, Throttle: {}".format(
            self.steering_angle, self.throttle))

    def image_center_cb(self, data):
        try:
            self.center_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print (e)

    def image_left_cb(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print (e)

    def image_right_cb(self, data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print (e)

    def capture_cb(self, data):
        """ Callback function - updates bot's capture state
        Args:
            data: `Bool` data from message
        Returns:
        """

        if data.data:
            self.debugger(DEBUG_LEVEL_2, "Received Capture Toggle: Desired state: {}".format(not self.recording))
            labels = ["L","C", "R"]
            if self.is_sim:
                status = ["OK", "OK", "OK"]
            else:
                #will get boolean with corresponding to ["L", "C", "R"]
                status = self.camera_status_service().camera_status 
                
            # Check the status of all cameras before start capturing data
            erro_camera_labels = []
            for idx, _ in enumerate(status):
                if status[idx] != "OK":
                    erro_camera_labels.append(labels[idx])
            
            # -----------------------------------------------------------------
            self.space_left = space_left(device_path=self.dest_folder, percentage=True)

            # -----------------------------------------------------------------
            # Check possible error cases
            log_str = None
            # 1 - Check for space in storing device
            if self.space_left <= float(os.getenv('MIN_USB_SPACE', 3)):
                log_str = "Can't record video, USB FULL"
            
            # 2 - Check if the usb device is connected
            elif self.dest_folder is None:
                log_str = "Can't record video, USB is not mounted!"

            # 3 - Cameras disconnected or not found
            # elif len(erro_camera_labels):
            #     log_str = "Can't record video, cameras: {} Not found".format(", ".join(erro_camera_labels))

            # 4 - Check if there's no csv file
            elif self.csv_file is None:
                log_str = "Can't record video, some problems with the USB"

            # -----------------------------------------------------------------
            if log_str is not None:
                self.notify_message(log_str=log_str, log_type='err')
                self.recording = False
            else:
                self.recording = not self.recording
                if self.recording: 
                    self.init_subscribers() # init pub/sub to catch data from robot
                    self.notify_message(log_str="data recording {} started".format(self.capture_id), 
                        log_type='info')
                else: 
                    # if stops recording, unsubscribe to save CPU usage
                    self.unregister_subscribers() # unregister subscribers
                    self.notify_message(log_str="data recording {} stopped".format(self.capture_id), 
                        log_type='info')
                    self.capture_id += 1 # Increment capture identifier
                    
    def notify_message(self, log_str, log_type='info'):
        """ Updates message to debugger and type 
        Args:
            log_str: `string` message
            log_type: `string` type of message debug, info, warn, err, fatal
        Returns:
        """
        self.debugger(DEBUG_LEVEL_0, log_str, log_type=log_type)

# # =============================================================================
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

def capture_images_memory(video_map):
    """ reads the images of camera L, C, and R from memory using memmap variable
    Args:
        video_map: `MultiImagesMemmap` memmap variable where images are located in memory
    Returns:
        _: `list` list with images loaded from memmap
    """

    return [video_map.read(cam_label) for cam_label in ["L", "C", "R"]]

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

def create_status_msg(size=(480,640), quality=80, device="/media", 
    images_folder=None):
    """ creates message type to report status 
    Args:
        size: `tuple` size of images to write/save
        quality: `int` [0-100] quality to save images
        device: `string` absolute path of device to save/write images
        images_folder: `string` folder to save/write images
    Returns:
        msg: `Status` message type to report status
    """

    msg = Status()
    msg.quality = quality
    msg.resolution = str(size[0])+"x"+str(size[1])
    msg.space_left = space_left(device_path=device, percentage=True)
    msg.fps = 0
    msg.recording = False
    if images_folder is not None:
        msg.number_images = get_number_files(images_folder)
    return msg

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

def get_current_date_str(format_str="%m-%d-%y"):
    """ gets date from remote server if possible
    Args:
        format_str: `string` format to get current date 
    Returns:
    """
    
    for _ in range(5):
        try:
            client = ntplib.NTPClient()
            response = client.request('pool.ntp.org')
            return time.strftime(format_str,time.localtime(response.tx_time))
        except:
            print('[WARNING]: Could not sync with time server... Retrying!')
            time.sleep(2)
    print('[ERROR]: Could not sync with time server. Using local time')
    return time.strftime(format_str)

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
            row = [ 'capture_id', 'timestamp', 'camera_label', 'image_file', 'steering']
            writer.writerow(row)

    return csv_file, message

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

# =============================================================================
def main():

    # create and set debugger: 0-DEBUG, 1-INFO, 2-WARNING, 3-ERROR, 4-FATAL_ERROR
    main_debugger = Debugger()
    
    # Initialize data capture ros node
    rospy.init_node('data_capture_node', anonymous=True)
    rospy.set_param('/data_capture/debug', 0)
    setProcessName("data_capture_node")

    rate = 15 #args.rate
    r = rospy.Rate(hz=rate) # Set ros node rate

    # create publishers
    capture_publisher = rospy.Publisher('data_capture/status', Status, queue_size=2)

    date = get_current_date_str("%m-%d-%y") # Get current date
    bot_id = int(os.getenv(key='ROVER_ID', default=0)) # Get bot's ID

    # Get base path
    base_path = None; device = None; sub_folder_path = "data"
    if not int(os.getenv(key='IS_SIM', default=0)): # If environment is a simulation
        usb_devices = get_mount_points() # Get connected usb devices
        if len(usb_devices):
            base_path = usb_devices[-1][-1] # Get absolute path to root in usb device
            device = usb_devices[-1][0] # Get usb device 
            base_path = os.path.join(base_path,"bot-{}-{}".format(bot_id, date))
            main_debugger.debugger(DEBUG_LEVEL_0, "USB Detected: {}".format(device), log_type = 'info')
            main_debugger.debugger(DEBUG_LEVEL_0, "Destination folder: {}".format(base_path), log_type = 'info')
    else: # If environment is real world
        main_debugger.debugger(DEBUG_LEVEL_0, "Runing in simulated environment", log_type = 'warn')
        base_path = os.getenv(key='USB_MOUNT', default="~/local_sim_data/")
        main_debugger.debugger(DEBUG_LEVEL_0, "Destination folder: {}".format(base_path), log_type = 'info')

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

    img_quality = 80 # Quality of images to write
    try: # Create data.csv headers if it not exists
        if base_path is not None:
            csv_file, log_msg = create_folder_csv_4data_capture(base_path)
        else:
            csv_file, log_msg = None, "USB device no found or it may have been disconnected"
            main_debugger.debugger(DEBUG_LEVEL_0, log_msg, log_type='warn')
    except Exception as err:
        csv_file, log_msg = None, "Error creating csv file"
        main_debugger.debugger(DEBUG_LEVEL_0, err, log_type='err')

    # Create data capture object to catch important variables and enable/disable data capturing
    bot_data_capture = DataCapture(bot_id=bot_id, csv_file=csv_file, dest_folder=base_path, 
        jpg_quality=img_quality, data_capture=int(os.getenv('DATA_CAPTURE', 0)))
    
    # Show messages in debuggers
    list_debuggers = [bot_data_capture, main_debugger]
    
    if not bot_data_capture.is_sim: # If environment is a simulation
        # Initialize memmap variable and wait for data
        video_map = MultiImagesMemmap(mode="r", name="main_stream", 
            memmap_path=os.getenv("MEMMAP_PATH", "/tmp"))
        video_map.wait_until_available() #initialize and find video data
        main_debugger.debugger(DEBUG_LEVEL_0, "Memmap video data ready", log_type='info')
        msg = create_status_msg(size=video_map.read().shape[:2], quality=img_quality,
            device=device, images_folder = base_path)
    else:
        msg = create_status_msg(quality=img_quality, device=device, 
            images_folder=base_path)

    # Create timer for checking usb space and images each minute
    check_usb_timer = RoscppTimer(60, check_usb, msg, bot_data_capture, 
        main_debugger, base_path, device)
    check_usb_timer.start()

    # Init ros node cycle
    main_debugger.debugger(DEBUG_LEVEL_0, "data capture node ready!", log_type='info')
    c_image_old = None; first_time = True
    while not rospy.is_shutdown():
        # makes recording slow, without this mean record fps 15 -> with this 10 
        node_debug_level = int(rospy.get_param('/data_capture/debug')) 
        update_debuggers(list_debuggers, node_debug_level) # Update debuggers
        msg.fps = bot_data_capture.fps # Update current frame rate

        if bot_data_capture.recording and base_path is not None: # If data capture then do
            if first_time: start_time = time.time() # Get time if first iteration
            start_time_local = time.time() # Get local start time
            i = 0; is_same_image = True 
            
            while is_same_image:
                if i >= 1000: #more than one sec and same image, 
                    main_debugger.debugger(DEBUG_LEVEL_1, 
                    "Could not grab a valid frame in more than 1 sec", 
                    log_type = 'warn')
                    break
                images = []
                if bot_data_capture.is_sim: # If environment is simulated
                    if(bot_data_capture.multiple_cameras): 
                        images.append(bot_data_capture.left_image)
                        images.append(bot_data_capture.center_image)
                        images.append(bot_data_capture.right_image)
                    else:
                        images.append([bot_data_capture.center_image])
                    images = np.array(images, dtype=np.uint8)
                else: 
                    images = capture_images_memory(video_map)
                
                # Check if current capture change
                is_same_image = np.array_equal(c_image_old, images[0]) if (bot_data_capture.is_sim and 
                    not bot_data_capture.multiple_cameras) else np.array_equal(c_image_old, images[1])
                if not is_same_image: 
                    c_image_old = np.array(images[0], dtype=np.uint8).copy() if(bot_data_capture.is_sim and 
                        not bot_data_capture.multiple_cameras) else images[1].copy()
                time.sleep(0.001); i +=1 # Wait and increment capture time index

            if not is_same_image and bot_data_capture.throttle > 0.0:
            # if not is_same_image:

                # Show captured images to record
                # for idx, img in enumerate(images):
                #     cv2.imshow("test_data_capture_{}".format(idx), img); cv2.waitKey(10)
                
                # Write images in destination
                bot_data_capture.images = images
                timestamp, name_l, name_c, name_r = write_images(
                    images=bot_data_capture.images, 
                    dest=sub_folder_path, 
                    quality=bot_data_capture.quality,
                    multiple_images=bot_data_capture.multiple_cameras)
                names = [name_l, name_c, name_r]

                # Structure: 'capture', 'timestamp','camera','filename', 'steering
                rows = [[bot_data_capture.capture_id, timestamp, cam_label, file_name, 
                    bot_data_capture.steering_angle] for file_name, cam_label in zip([name_l, 
                    name_c, name_r], ["L", "C", "R"])]

                # Write data and variables in csv file
                with open(bot_data_capture.csv_file, 'a') as fd:
                    writer = csv.writer(fd); writer.writerows(rows)

                first_time = False; msg.recording = True

                # Update frame rate
                bot_data_capture.fps = 1.0/(time.time()-start_time)
                fps = 1.0/(time.time()-start_time_local)

                # If time left for next capture then wait
                start_time = time.time()
                time2sleep = 1.0/bot_data_capture.desired_fps - 1.0/fps - 1.0/rate
                main_debugger.debugger(DEBUG_LEVEL_3, "Capture FPS: {}, time2sleep: {}".format(
                    bot_data_capture.fps, time2sleep), log_type='info')
                if time2sleep >= 0:
                    time.sleep(time2sleep)

        else:
            msg.recording = False; first_time = True
            bot_data_capture.fps = 0; time.sleep(0.1)

        capture_publisher.publish(msg)
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