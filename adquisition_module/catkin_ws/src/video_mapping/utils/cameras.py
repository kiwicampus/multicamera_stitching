# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Phone: +57 (350) 283 51 22
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer Vision Team

Supported Camera Resolutions: 
    Aspect ratio 4:3  -> (240, 320), (480, 640), (600, 800)
    Aspect ratio 16:9 -> (180, 320), (360, 640), (720, 1280), (1080, 1920)

Camera thread object that reads in a thread a camera and handles errors
Constructor: int camera_number: physical device number: ex. if 1 , means /dev/video1
             int height, int width: desired resolution
             string camera_id: local name of the camera: ex: "LL"
             np.array mtx, dist: camera matrix and distortion coefficients of intrinsic calibration
             bool start_reading: if thread starts reading right away images when thread is started
             bool camera_enabled: if thread opens or not physical device
"""

# =============================================================================
import numpy as np
import time
import yaml
import os

import rospy
import cv2
import io

import subprocess
import re

from threading import Thread, Event

from extended_rospylogs import Debugger, update_debuggers, loginfo_cond, logerr_cond
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

from video_mapping.srv import GetMoreCameraStatus
from video_mapping.srv import GetMoreCameraStatusResponse

# =============================================================================
# Video cameras ports
def read_cam_ports(file_path="configs/cam_ports.yaml"):
    abs_path = os.path.join(os.path.dirname(__file__), file_path)
    if os.path.isfile(abs_path):
        with open(abs_path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
            return data_loaded
    else:
        default_ports = {'C': '1.0.0', 'L': '1.0.0', 'R': '1.0.0'}
        with io.open(abs_path, 'w', encoding='utf8') as outfile:
            yaml.dump(default_ports, outfile, default_flow_style=False, allow_unicode=True)
        return default_ports

PORTS = read_cam_ports()

# =============================================================================
class CameraHandler(Thread, Debugger):

    def __init__(self, camera_number, height, width, camera_id, mtx, dist, start_reading=True, camera_enabled=True):
        """ Initializes camera Handler thread 
        Args:
            camera_number: `int` camera port number
            height: `int` video camera height
            width: `int` video camera width
            camera_id: `int` camera identifier
            mtx: `numpy.narray` camera's distortion matrix
            dist: `numpy.narray` camera's distortion vector
            start_reading: `boolean` Enable/Disable image start reading
            camera_enabled: `boolean` Enable/Disable camera reading
        Returns:
        """

        # start legacy component
        super(CameraHandler, self).__init__() 

        self.grab_frames = start_reading # Enable/Disable image reading
        self.camera_number = camera_number # Camera port number
        self.video_device = "/dev/video" + str(camera_number) # Camera video device
        self.height = height # Video height
        self.width = width # Video width
        self.camera_id = camera_id # Camera label

        self.debug_str = "OK" # string used to expose camera condition in service get_camera_status_verbose
        self.camera_enabled = camera_enabled # specifies if even it tries to read the camera

        self.image = None # Camera Image
        self.grabbed = None # Camera status

        # Thread variables
        self.run_event = Event()
        self.run_event.set()
        self.daemon = True
        self.video_handler = None

        # If camera is enable start video thread
        if self.camera_enabled:
            
            # Start camera port
            self.init_video_handler() 

        # If camera is not enable then report any issues
        else:
            if self.camera_number is None: # If camera was not found
                self.debug_str = "[ERROR]: NO {} CAMERA FOUND!".format(self.camera_id)
            else: # If camera not initialized by environment variables
                self.debug_str = "[WARNING]:Camera {} found ({}), but not activated by software (check DATA_CAPTURE or REAR_CAMERA_ENABLED env vars)".format(
                    self.camera_id, self.video_device)
            self.set_error_image("[ERROR]: READ {} IMAGE ERROR!".format(self.camera_id))

    def init_video_handler(self):
        """ InitializeS video handler
        Args:
        Returns:
        """

        # Open camera port
        self.video_handler = cv2.VideoCapture(self.video_device)

        # Set desired size to camera handler
        # Some OpenCV versions requirers assing the size twice -Dont know why
        self.set_video_size()

        # opens successfully the camera
        if self.video_handler.isOpened(): 

            self.set_video_size()
    
            self.debugger(DEBUG_LEVEL_0, "[INFO]: Loading {} ({}) first image...".format(
                self.camera_id, self.video_device))

            try: # Grab frame from camera and assing properties
                self.grabbed, self.image = self.video_handler.read()
                self.height, self.width = (
                    int(self.video_handler.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                    int(self.video_handler.get(cv2.CAP_PROP_FRAME_WIDTH)))

            # If not possible open camera port print exception 
            except Exception as e:
                self.debugger(DEBUG_LEVEL_0,
                    "[ERROR]: Something error ocurred reading frames from camera {} (device {}) -> {}".format(
                        self.camera_id, self.video_device, e), log_type = 'err')
                self.grabbed = False

        # If opening camera process did not succeed then report the error
        if not self.video_handler.isOpened() or not self.grabbed:
            if self.camera_number is not None:
                self.debug_str = "[ERROR]: Camera {} on {} could not be opened, may have been disconnected or 'No space left on device' error".format(
                    self.camera_id, self.video_device)
                self.debugger(DEBUG_LEVEL_0, self.debug_str, log_type = 'err')
            else:
                self.debug_str = "NO {} CAMERA FOUND!".format(self.camera_id)
            self.video_handler = None
            self.set_error_image("NO {} CAMERA FOUND!".format(self.camera_id))
    
    def set_video_size(self):
        """ Assigns video handler's size
        Args:
        Returns:
        """

        self.video_handler.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.video_handler.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)

    def set_error_image(self, error_msg):
        """ Set in camera thread image a default error image
        Args:
            error_msg: `string` error message to print on image
        Returns:
        """
        self.image = np.zeros((self.height, self.width, 3 ), dtype=np.uint8)
        font_size = 0.0015625*self.width + 0.5 #empirical formula so that: when w=6400 -> font=1.5 and w=1920 -> font=3.5
        cv2.putText(self.image, error_msg, (self.width//8, self.height//2),
                    cv2.FONT_HERSHEY_SIMPLEX, font_size, (255,255,255), 4)

    def run(self):
        if self.video_handler is not None: 
            while self.run_event.is_set():
                if self.grab_frames:
                    try:
                        tic = time.time()
                        self.grabbed, self.image = self.video_handler.read()
                    
                    except Exception as e: #possible "selec timeout" error
                        self.debug_str = "Error, possible select timeout on camera {} (device {}) -> {}".format(self.camera_id, self.video_device, e)
                        self.debugger(DEBUG_LEVEL_0,
                            "Some error ocurred reading frames from camera {} (device {}) -> {}".format(self.camera_id, self.video_device, e),
                            log_type = 'err')
                        self.set_error_image("READ {} IMAGE ERROR!".format(self.camera_id))
                        break

                    if not self.grabbed: #hot Unplugged error
                        self.debug_str = "Hot unplugged {} (device {})".format(self.camera_id, self.video_device)
                        self.debugger(DEBUG_LEVEL_0,
                            "Some error ocurred reading frames from camera {} (device {})".format(self.camera_id, self.video_device),
                            log_type = 'err')
                        self.set_error_image("READ {} IMAGE ERROR!".format(self.camera_id))
                        self.video_handler = None
                        raise Exception("Camera {} HOT Unplugged!".format(self.camera_id))

                    fps = 1.0/(time.time() - tic)
                    self.debugger(DEBUG_LEVEL_4,
                        "Grabbed FPS {} (device {}): {}".format(self.camera_id, 
                        self.video_device,fps))
                    
                    if (fps < 0.5):
                        self.debugger(DEBUG_LEVEL_0,
                            "Posible select timeout issue with camera {} (device {})".format(
                                self.camera_id, self.video_device),
                            log_type = 'warn')
                        self.video_handler.release() #reset video handler to maybe fix the selectimeout error
                        time.sleep(1)
                        self.init_video_handler()
                else:
                    time.sleep(0.03) # wait as if a frame were read
            
            if self.video_handler is not None:
                self.video_handler.release()
                self.video_handler = None

class CamerasSupervisorBase(Debugger):

    def __init__(self, mtx, dist, CameraClass=CameraHandler, 
        cameras_labels=["L", "C", "R"]):

        # ---------------------------------------------------------------------
        # start legacy components
        super(CamerasSupervisorBase, self).__init__()

        # Set cameras labels
        self.cameras_labels = cameras_labels

        # Get video/image dimensions
        self.video_height = int(os.environ.get("VIDEO_HEIGHT", 360))
        self.video_width = int(os.environ.get("VIDEO_WIDTH", 640))

        # Get available connected cameras
        self.usb_ports_cameras = self.find_cameras()

        # Get video device number
        self.video_numbers = list(map(lambda desired_camera: self.get_video_device(
            desired_camera, self.usb_ports_cameras), self.cameras_labels))
        self.debugger(DEBUG_LEVEL_0, "[INFO]: Video device numbers detected: {}".format(
            self.video_numbers))
        
        # ---------------------------------------------------------------------
        # Set process variables
        self.num_cameras = len(self.cameras_labels)
        initiate_cameras = [True]*self.num_cameras
        enabled_cameras = [True]*self.num_cameras

        # ---------------------------------------------------------------------
        # Initializes camera handler threads objects
        
        self.camera_handlers = [CameraClass(
            video, self.video_height, self.video_width, camera_id, mtx, dist,
            start_reading=start, camera_enabled=camera_enabled) 
            for video, start, camera_id, camera_enabled in zip(
                self.video_numbers, initiate_cameras, self.cameras_labels, enabled_cameras)]

        # Check camera status
        self.cameras_status, _ = self.check_cameras(self.camera_handlers, self.cameras_labels)

        video_handler = self.get_valid_video_handler(self.camera_handlers)

        if video_handler is not None:
            self.shape = (int(video_handler.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                          int(video_handler.get(cv2.CAP_PROP_FRAME_WIDTH)),
                          3*(self.num_cameras+1))
            self.video_height, self.video_width = int(self.shape[0]), int(self.shape[1])
        else: #there are not valid video streaming... send just garbage
            self.shape = (self.video_height, self.video_width, 3*(self.num_cameras+1))

        self.debugger(DEBUG_LEVEL_0, "[INFO]: Shape of video mapping array: {}".format(self.shape))

        # Create services to report cameras status
        rospy.Service('video_mapping/get_cameras_status_verbose', GetMoreCameraStatus, self.get_cameras_status_verbose)
    
    def find_cameras(self):
        """ Finds the camera numbers and ports that correspond to real cameras 
            Note: (devices may be repeated)
        Args:
        Returns:
            usb_ports_cameras" `list` list with video devices ports and numbers
        """

        # Finds and lists video devices numbers
        cams = self.find_video_devices()

        # Finds and lists video devices ports
        avail_ports = self.find_usb_ports(cams)

        # Zip video devices ports and numbers
        usb_ports_cameras = dict()
        for port, cam in zip(avail_ports, cams):
            if not port in usb_ports_cameras:
                usb_ports_cameras[port] = cam

        # list with video devices ports and numbers
        return usb_ports_cameras

    def find_video_devices(self):
        """ Finds and lists video devices numbers
        Args:
        Returns:
            cams_list: `list` with video camera devices numbers
        """

        # Check for video devices
        p = re.compile(r".+video(?P<video>\d+)$", re.I)
        devices = subprocess.check_output("ls -l /dev", shell=True).decode('utf-8')
        avail_cameras = []

        for device in devices.split('\n'):
            if device:
                info = p.match(device)
                if info:
                    dinfo = info.groupdict()
                    avail_cameras.append(dinfo["video"])
        cams_list = list(sorted(map(int,avail_cameras)))

        return cams_list

    def find_usb_ports(self, cameras):
        """ Finds and lists video devices ports
        Args:
            cameras: `list` with video camera devices numbers
        Returns:
            avail_ports: `list` with video camera devices ports
        """
        
        # find usb port given a video number device
        avail_ports = []
        p = re.compile(r"\d-(?P<video>[0-9.]+).+", re.I)

        for cam in cameras:
            try:
            # List of physical ports used in /dev/video# (some devices maybe represent same thing)
                path = subprocess.check_output("udevadm info --query=path --name=/dev/video" + str(cam), shell=True).decode('utf-8')

            except Exception as e:
                self.debugger(DEBUG_LEVEL_0,
                    "----- ERROR READING VIDEO DEVICE ----- (Error: {})".format(e),
                    log_type = 'err')
                avail_ports.append('None-{}'.format(cam))
                continue

            self.debugger(DEBUG_LEVEL_0, path.strip('\n'))
            path = path[0:path.find("video4linux")].split('/')[-2] #get last item where the port is explicit
            info = p.match(path) #get actual address
            if info:
                dinfo = info.groupdict()
                avail_ports.append(dinfo["video"])

        return avail_ports

    def get_video_device(self, desired_camera, usb_ports_cameras):     
        """ returns video device number 
        Args:
            desired_camera: `type` description
            usb_ports_cameras: `type` description
        Returns:
            `list` list with video devices ports and numbers
        """
        
        port = PORTS[desired_camera]
        if not port in usb_ports_cameras:
            return None
        else:
            return usb_ports_cameras[port]

    def check_cameras(self, camera_handlers, camera_tags, stdout=True):
        status = []
        status_str = []
        for camera, tag in zip(camera_handlers, camera_tags):
            if camera.video_handler is None:
                status.append(False)
                if stdout:
                    self.debugger(DEBUG_LEVEL_0,
                        "Camera {} is not connected or not being recognized".format(tag),
                        log_type = 'warn')
            else:
                status.append(True)
            status_str.append(camera.debug_str)
        return status, status_str

    def get_valid_video_handler(self, camera_handlers):
        for camera in camera_handlers:
            # center camera may have zoom so that resolution does not work
            if camera.video_handler is not None and camera.camera_id != "C": 
                return camera.video_handler
        return None

    def get_cameras_status_verbose(self, data):
        self.cameras_status, debug_str = self.check_cameras(self.camera_handlers, self.cameras_labels, stdout=False)
        return GetMoreCameraStatusResponse(debug_str)

class CamerasSupervisor(CamerasSupervisorBase, Debugger):

    def __init__(self, *args, **kwargs):

        super(CamerasSupervisor, self).__init__(*args, **kwargs)

        map(lambda o: o.start(), self.camera_handlers)

# =============================================================================