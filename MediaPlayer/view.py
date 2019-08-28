from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, QObject, QRunnable, pyqtSlot, QThreadPool, QRect
from PyQt5.QtWidgets import QMainWindow, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QApplication, QSlider, QFileDialog, QLabel, QCheckBox
#from PyQt5 import QtQuick
from pyqtgraph import ImageView
import numpy as np
import os 
import cv2
from model import data_reader  
import time
import traceback, sys

from controller import calibration_utils
from Extrinsic import draw_extrinsic
from StitcherClass import Stitcher

class WorkerSignals(QObject):
    '''
    Defines the signals available from a running worker thread.

    Supported signals are:

    finished
        No data

    error
        `tuple` (exctype, value, traceback.format_exc() )

    result
        `object` data returned from processing, anything

    progress
        `int` indicating % progress 

    '''
    finished = pyqtSignal()
    error = pyqtSignal(tuple)
    result = pyqtSignal(object)
    progress = pyqtSignal(int)

class Worker(QRunnable):
    '''
    Worker thread

    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    :param callback: The function callback to run on this worker thread. Supplied args and 
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    '''

    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()

        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()    

        # Add the callback to our kwargs
        self.kwargs['progress_callback'] = self.signals.progress        

    @pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''
        
        # Retrieve args/kwargs here; and fire processing using them
        try:
            result = self.fn(*self.args, **self.kwargs)
        except:
            traceback.print_exc()
            exctype, value = sys.exc_info()[:2]
            self.signals.error.emit((exctype, value, traceback.format_exc()))
        else:
            self.signals.result.emit(result)  # Return the result of the processing
        finally:
            self.signals.finished.emit()  # Done

class StartWindow(QMainWindow):
    '''
    Main interface window class with the follow main elements:
        - Button to open data capture folder
        - Image frame widget to visualize images
        - Slider to transition images among the sequence of the capture
    '''
    def __init__(self):
        '''
        UI initialization with essential feature set for the media player
        '''
        super(QMainWindow, self).__init__()

        self.setWindowTitle("Media player Vision system")

        self.central_widget = QWidget() # Central widget of the window

        self.button_loadfolder = QPushButton('Load folder') # Defines button and attaches it to the central widget

        self.button_next_capture = QPushButton('Next capture')
        self.button_previous_capture = QPushButton('Prev capture')

        self.button_next_camera = QPushButton('Next camera') # Access to next camera image sequence for a given capture
        self.button_previous_camera = QPushButton('Prev camera') # Access to previous camera image sequence for a given capture
        
        self.button_play = QPushButton('Play/pause') # Reproduce a sequence of images for a given capture and camera
        
        self.info_label = QLabel()
        self.camera_number_label = QLabel()
        self.camera_label = QLabel()
        self.capture_label = QLabel()
        
        self.image_view = ImageView() # Image frame definition

        self.image_view.ui.histogram.hide() # Hides histogram from image frame
        self.image_view.ui.roiBtn.hide() # Hides roi button from image frame
        self.image_view.ui.menuBtn.hide() # Hides menu button from image frame

        self.slider = QSlider(Qt.Horizontal) # Horizontal slider definition
        self.slider.setRange(0,100) # Sets range of slider between 0 and 100
        self.slider.setTickPosition(QSlider.TicksBelow) # Position ticks below slider
        self.slider.setTickInterval(10) # Tick interval set to 10

        self.layout_left = QVBoxLayout() # Vertical layout definition
        
        self.layout_left.addWidget(self.info_label) # Information label with basic instructions
        self.layout_left.addWidget(self.button_loadfolder) # Add button load folder to layout
        self.layout_left.addWidget(self.button_next_capture) # Add button load folder to layout
        self.layout_left.addWidget(self.button_previous_capture) # Add button load folder to layout
        self.layout_left.addWidget(self.button_next_camera) # Add button load folder to layout
        self.layout_left.addWidget(self.button_previous_camera) # Add button load folder to layout
        self.layout_left.addWidget(self.button_play) # Add button play/pause to layout

        self.layout_left.addWidget(self.camera_number_label)
        self.layout_left.addWidget(self.camera_label)
        self.layout_left.addWidget(self.capture_label)

        self.layout_left.addWidget(self.image_view) # Add image frame to layout
        #self.layout_left.addWidget(self.slider) # Add horizontal slider to layout
        
        
        self.info_label.setWordWrap(True)
        self.info_label.setText("1) Load a data capture folder opening the Load Folder dialog. \n\n2) Then interact with different captures, cameras and sequence player using the buttons.")
        
        self.stitcher_label = QLabel()
        self.stitcher_view = ImageView() # Image frame definition

        self.checkbox_status_label = QLabel()
        self.stitcher_checkbox = QCheckBox()

        self.stitcher_view.ui.histogram.hide() # Hides histogram from image frame
        self.stitcher_view.ui.roiBtn.hide() # Hides roi button from image frame
        self.stitcher_view.ui.menuBtn.hide() # Hides menu button from image frame


        self.layout_right = QVBoxLayout()

        self.layout_right.addWidget(self.stitcher_label)
        self.layout_right.addWidget(self.checkbox_status_label)
        self.layout_right.addWidget(self.stitcher_checkbox)
        self.layout_right.addWidget(self.stitcher_view)
        
        self.stitcher_label.setWordWrap(True)
        self.stitcher_label.setText("Stitched image\n\n")
        
        self.checkbox_status_label.setWordWrap(True)
        self.checkbox_status_label.setText("Check to enable stitching")
   
        self.layout_h = QHBoxLayout()

        self.layout_h.addLayout(self.layout_left)
        self.layout_h.addLayout(self.layout_right)

        self.layout = QVBoxLayout(self.central_widget)
        self.layout.addLayout(self.layout_h)
        self.layout.addWidget(self.slider)
        self.setCentralWidget(self.central_widget)

        self.button_loadfolder.clicked.connect(self.load_files) # Connects function self.load_files to the action clicked over button loadfolder
        self.button_next_capture.clicked.connect(self.next_capture) # Connects function next_capture to action clicked over button 
        self.button_previous_capture.clicked.connect(self.previous_capture) # Connects function previous_capture to action clicked over button 
        self.button_next_camera.clicked.connect(self.next_camera)
        self.button_previous_camera.clicked.connect(self.previous_camera)
        self.button_play.clicked.connect(self.play_pause)
        self.slider.valueChanged.connect(self.update_image) # Connects function self.update_image to action change in slider position 

        self.stitcher_checkbox.setChecked(False)
        self.stitcher_checkbox.stateChanged.connect(self.check_stitching)

        self.button_next_capture.setEnabled(False)
        self.button_previous_capture.setEnabled(False)
        self.button_next_camera.setEnabled(False)
        self.button_previous_camera.setEnabled(False)
        self.button_play.setEnabled(False)
        self.slider.setEnabled(False)
        
        self.data_reader = data_reader() # Instantiation of data reader class

        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())

        self.inThread = False # True when thread of image sequence is running
        self.playing = False # True when play is active

        self.endThread = False # True when signal to end thread has been activated
        self.image_index = None # Used to define image index in current reproduction thread

        self.calibrator = calibration_utils()
        self.calibrator.load_calibration()
        self.local_intrinsic = True

        self.cam_labels = None 

        self.stitcher_ready = False

    def load_files(self):
        '''
        This function loads all the images from a data capture folder using
        a data_reader object and stores the data in a customized data structure (3-d list).
        The dimensionality of this array is used to scale the extend of the capture index, 
        camera index and the slider (timestamp sequence).  
        '''
        # Options definition for QFileDialog
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        options |= QFileDialog.DontUseCustomDirectoryIcons
        dialog = QFileDialog() # File dialog object instantiation
        dialog.setOptions(options) # Assign options to QFileDialog
        dialog.setFileMode(QFileDialog.DirectoryOnly) # Set file dialog to directory search only
        
        folder = dialog.getExistingDirectory(self, 'Select directory') # Assigns the variable folder when path has been defined in the dialog
        if folder:
            print(folder)
            try:
                if self.inThread:
                    self.endThread = True
                
                self.data_reader.load_data(folder) # load data from selected folder using the object data_reader

                self.cam_labels =  dict([(value, key) for key, value in self.data_reader.camera_labels.items()])

                self.slider.setRange(0, len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets slider range according to dimensions of self.data_reader.images list 
                self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Positions ticks 1/10th of the total list length

                self.image_index = self.slider.value()

                self.show_image(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()])

                self.camera_number_label.setText("There are "+str(len(self.data_reader.camera_labels))+" cameras in the current capture.")
                self.capture_label.setText("Capture "+str((self.data_reader.current_capture)+1))
                self.camera_label.setText("Camera "+str((2-self.data_reader.current_camera)+1))

                self.button_next_capture.setEnabled(True)
                self.button_previous_capture.setEnabled(True)
                self.button_next_camera.setEnabled(True)
                self.button_previous_camera.setEnabled(True)
                self.button_play.setEnabled(True)
                self.slider.setEnabled(True)
            except:
                print('Error reading folder, verify that folder includes csv file and belongs to a datacapture type.')
            
        

    def update_image(self, value):
        '''
        Updates image when moving the slider along the length of sequence of images
        for a given capture image set
        '''
        # If value is in the range of self.data_reader.images
        if value < len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera]):
            self.show_image(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][value])
            self.image_index = value
        else:
            print('Slider value is outside the range of images') # Value in slider is out of range of list indexes

    def next_capture(self):
        '''
        Access to next capture data structure. Updates the image frame with the first image of the capture for the first camera index.
        '''
        self.data_reader.current_capture += 1
        if (self.data_reader.current_capture > (len(self.data_reader.images)-1)):
            self.data_reader.current_capture = 0
        
        self.data_reader.current_camera = len(self.data_reader.camera_labels) - 1

        self.camera_number_label.setText("There are "+str(len(self.data_reader.camera_labels))+" in the current capture.")
        self.capture_label.setText("Capture "+str((self.data_reader.current_capture)+1))
        self.camera_label.setText("Camera "+str((2-self.data_reader.current_camera)+1))

        if self.inThread:
            self.endThread = True

        self.slider.setRange(0,len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets range of slider between 0 and 100
        self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Tick interval set to 10
        self.slider.setValue(0)
        self.image_index = 0

        self.show_image(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()])

    
    def previous_capture(self):
        '''
        Access to previous capture data structure. Updates the image frame with the first image of the capture for the first camera index.
        '''
        self.data_reader.current_capture -= 1
        if (self.data_reader.current_capture < 0):
            self.data_reader.current_capture = len(self.data_reader.images)-1
        
        self.data_reader.current_camera = len(self.data_reader.camera_labels) - 1

        self.camera_number_label.setText("There are "+str(len(self.data_reader.camera_labels))+" in the current capture.")
        self.capture_label.setText("Capture "+str((self.data_reader.current_capture)+1))
        self.camera_label.setText("Camera "+str((2-self.data_reader.current_camera)+1))

        if self.inThread:
            self.endThread = True

        self.slider.setRange(0,len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets range of slider between 0 and 100
        self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Tick interval set to 10
        self.slider.setValue(0)
        self.image_index = 0

        self.show_image(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()])

    def next_camera(self):
        '''
        Access to next camera image set. Updates the image frame with the first image of the camera image set.
        '''
        self.data_reader.current_camera -= 1
        if (self.data_reader.current_camera < 0):
            self.data_reader.current_camera = len(self.data_reader.camera_labels)-1  

        self.camera_label.setText("Camera "+str((2-self.data_reader.current_camera)+1))
        
        if not(self.inThread):
            self.slider.setRange(0,len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets range of slider between 0 and 100
            self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Tick interval set to 10
            self.slider.setValue(0)

            self.show_image(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()])
            self.image_index =  self.slider.value()
        
        # When a reproduction thread is running and pause is active, update image with new camera index
        if (self.inThread and not(self.playing)):
            self.show_image(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.image_index])

    
    def previous_camera(self):
        '''
        Access to next camera image set. Updates the image frame with the first image of the camera image set.
        '''
        self.data_reader.current_camera += 1
        if (self.data_reader.current_camera > (len(self.data_reader.camera_labels)-1)):
            self.data_reader.current_camera = 0

        self.camera_label.setText("Camera "+str((2-self.data_reader.current_camera)+1))

        if not(self.inThread):
            self.slider.setRange(0,len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets range of slider between 0 and 100
            self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Tick interval set to 10
            self.slider.setValue(0)

            self.show_image(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()])
            self.image_index =  self.slider.value()
        
        # When a reproduction thread is running and pause is active, update image with new camera index
        if (self.inThread and not(self.playing)):
            self.show_image(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.image_index])

    def show_image(self, image_path):
        image = cv2.imread(image_path) # Loads image 
        image = cv2.flip(image, 0)
        
        # If intrinsic calibration available
        if self.calibrator.intrinsic_calibration["mtx"] is not None and self.local_intrinsic:
            # Undistord the image
            image=cv2.undistort(src=image, cameraMatrix=self.calibrator.intrinsic_calibration["mtx"], 
                distCoeffs=self.calibrator.intrinsic_calibration["dist"])
            #print('Intrinsic has been applied!')
            # If extrinsic calibration available 

            current_camera_label = self.cam_labels[self.data_reader.current_camera]
            if self.calibrator.extrinsic_calibrations[current_camera_label]["M"] is not None:
                image = cv2.warpPerspective(src=image, M=self.calibrator.extrinsic_calibrations[current_camera_label]["M"] ,
                    dsize=self.calibrator.extrinsic_calibrations[current_camera_label]["dst_size"])
                #draw_extrinsic(img_src=image, src_pts=self.calibrator.extrinsic_calibrations[current_camera_label]["src_pts"])
                #print('Extrinsic has been applied!')
        
        self.image_view.setImage(image[:,:,0].T, autoRange= True) # Displays image in image frame
        if self.stitcher_checkbox.isChecked()!=0:
            self.show_stitcher()

    def show_stitcher(self):

        images_dic = {self.cam_labels[i]:self.data_reader.images[self.data_reader.current_capture][i][self.image_index] for i in self.cam_labels.keys()}
        if not(self.stitcher_ready):
            # Stitcher variables
            stitcher_conf_path=save_path=os.path.join(os.path.dirname(os.getenv(
                key="CAM_PORTS_PATH")), 'Stitcher_config.pkl')
            self.camera_stitcher =  Stitcher(images_dic = images_dic, super_mode = False)
            self.camera_stitcher = self.camera_stitcher.load_stitcher(load_path = stitcher_conf_path) 
            self.stitcher_ready = True
            print('Stitcher file has been loaded!')
        #print(images_dic)
        for key in images_dic.keys():
            images_dic[key] = cv2.flip( cv2.imread(self.data_reader.path+'/data/'+images_dic[key]), 0)[:,:,0].T

        stitcher_img = self.camera_stitcher.stitch(images_dic=images_dic)
        #print('Shape: ', stitcher_img.shape)
        self.stitcher_view.setImage(stitcher_img, autoRange= True)
        

    def progress_fn(self, n):
        '''
        Information of progress of the thread
        '''
        print("%d%%" % n)

    def print_output(self,s):
        '''
        Print results of different operations in the thread
        '''
        print(s)

    def thread_complete(self):
        '''
        Method called when the thread has been completed
        '''
        print("Sequence reproduction complete!")
        self.inThread = False
        self.playing = False
        self.endThread = False

        # Enables slider after video reproduction
        self.slider.setEnabled(True)

    def logic_play_pause(self,rate, progress_callback):
        '''
        Main play and pause logic used to show a sequence of images at a give rate
        '''
        self.image_index = 0
        total_images = len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])
        while (self.image_index < total_images):
            # When play function is enabled, play the sequence, otherwise do nothing
            if (self.endThread):
                break
            else:
                if (self.playing):

                    self.show_image(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.image_index])
                    self.slider.setValue(self.image)

                    time.sleep(rate) # Rate of reproduction of image sequence
                    progress_callback.emit(self.image_index*100.0/total_images) # Emit value of sequence progress to callback function
                    self.image_index += 1 # Increase image index to read next image in sequence
                else:
                    time.sleep(0.2)

    def play_pause(self):
        '''
        Play the sequence of images for a capture and camera at a given time rate 
        This function creates another thread where the sequence of images will be reproduced
        '''
        self.playing = not(self.playing) # Toggles self.playing flag

        if not(self.inThread):
            worker = Worker(self.logic_play_pause, 0.016) # Any other args, kwargs are passed to the run function
            worker.signals.result.connect(self.print_output)
            worker.signals.finished.connect(self.thread_complete)
            worker.signals.progress.connect(self.progress_fn)

            # Execute
            self.threadpool.start(worker)
            self.inThread = True # Set thread in active state

            # Disables slider when video is being reproduced
            #self.slider.setEnabled(False)

    def check_stitching(self, value):
        if value != 0:
            print('Checkbox has been checked!')
            print('These are camera labels: ', self.cam_labels.keys())
            print('Camera index: ', self.data_reader.current_camera)
            print('Image index: ', self.image_index)
            
            self.show_stitcher()
        else:
            image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.image_index]) # Loads image
            image = image[:,:,0].T
            shape = image.shape
            dummy = np.zeros(shape)
            self.stitcher_view.setImage(dummy)
            print('Check is unchecked!')

if __name__ == '__main__':
    app = QApplication([])
    window = StartWindow()
    window.show()
    app.exit(app.exec_())