import sys
import os 
import time
import traceback

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import Qt, QThread, QTimer, pyqtSignal, QObject, QRunnable, pyqtSlot, QThreadPool, QRect
from PyQt5.QtWidgets import QMainWindow, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QApplication, QSlider, QFileDialog, QLabel, QCheckBox

from decimal import Decimal

import numpy as np
import cv2

from model import data_reader
from controller import calibration_utils
from Extrinsic import draw_extrinsic
from StitcherClass import Stitcher

qtcreator_file  = "ui/player.ui" # Enter file here.
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtcreator_file)


class MyWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    '''
    Main interface window class with the follow main elements:
        - Button to open data capture folder
        - Image frame widget to visualize images
        - Slider to transition images among the sequence of the capture
        - Next/previous camera buttons to navigate among camera sequences
        - Next/previous capture buttons to navigate among capture sequences
    '''
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.i = 35

        rMyIcon = QtGui.QPixmap("icons/usb.png");
        self.button_loadfolder.setIcon(QtGui.QIcon(rMyIcon))
        self.button_loadfolder.setIconSize(QtCore.QSize(self.i,self.i))

        prevMyIcon = QtGui.QPixmap("icons/previous-1.png");
        self.button_previous_capture.setIcon(QtGui.QIcon(prevMyIcon))
        self.button_previous_capture.setIconSize(QtCore.QSize(self.i,self.i))

        self.button_previous_camera.setIcon(QtGui.QIcon(prevMyIcon))
        self.button_previous_camera.setIconSize(QtCore.QSize(self.i,self.i))

        nextMyIcon = QtGui.QPixmap("icons/skip-1.png");
        self.button_next_capture.setIcon(QtGui.QIcon(nextMyIcon))
        self.button_next_capture.setIconSize(QtCore.QSize(self.i,self.i))

        self.button_next_camera.setIcon(QtGui.QIcon(nextMyIcon))
        self.button_next_camera.setIconSize(QtCore.QSize(self.i,self.i))
        
        
        playMyIcon = QtGui.QPixmap("icons/play.png")
        pauseMyIcon = QtGui.QPixmap("icons/pause.png")
        self.play_pause_icons = [QtGui.QPixmap("icons/pause.png") , QtGui.QPixmap("icons/play.png")]
        self.button_play.setIcon(QtGui.QIcon(playMyIcon))
        self.button_play.setIconSize(QtCore.QSize(self.i,self.i))

        self.slider.setRange(0,100) # Sets range of slider between 0 and 100
        self.slider.setTickPosition(QSlider.TicksBelow) # Position ticks below slider
        #self.slider.setTickInterval(10) # Tick interval set to 10

        self.button_play.clicked.connect(self.toggle_icons)
        self.play = True

        self.stitcher_view.view.setImage('sharingan.jpg')
        self.stitcher_view.view.pixmap_enabled = True
        self.stitcher_view.view.setMouseTracking(True)

        self.image_view.view.setImage('sharingan.jpg')

        self.button_loadfolder.clicked.connect(self.load_files) # Connects function self.load_files to the action clicked over button loadfolder
        self.slider.valueChanged.connect(self.update_image) # Connects function self.update_image to action change in slider position 
        self.button_next_capture.clicked.connect(self.next_capture) # Connects function next_capture to action clicked over button 
        self.button_previous_capture.clicked.connect(self.previous_capture) # Connects function previous_capture to action clicked over button 
        self.button_next_camera.clicked.connect(self.next_camera)
        self.button_previous_camera.clicked.connect(self.previous_camera)

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

        #self.calibrator = calibration_utils()
        #self.calibrator.load_calibration()
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
                try:
                    self.data_reader.load_data(folder) # load data from selected folder using the object data_reader
                except:
                    print("Error creating data reader object.")
                self.cam_labels =  dict([(value, key) for key, value in self.data_reader.camera_labels.items()])

                self.slider.setRange(0, len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets slider range according to dimensions of self.data_reader.images list 
                self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Positions ticks 1/10th of the total list length

                self.image_index = self.slider.value()
                
                print('Index values: ', self.data_reader.current_capture, self.data_reader.current_camera, self.image_index)
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

    def show_image(self, image_path):
        print('Showing image: ', image_path)
        self.image_view.view.setImage(image_path)
        '''
        # If intrinsic calibration available
        if self.calibrator.intrinsic_calibration["mtx"] is not None and self.local_intrinsic:
            # Undistord the image
            image=cv2.undistort(src=image, cameraMatrix=self.calibrator.intrinsic_calibration["mtx"], 
                distCoeffs=self.calibrator.intrinsic_calibration["dist"])

            current_camera_label = self.cam_labels[self.data_reader.current_camera]
            # If extrinsic calibration available
            if self.calibrator.extrinsic_calibrations[current_camera_label]["M"] is not None:
                image = cv2.warpPerspective(src=image, M=self.calibrator.extrinsic_calibrations[current_camera_label]["M"] ,
                    dsize=self.calibrator.extrinsic_calibrations[current_camera_label]["dst_size"])
                #draw_extrinsic(img_src=image, src_pts=self.calibrator.extrinsic_calibrations[current_camera_label]["src_pts"])
        
        self.image_view.setImage(image[:,:,0].T) # Displays image in image frame
        if self.stitcher_checkbox.isChecked()!=0:
            self.show_stitcher()
        '''

    def toggle_icons(self):
        self.play = not(self.play)
        self.button_play.setIcon(QtGui.QIcon(self.play_pause_icons[int(self.play)]))
        self.button_play.setIconSize(QtCore.QSize(self.i,self.i))

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())