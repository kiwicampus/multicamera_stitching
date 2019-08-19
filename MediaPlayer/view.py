from PyQt5.QtCore import Qt, QThread, QTimer
from PyQt5.QtWidgets import QMainWindow, QWidget, QPushButton, QVBoxLayout, QApplication, QSlider, QFileDialog
from pyqtgraph import ImageView
import numpy as np
import os 
import cv2
from model import data_reader  

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

        self.button_loadfolder = QPushButton('Load folder', self.central_widget) # Defines button and attaches it to the central widget

        self.button_next_capture = QPushButton('Next capture', self.central_widget)
        self.button_previous_capture = QPushButton('Prev capture', self.central_widget)

        self.button_next_camera = QPushButton('Next camera', self.central_widget) # Access to next camera image sequence for a given capture
        self.button_previous_camera = QPushButton('Prev camera', self.central_widget) # Access to previous camera image sequence for a given capture

        self.image_view = ImageView() # Image frame definition
        self.image_view.ui.histogram.hide() # Hides histogram from image frame
        self.image_view.ui.roiBtn.hide() # Hides roi button from image frame
        self.image_view.ui.menuBtn.hide() # Hides menu button from image frame

        self.slider = QSlider(Qt.Horizontal) # Horizontal slider definition
        self.slider.setRange(0,100) # Sets range of slider between 0 and 100
        self.slider.setTickPosition(QSlider.TicksBelow) # Position ticks below slider
        self.slider.setTickInterval(10) # Tick interval set to 10

        self.layout = QVBoxLayout(self.central_widget) # Vertical layout definition
        self.layout.addWidget(self.button_loadfolder) # Add button load folder to layout
        self.layout.addWidget(self.button_next_capture) # Add button load folder to layout
        self.layout.addWidget(self.button_previous_capture) # Add button load folder to layout
        self.layout.addWidget(self.button_next_camera) # Add button load folder to layout
        self.layout.addWidget(self.button_previous_camera) # Add button load folder to layout
        self.layout.addWidget(self.image_view) # Add image frame to layout
        self.layout.addWidget(self.slider) # Add horizontal slider to layout
        self.setCentralWidget(self.central_widget)

        self.button_loadfolder.clicked.connect(self.load_files) # Connects function self.load_files to the action clicked over button loadfolder
        self.button_next_capture.clicked.connect(self.next_capture) # Connects function next_capture to action clicked over button 
        self.button_previous_capture.clicked.connect(self.previous_capture) # Connects function previous_capture to action clicked over button 
        self.button_next_camera.clicked.connect(self.next_camera)
        self.button_previous_camera.clicked.connect(self.previous_camera)
        self.slider.valueChanged.connect(self.update_image) # Connects function self.update_image to action change in slider position 

        self.button_next_capture.setEnabled(False)
        self.button_previous_capture.setEnabled(False)
        self.button_next_camera.setEnabled(False)
        self.button_previous_camera.setEnabled(False)

        self.data_reader = data_reader() # Instantiation of data reader class

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
                self.data_reader.load_data(folder) # load data from selected folder using the object data_reader

                self.slider.setRange(0, len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets slider range according to dimensions of self.data_reader.images list 
                self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Positions ticks 1/10th of the total list length

                image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()]) # Loads image 0
                self.image_view.setImage(image[:,:,0]) # Displays image in image frame

                self.button_next_capture.setEnabled(True)
                self.button_previous_capture.setEnabled(True)
                self.button_next_camera.setEnabled(True)
                self.button_previous_camera.setEnabled(True)
            except:
                print('Error reading folder, verify that folder includes csv file and belongs to a datacapture type.')
            
        

    def update_image(self, value):
        '''
        Updates image when moving the slider along the length of sequence of images
        for a given capture image set
        '''
        # If value is in the range of self.data_reader.images
        if value < len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera]):
            image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][value]) # Load the image at the index value
            self.image_view.setImage(image[:,:,0]) # Displays image at index value in image frame
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

        self.slider.setRange(0,len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets range of slider between 0 and 100
        self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Tick interval set to 10
        self.slider.setValue(0)

        image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()]) # Loads image 0
        self.image_view.setImage(image[:,:,0]) # Displays image in image frame

    
    def previous_capture(self):
        '''
        Access to previous capture data structure. Updates the image frame with the first image of the capture for the first camera index.
        '''
        self.data_reader.current_capture -= 1
        if (self.data_reader.current_capture < 0):
            self.data_reader.current_capture = len(self.data_reader.images)-1
        
        self.data_reader.current_camera = len(self.data_reader.camera_labels) - 1

        self.slider.setRange(0,len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets range of slider between 0 and 100
        self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Tick interval set to 10
        self.slider.setValue(0)

        image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()]) # Loads image 0
        self.image_view.setImage(image[:,:,0]) # Displays image in image frame

    def next_camera(self):
        '''
        Access to next camera image set. Updates the image frame with the first image of the camera image set.
        '''
        self.data_reader.current_camera -= 1
        if (self.data_reader.current_camera < 0):
            self.data_reader.current_camera = len(self.data_reader.camera_labels)-1  

        self.slider.setRange(0,len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets range of slider between 0 and 100
        self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Tick interval set to 10
        self.slider.setValue(0)

        image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()]) # Loads image 0
        self.image_view.setImage(image[:,:,0]) # Displays image in image frame
    
    def previous_camera(self):
        '''
        Access to next camera image set. Updates the image frame with the first image of the camera image set.
        '''
        self.data_reader.current_camera += 1
        if (self.data_reader.current_camera > (len(self.data_reader.camera_labels)-1)):
            self.data_reader.current_camera = 0

        self.slider.setRange(0,len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])-1) # Sets range of slider between 0 and 100
        self.slider.setTickInterval(int(len(self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera])/10)) # Tick interval set to 10
        self.slider.setValue(0)

        image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[self.data_reader.current_capture][self.data_reader.current_camera][self.slider.value()]) # Loads image 0
        self.image_view.setImage(image[:,:,0]) # Displays image in image frame

if __name__ == '__main__':
    app = QApplication([])
    window = StartWindow()
    window.show()
    app.exit(app.exec_())