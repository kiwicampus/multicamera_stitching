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

        self.button_loadfolder = QPushButton('Load folder', self.central_widget) # Defines button and attaches it to te central widget

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
        self.layout.addWidget(self.image_view) # Add image frame to layout
        self.layout.addWidget(self.slider) # Add horizontal slider to layout
        self.setCentralWidget(self.central_widget)

        self.button_loadfolder.clicked.connect(self.load_files) # Connects function self.load_files to the action clicked over button
        self.slider.valueChanged.connect(self.update_image) # Connects function self.update_image to action change in slider position 

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
            self.data_reader.load_data(folder) # load data from selected folder using the object data_reader

            self.slider.setRange(0, len(self.data_reader.images[0][0])) # Sets slider range according to dimensions of self.data_reader.images list 
            self.slider.setTickInterval(int(len(self.data_reader.images[0][0])/10)) # Positions ticks 1/10th of the total list length

            image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[0][0][self.slider.value()]) # Loads image 0
            self.image_view.setImage(image[:,:,0]) # Displays image in image frame
        

    def update_image(self, value):
        '''
        Updates image when moving the slider along the length of sequence of images
        for a given capture image set
        '''
        # If value is in the range of self.data_reader.images
        if value < len(self.data_reader.images[0][0]):
            image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[0][0][value]) # Load the image at the index value
            self.image_view.setImage(image[:,:,0]) # Displays image at index value in image frame
        else:
            print('Slider value is outside the range of images') # Value in slider is out of range of list indexes

    def next_capture(self):
        '''
        Access to next capture data structure. Updates the image frame with the first image of the capture for the first camera index.
        '''
        pass
    
    def previous_capture(self):
        '''
        Access to previous capture data structure. Updates the image frame with the first image of the capture for the first camera index.
        '''
        pass

    def next_camera(self):
        '''
        Access to next camera image set. Updates the image frame with the first image of the camera image set.
        '''
        pass
    
    def previous_camera(self):
        '''
        Access to next camera image set. Updates the image frame with the first image of the camera image set.
        '''
        pass

if __name__ == '__main__':
    app = QApplication([])
    window = StartWindow()
    window.show()
    app.exit(app.exec_())