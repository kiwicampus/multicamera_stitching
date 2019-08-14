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
    def __init__(self, camera = None):
        super(QMainWindow, self).__init__()
        self.camera = camera

        self.setWindowTitle("Media player Vision system")

        self.central_widget = QWidget()
        self.button_loadfolder = QPushButton('Load folder', self.central_widget)
        
        self.image_view = ImageView()
        self.image_view.ui.histogram.hide()
        self.image_view.ui.roiBtn.hide()
        self.image_view.ui.menuBtn.hide()

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0,100)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(10)

        self.layout = QVBoxLayout(self.central_widget)
        self.layout.addWidget(self.button_loadfolder)
        self.layout.addWidget(self.image_view)
        self.layout.addWidget(self.slider)
        self.setCentralWidget(self.central_widget)

        self.button_loadfolder.clicked.connect(self.load_files)
        self.slider.valueChanged.connect(self.update_image)

        self.files = []

        self.data_reader = data_reader()

    def load_files(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        options |= QFileDialog.DontUseCustomDirectoryIcons
        dialog = QFileDialog()
        dialog.setOptions(options)
        dialog.setFileMode(QFileDialog.DirectoryOnly)
        folder = dialog.getExistingDirectory(self, 'Select directory')
        if folder:
            print(folder)
            self.data_reader.load_data(folder)

            self.slider.setRange(0, len(self.data_reader.images[0][0]))
            self.slider.setTickPosition(QSlider.TicksBelow)
            self.slider.setTickInterval(int(len(self.data_reader.images[0][0])/10))

            image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[0][0][self.slider.value()])
            self.image_view.setImage(image[:,:,0])

    def update_image(self, value):
        if value < len(self.data_reader.images[0][0]):
            image = cv2.imread(self.data_reader.path+'/data/'+self.data_reader.images[0][0][value])
            self.image_view.setImage(image[:,:,0])
        else:
            print('Slider value is outside the range of images')

if __name__ == '__main__':
    app = QApplication([])
    window = StartWindow()
    window.show()
    app.exit(app.exec_())