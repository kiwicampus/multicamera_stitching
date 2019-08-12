from PyQt5.QtCore import Qt, QThread, QTimer
from PyQt5.QtWidgets import QMainWindow, QWidget, QPushButton, QVBoxLayout, QApplication, QSlider
from pyqtgraph import ImageView
import numpy as np

class StartWindow(QMainWindow):
    def __init__(self, camera = None):
        super(QMainWindow, self).__init__()
        self.camera = camera

        self.central_widget = QWidget()
        self.button_loadfolder = QPushButton('Load folder', self.central_widget)
        
        self.image_view = ImageView()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0,100)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(10)

        self.layout = QVBoxLayout(self.central_widget)
        self.layout.addWidget(self.button_loadfolder)
        self.layout.addWidget(self.image_view)
        self.layout.addWidget(self.slider)
        self.setCentralWidget(self.central_widget)

        self.button_loadfolder.clicked.connect(self.load_folder)
        self.slider.valueChanged.connect(self.update_tick)

    def load_folder(self):
        pass

    def update_tick(self, value):
        pass


if __name__ == '__main__':
    app = QApplication([])
    window = StartWindow()
    window.show()
    app.exit(app.exec_())