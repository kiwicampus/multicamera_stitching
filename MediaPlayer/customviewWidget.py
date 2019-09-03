import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QIcon, QPixmap, QPainter, QPen
from PyQt5.QtCore import Qt

class ImageVisor(QWidget):
    '''
    This class defines a custom ImageVisor with QPixMap and image setting
    features. 
    This widget can be used in two ways, as a visualizer and as a hybrid between visualizer and
    pixel map sensor to identify interactions of the mouse pointer with the image. The former option is
    the default. To use the pixel map sensor set pixmap_enabled to true in the initializer
    '''
    def __init__(self, image_path = None, pixmap_enabled = False, parent=None):
        QWidget.__init__(self, parent=parent)
        self.pixmap_enabled = pixmap_enabled

        if image_path!= None:
            self.setImage(image_path) # If image is defined set the image in the element

        self.pointer_position = None # mouse pointer position when hovering over image
        self.click_position = None # click action position when click over image
        self.mark_tracker = [] # storage of x and y pixel position

        self.setMouseTracking(self.pixmap_enabled) # Enables mouse tracking. Enabled when pixel mapping -pixmap- is needed. Disabled otherwise

    def paintEvent(self, event):
        '''
        This function paints a pixmap over the window. We can add marks and other 
        geometrical shapes when needed
        '''
        painter = QPainter(self)
        painter.drawPixmap(self.rect(), self.pixmap)
        # If pixel mapping is enabled read mouse and click positions and create markers
        if self.pixmap_enabled:
            # If there is pointer_position draw blue mark
            if self.pointer_position:
                painter.setPen(QPen(Qt.blue, 5, Qt.SolidLine))
                painter.drawEllipse(self.pointer_position, 5, 5)
            # If click position has been set show red markers (the ones defined with click over the image)
            if self.click_position:
                for pos in self.mark_tracker:
                    painter.setPen(QPen(Qt.red, 10, Qt.SolidLine))
                    painter.drawEllipse(pos, 5, 5)

    def mouseMoveEvent(self, event):
        '''
        Callback for mouse move events
        '''
        self.pointer_position = event.pos() # Read mouse pointer position
        print('Mouse position: ', self.pointer_position)
        self.update() # Update painter

    def mousePressEvent(self, event):
        '''
        Callback for mouse press (clicks) events
        '''
        self.click_position = event.pos() # Read mouse pointer position when clicking
        self.mark_tracker.append(self.click_position) # Append position to the mark tracker for metrics
        self.update() # Update painter

    def setImage(self, path): 
        self.pixmap = QPixmap(path) # Update pixmap property with QPixmap object based on the image at path
        self.update # Update painter
        
class ImageWindow(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setLayout(QVBoxLayout()) # Define vertical box layout
        self.view = ImageVisor() # Instatiate object ImageVisor
        self.layout().addWidget(self.view) # Add ImageVisor object to the layout
        self.resize(640,360) # Set window size

if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = ImageWindow()
    w.show()
    sys.exit(app.exec_())   