import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QIcon, QPixmap, QPainter, QPen
from PyQt5.QtCore import Qt
import cv2

class ImageVisor(QWidget):
    def __init__(self, pixmap,parent=None):
        QWidget.__init__(self, parent=parent)
        self.index = 0
        self.image_path = None
        self.pixmap = pixmap
        self.pos = None
        self.pos_click = None
        self.setMouseTracking(True)
        self.marks = []

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(self.rect(), self.pixmap)
        if self.pos:
            painter.setPen(QPen(Qt.blue, 5, Qt.SolidLine))
            painter.drawEllipse(self.pos, 5, 5)
        if self.pos_click:
            for pos in self.marks:
                painter.setPen(QPen(Qt.red, 10, Qt.SolidLine))
                painter.drawEllipse(pos, 5, 5)

    def mouseMoveEvent(self, event):
        self.pos = event.pos()
        print('New position!: ',self.pos)
        self.update()

    def mousePressEvent(self, event):
        self.pos_click = event.pos()
        print('New click on image!', self.pos_click)
        self.marks.append(self.pos_click)

        #self.index += 1
        #if self.index == len(self.image_path):
        #    self.index = 0
        #self.setImage(self.image_path[self.index])
        #self.update()

    def setImage(self, path):
        pixmap = QPixmap(path)
        self.pixmap = pixmap
        
class ImageWindow(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setLayout(QVBoxLayout())
        img = cv2.imread('img1.jpg')
        print('Size: ', img.shape)
        
        self.label = ImageVisor(QPixmap('img1.jpg'))
        self.layout().addWidget(self.label)
        #self.resize(img.shape[1]//2,img.shape[0]//2)
        self.resize(640,360)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = ImageWindow(['img1.jpg' , 'img2.jpg' , 'img3.jpg'])
    w.show()
    sys.exit(app.exec_())   