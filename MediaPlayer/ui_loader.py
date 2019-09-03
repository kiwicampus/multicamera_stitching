import sys
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from decimal import Decimal

qtcreator_file  = "ui/icon_test_v2.ui" # Enter file here.
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtcreator_file)


class MyWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.i = 35

        rMyIcon = QtGui.QPixmap("icons/usb.png");
        self.button_loadfile.setIcon(QtGui.QIcon(rMyIcon))
        self.button_loadfile.setIconSize(QtCore.QSize(self.i,self.i))

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
        
        
        playMyIcon = QtGui.QPixmap("icons/play.png");
        pauseMyIcon = QtGui.QPixmap("icons/pause.png");
        self.play_pause_icons = [QtGui.QPixmap("icons/pause.png") , QtGui.QPixmap("icons/play.png")]
        self.button_play.setIcon(QtGui.QIcon(playMyIcon))
        self.button_play.setIconSize(QtCore.QSize(self.i,self.i))

        self.button_play.clicked.connect(self.toggle_icons)
        self.play = True

        self.stitcher_view.view.setImage('sharingan.jpg')
        self.stitcher_view.view.pixmap_enabled = True
        self.stitcher_view.view.setMouseTracking(True)

        self.image_view.view.setImage('sharingan.jpg')

    def toggle_icons(self):
        self.play = not(self.play)
        self.button_play.setIcon(QtGui.QIcon(self.play_pause_icons[int(self.play)]))
        self.button_play.setIconSize(QtCore.QSize(self.i,self.i))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
