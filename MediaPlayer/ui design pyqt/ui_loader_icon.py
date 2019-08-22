import sys
from PyQt5 import QtCore, QtGui, QtWidgets, uic
from decimal import Decimal

qtcreator_file  = "ui/icon_test.ui" # Enter file here.
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtcreator_file)


class MyWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)

        self.image_view.ui.histogram.hide() # Hides histogram from image frame
        self.image_view.ui.roiBtn.hide() # Hides roi button from image frame
        self.image_view.ui.menuBtn.hide() # Hides menu button from image frame

        rMyIcon = QtGui.QPixmap("usb.png");
        self.button_loadfile.setIcon(QtGui.QIcon(rMyIcon))
        self.button_loadfile.setIconSize(QtCore.QSize(48,48))

        prevMyIcon = QtGui.QPixmap("previous-1.png");
        self.button_previous_capture.setIcon(QtGui.QIcon(prevMyIcon))
        self.button_previous_capture.setIconSize(QtCore.QSize(48,48))

        self.button_previous_camera.setIcon(QtGui.QIcon(prevMyIcon))
        self.button_previous_camera.setIconSize(QtCore.QSize(48,48))

        nextMyIcon = QtGui.QPixmap("skip-1.png");
        self.button_next_capture.setIcon(QtGui.QIcon(nextMyIcon))
        self.button_next_capture.setIconSize(QtCore.QSize(48,48))

        self.button_next_camera.setIcon(QtGui.QIcon(nextMyIcon))
        self.button_next_camera.setIconSize(QtCore.QSize(48,48))
        
        
        playMyIcon = QtGui.QPixmap("play.png");
        pauseMyIcon = QtGui.QPixmap("pause.png");
        self.play_pause_icons = [QtGui.QPixmap("pause.png") , QtGui.QPixmap("play.png")]
        self.button_play.setIcon(QtGui.QIcon(playMyIcon))
        self.button_play.setIconSize(QtCore.QSize(48,48))

        self.button_play.clicked.connect(self.toggle_icons)
        self.play = True

    def toggle_icons(self):
        self.play = not(self.play)
        self.button_play.setIcon(QtGui.QIcon(self.play_pause_icons[int(self.play)]))
        self.button_play.setIconSize(QtCore.QSize(48,48))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
