import configparser
import os

from base_gui_fwin import Ui_Form
from base_gui_window import Ui_MainWindow

from DroneControl import DroneControl

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QMutex, QWaitCondition
from PyQt5.QtGui import QImage, QPixmap


class Gui(Ui_Form):
    def __init__(self):
        super().__init__()

        self.Form = QtWidgets.QWidget()
        self.setupUi(self)
        self.initUi()

        self.show()
        # self.Form.show()
        self.drone = DroneControl()

        self.mutex_label_pixmap = QMutex()
        self.condition_pixmap = QWaitCondition()

        self.connectWidget()

        self.parseConfig()

    def initUi(self):
        # self.highH_spinBox.setValue(255)
        # self.highS_spinBox.setValue(255)
        # self.highV_spinBox.setValue(255)

        self.imageLabel.setScaledContents(True)

    def connectWidget(self):
        self.drone.changePixmap1.connect(self.setImage1)
        self.drone.changeFpsCounter.connect(lambda val: self.fps_label.setText(str(val)))

        self.pX_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('p','x',val))
        self.iX_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('i','x',val))
        self.dX_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('d','x',val))
        self.mpX_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('m','x',val))
        self.pY_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('p', 'y', val))
        self.iY_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('i', 'y', val))
        self.dY_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('d', 'y', val))
        self.mpY_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('m', 'y', val))
        self.pZ_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('p', 'z', val))
        self.iZ_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('i', 'z', val))
        self.dZ_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('d', 'z', val))
        self.mpZ_doubleSpinBox.valueChanged.connect(lambda val: self.drone.onPIDChanged('m', 'z', val))

        self.lowH_spinBox.valueChanged.connect(lambda val: self.drone.onHSVChanged('h', 'low', val))
        self.lowS_spinBox.valueChanged.connect(lambda val: self.drone.onHSVChanged('s', 'low', val))
        self.lowV_spinBox.valueChanged.connect(lambda val: self.drone.onHSVChanged('v', 'low', val))
        self.highH_spinBox.valueChanged.connect(lambda val: self.drone.onHSVChanged('h', 'high', val))
        self.highS_spinBox.valueChanged.connect(lambda val: self.drone.onHSVChanged('s', 'high', val))
        self.highV_spinBox.valueChanged.connect(lambda val: self.drone.onHSVChanged('v', 'high', val))

        self.lowH_spinBox_2.valueChanged.connect(lambda val: self.drone.onHSVChanged('h', 'low2', val))
        self.lowS_spinBox_2.valueChanged.connect(lambda val: self.drone.onHSVChanged('s', 'low2', val))
        self.lowV_spinBox_2.valueChanged.connect(lambda val: self.drone.onHSVChanged('v', 'low2', val))
        self.highH_spinBox_2.valueChanged.connect(lambda val: self.drone.onHSVChanged('h', 'high2', val))
        self.highS_spinBox_2.valueChanged.connect(lambda val: self.drone.onHSVChanged('s', 'high2', val))
        self.highV_spinBox_2.valueChanged.connect(lambda val: self.drone.onHSVChanged('v', 'high2', val))

        self.erode_size_spinBox.valueChanged.connect(lambda  val: self.drone.onMorphChange('erode', 'erode1', val))
        self.dilate_size_spinBox.valueChanged.connect(lambda  val: self.drone.onMorphChange('dilate', 'dilate1', val))
        self.erode_size_spinBox_2.valueChanged.connect(lambda val: self.drone.onMorphChange('erode', 'erode2',val))
        self.dilate_size_spinBox_2.valueChanged.connect(lambda val: self.drone.onMorphChange('dilate', 'dilate2', val))

        self.offset_x_spinBox.valueChanged.connect(lambda :self.drone.onLandingAreaChanged(self.offset_x_spinBox.value(), \
                                                    self.offset_y_spinBox.value(), self.offset_z_spinBox.value()))
        self.offset_y_spinBox.valueChanged.connect(
            lambda: self.drone.onLandingAreaChanged(self.offset_x_spinBox.value(), \
                                                    self.offset_y_spinBox.value(), self.offset_z_spinBox.value()))
        self.offset_z_spinBox.valueChanged.connect(
            lambda: self.drone.onLandingAreaChanged(self.offset_x_spinBox.value(), \
                                                    self.offset_y_spinBox.value(), self.offset_z_spinBox.value()))

        self.save_pushButton.clicked.connect(self.saveConfig)

    def setImage1(self, image):
        self.mutex_label_pixmap.lock()
        try:
            self.imageLabel.setPixmap(QPixmap.fromImage(image))
        finally:
            self.mutex_label_pixmap.unlock()
            self.condition_pixmap.wakeAll()

    def parseConfig(self):
        if not os.path.exists('drone_param.ini'):
            self.saveConfig()
        else:
            config = configparser.ConfigParser()
            config.read('drone_param.ini')
            try:
                self.lowH_spinBox.setValue(int(config['hsv1']['low_h']))
                self.highH_spinBox.setValue(int(config['hsv1']['high_h']))
                self.lowS_spinBox.setValue(int(config['hsv1']['low_s']))
                self.highS_spinBox.setValue(int(config['hsv1']['high_s']))
                self.lowV_spinBox.setValue(int(config['hsv1']['low_v']))
                self.highV_spinBox.setValue(int(config['hsv1']['high_v']))
                self.erode_size_spinBox.setValue(int(config['hsv1']['erode']))
                self.dilate_size_spinBox.setValue(int(config['hsv1']['dilate']))

                self.lowH_spinBox_2.setValue(int(config['hsv2']['low_h']))
                self.highH_spinBox_2.setValue(int(config['hsv2']['high_h']))
                self.lowS_spinBox_2.setValue(int(config['hsv2']['low_s']))
                self.highS_spinBox_2.setValue(int(config['hsv2']['high_s']))
                self.lowV_spinBox_2.setValue(int(config['hsv2']['low_v']))
                self.highV_spinBox_2.setValue(int(config['hsv2']['high_v']))
                self.erode_size_spinBox_2.setValue(int(config['hsv2']['erode']))
                self.dilate_size_spinBox_2.setValue(int(config['hsv2']['dilate']))

                self.offset_x_spinBox.setValue(int(config['offset']['x']))
                self.offset_y_spinBox.setValue(int(config['offset']['y']))
                self.offset_z_spinBox.setValue(int(config['offset']['z']))

                self.pX_doubleSpinBox.setValue(float(config['pidmx']['p']))
                self.iX_doubleSpinBox.setValue(float(config['pidmx']['i']))
                self.dX_doubleSpinBox.setValue(float(config['pidmx']['d']))
                self.mpX_doubleSpinBox.setValue(float(config['pidmx']['mp']))

                self.pY_doubleSpinBox.setValue(float(config['pidmy']['p']))
                self.iY_doubleSpinBox.setValue(float(config['pidmy']['i']))
                self.dY_doubleSpinBox.setValue(float(config['pidmy']['d']))
                self.mpY_doubleSpinBox.setValue(float(config['pidmy']['mp']))

                self.pZ_doubleSpinBox.setValue(float(config['pidmz']['p']))
                self.iZ_doubleSpinBox.setValue(float(config['pidmz']['i']))
                self.dZ_doubleSpinBox.setValue(float(config['pidmz']['d']))
                self.mpZ_doubleSpinBox.setValue(float(config['pidmz']['mp']))

            except Exception:
                print("error")


    def saveConfig(self):
        config = configparser.ConfigParser()
        config['hsv1'] = {
            'low_h': self.lowH_spinBox.value(),
            'high_h': self.highH_spinBox.value(),
            'low_s': self.lowS_spinBox.value(),
            'high_s': self.highS_spinBox.value(),
            'low_v': self.lowV_spinBox.value(),
            'high_v': self.highV_spinBox.value(),
            'erode': self.erode_size_spinBox.value(),
            'dilate': self.dilate_size_spinBox.value(),

        }
        config['hsv2'] = {
            'low_h': self.lowH_spinBox_2.value(),
            'high_h': self.highH_spinBox_2.value(),
            'low_s': self.lowS_spinBox_2.value(),
            'high_s': self.highS_spinBox_2.value(),
            'low_v': self.lowV_spinBox_2.value(),
            'high_v': self.highV_spinBox_2.value(),
            'erode': self.erode_size_spinBox_2.value(),
            'dilate': self.dilate_size_spinBox_2.value(),
        }

        config['offset'] = {
            'x': self.offset_x_spinBox.value(),
            'y': self.offset_y_spinBox.value(),
            'z': self.offset_z_spinBox.value(),

        }

        config['pidmx'] = {
            'p': self.pX_doubleSpinBox.value(),
            'i': self.iX_doubleSpinBox.value(),
            'd': self.dX_doubleSpinBox.value(),
            'mp': self.mpX_doubleSpinBox.value()
        }

        config['pidmy'] = {
            'p': self.pY_doubleSpinBox.value(),
            'i': self.iY_doubleSpinBox.value(),
            'd': self.dY_doubleSpinBox.value(),
            'mp': self.mpY_doubleSpinBox.value()
        }

        config['pidmz'] = {
            'p': self.pZ_doubleSpinBox.value(),
            'i': self.iZ_doubleSpinBox.value(),
            'd': self.dZ_doubleSpinBox.value(),
            'mp': self.mpZ_doubleSpinBox.value()
        }

        with open('drone_param.ini', 'w') as f:
            config.write(f)
        pass

    def closeEvent(self, event):
        self.drone.stopAllThread()
        print("QUIT BROOO")

# if __name__ == "__main__":
#     import sys
#     app = QtWidgets.QApplication(sys.argv)
#     ui = Gui()
#     # Form = QtWidgets.QWidget()
#     # ui = Ui_Form()
#     # ui.setupUi(Form)
#     # Form.show()
#     sys.exit(app.exec_())

if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)

    # MainWindow = QtWidgets.QMainWindow()
    ui = Gui()
    # ui.setupUi(MainWindow)
    # MainWindow.show()
    sys.exit(app.exec_())