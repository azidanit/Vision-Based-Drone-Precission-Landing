# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui\base_gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(850, 600)
        self.imageLabel = QtWidgets.QLabel(Form)
        self.imageLabel.setGeometry(QtCore.QRect(10, 10, 547, 411))
        self.imageLabel.setStyleSheet("background-color: rgb(85, 255, 255);")
        self.imageLabel.setScaledContents(True)
        self.imageLabel.setObjectName("imageLabel")
        self.formLayoutWidget = QtWidgets.QWidget(Form)
        self.formLayoutWidget.setGeometry(QtCore.QRect(570, 100, 111, 121))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.formLayout = QtWidgets.QFormLayout(self.formLayoutWidget)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setObjectName("formLayout")
        self.label = QtWidgets.QLabel(self.formLayoutWidget)
        self.label.setObjectName("label")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label)
        self.pX_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget)
        self.pX_doubleSpinBox.setMaximum(500.0)
        self.pX_doubleSpinBox.setSingleStep(0.5)
        self.pX_doubleSpinBox.setObjectName("pX_doubleSpinBox")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.pX_doubleSpinBox)
        self.label_2 = QtWidgets.QLabel(self.formLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_2)
        self.iX_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget)
        self.iX_doubleSpinBox.setMaximum(500.0)
        self.iX_doubleSpinBox.setSingleStep(0.5)
        self.iX_doubleSpinBox.setObjectName("iX_doubleSpinBox")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.iX_doubleSpinBox)
        self.label_3 = QtWidgets.QLabel(self.formLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_3)
        self.dX_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget)
        self.dX_doubleSpinBox.setMaximum(500.0)
        self.dX_doubleSpinBox.setSingleStep(0.5)
        self.dX_doubleSpinBox.setObjectName("dX_doubleSpinBox")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.dX_doubleSpinBox)
        self.label_4 = QtWidgets.QLabel(self.formLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.label_4)
        self.mpX_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget)
        self.mpX_doubleSpinBox.setMaximum(500.0)
        self.mpX_doubleSpinBox.setSingleStep(0.5)
        self.mpX_doubleSpinBox.setObjectName("mpX_doubleSpinBox")
        self.formLayout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.mpX_doubleSpinBox)
        self.formLayoutWidget_2 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_2.setGeometry(QtCore.QRect(690, 100, 111, 121))
        self.formLayoutWidget_2.setObjectName("formLayoutWidget_2")
        self.formLayout_2 = QtWidgets.QFormLayout(self.formLayoutWidget_2)
        self.formLayout_2.setContentsMargins(0, 0, 0, 0)
        self.formLayout_2.setObjectName("formLayout_2")
        self.label_5 = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_5.setObjectName("label_5")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_5)
        self.pY_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget_2)
        self.pY_doubleSpinBox.setMaximum(500.0)
        self.pY_doubleSpinBox.setSingleStep(0.5)
        self.pY_doubleSpinBox.setObjectName("pY_doubleSpinBox")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.pY_doubleSpinBox)
        self.label_6 = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_6.setObjectName("label_6")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_6)
        self.iY_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget_2)
        self.iY_doubleSpinBox.setMaximum(500.0)
        self.iY_doubleSpinBox.setSingleStep(0.5)
        self.iY_doubleSpinBox.setObjectName("iY_doubleSpinBox")
        self.formLayout_2.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.iY_doubleSpinBox)
        self.label_7 = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_7.setObjectName("label_7")
        self.formLayout_2.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_7)
        self.dY_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget_2)
        self.dY_doubleSpinBox.setMaximum(500.0)
        self.dY_doubleSpinBox.setSingleStep(0.5)
        self.dY_doubleSpinBox.setObjectName("dY_doubleSpinBox")
        self.formLayout_2.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.dY_doubleSpinBox)
        self.mpY_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget_2)
        self.mpY_doubleSpinBox.setMaximum(500.0)
        self.mpY_doubleSpinBox.setSingleStep(0.5)
        self.mpY_doubleSpinBox.setObjectName("mpY_doubleSpinBox")
        self.formLayout_2.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.mpY_doubleSpinBox)
        self.label_8 = QtWidgets.QLabel(self.formLayoutWidget_2)
        self.label_8.setObjectName("label_8")
        self.formLayout_2.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.label_8)
        self.formLayoutWidget_3 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_3.setGeometry(QtCore.QRect(570, 230, 111, 121))
        self.formLayoutWidget_3.setObjectName("formLayoutWidget_3")
        self.formLayout_3 = QtWidgets.QFormLayout(self.formLayoutWidget_3)
        self.formLayout_3.setContentsMargins(0, 0, 0, 0)
        self.formLayout_3.setObjectName("formLayout_3")
        self.label_9 = QtWidgets.QLabel(self.formLayoutWidget_3)
        self.label_9.setObjectName("label_9")
        self.formLayout_3.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_9)
        self.pZ_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget_3)
        self.pZ_doubleSpinBox.setMaximum(500.0)
        self.pZ_doubleSpinBox.setSingleStep(0.5)
        self.pZ_doubleSpinBox.setObjectName("pZ_doubleSpinBox")
        self.formLayout_3.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.pZ_doubleSpinBox)
        self.label_10 = QtWidgets.QLabel(self.formLayoutWidget_3)
        self.label_10.setObjectName("label_10")
        self.formLayout_3.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_10)
        self.iZ_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget_3)
        self.iZ_doubleSpinBox.setMaximum(500.0)
        self.iZ_doubleSpinBox.setSingleStep(0.5)
        self.iZ_doubleSpinBox.setObjectName("iZ_doubleSpinBox")
        self.formLayout_3.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.iZ_doubleSpinBox)
        self.label_11 = QtWidgets.QLabel(self.formLayoutWidget_3)
        self.label_11.setObjectName("label_11")
        self.formLayout_3.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_11)
        self.dZ_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget_3)
        self.dZ_doubleSpinBox.setMaximum(500.0)
        self.dZ_doubleSpinBox.setSingleStep(0.5)
        self.dZ_doubleSpinBox.setObjectName("dZ_doubleSpinBox")
        self.formLayout_3.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.dZ_doubleSpinBox)
        self.mpZ_doubleSpinBox = QtWidgets.QDoubleSpinBox(self.formLayoutWidget_3)
        self.mpZ_doubleSpinBox.setMaximum(500.0)
        self.mpZ_doubleSpinBox.setSingleStep(0.5)
        self.mpZ_doubleSpinBox.setObjectName("mpZ_doubleSpinBox")
        self.formLayout_3.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.mpZ_doubleSpinBox)
        self.label_12 = QtWidgets.QLabel(self.formLayoutWidget_3)
        self.label_12.setObjectName("label_12")
        self.formLayout_3.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.label_12)
        self.formLayoutWidget_4 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_4.setGeometry(QtCore.QRect(690, 230, 111, 121))
        self.formLayoutWidget_4.setObjectName("formLayoutWidget_4")
        self.formLayout_4 = QtWidgets.QFormLayout(self.formLayoutWidget_4)
        self.formLayout_4.setContentsMargins(0, 0, 0, 0)
        self.formLayout_4.setObjectName("formLayout_4")
        self.label_13 = QtWidgets.QLabel(self.formLayoutWidget_4)
        self.label_13.setObjectName("label_13")
        self.formLayout_4.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_13)
        self.label_14 = QtWidgets.QLabel(self.formLayoutWidget_4)
        self.label_14.setObjectName("label_14")
        self.formLayout_4.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_14)
        self.label_15 = QtWidgets.QLabel(self.formLayoutWidget_4)
        self.label_15.setObjectName("label_15")
        self.formLayout_4.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_15)
        self.offset_x_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_4)
        self.offset_x_spinBox.setMinimum(-999)
        self.offset_x_spinBox.setMaximum(999)
        self.offset_x_spinBox.setSingleStep(10)
        self.offset_x_spinBox.setObjectName("offset_x_spinBox")
        self.formLayout_4.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.offset_x_spinBox)
        self.offset_y_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_4)
        self.offset_y_spinBox.setMinimum(-999)
        self.offset_y_spinBox.setMaximum(999)
        self.offset_y_spinBox.setSingleStep(10)
        self.offset_y_spinBox.setObjectName("offset_y_spinBox")
        self.formLayout_4.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.offset_y_spinBox)
        self.offset_z_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_4)
        self.offset_z_spinBox.setMinimum(-999)
        self.offset_z_spinBox.setMaximum(999)
        self.offset_z_spinBox.setSingleStep(10)
        self.offset_z_spinBox.setObjectName("offset_z_spinBox")
        self.formLayout_4.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.offset_z_spinBox)
        self.formLayoutWidget_5 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_5.setGeometry(QtCore.QRect(570, 12, 160, 80))
        self.formLayoutWidget_5.setObjectName("formLayoutWidget_5")
        self.formLayout_5 = QtWidgets.QFormLayout(self.formLayoutWidget_5)
        self.formLayout_5.setContentsMargins(0, 0, 0, 0)
        self.formLayout_5.setObjectName("formLayout_5")
        self.label_16 = QtWidgets.QLabel(self.formLayoutWidget_5)
        self.label_16.setObjectName("label_16")
        self.formLayout_5.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_16)
        self.errorX_label = QtWidgets.QLabel(self.formLayoutWidget_5)
        self.errorX_label.setObjectName("errorX_label")
        self.formLayout_5.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.errorX_label)
        self.label_18 = QtWidgets.QLabel(self.formLayoutWidget_5)
        self.label_18.setObjectName("label_18")
        self.formLayout_5.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_18)
        self.errorY_label = QtWidgets.QLabel(self.formLayoutWidget_5)
        self.errorY_label.setObjectName("errorY_label")
        self.formLayout_5.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.errorY_label)
        self.label_20 = QtWidgets.QLabel(self.formLayoutWidget_5)
        self.label_20.setObjectName("label_20")
        self.formLayout_5.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_20)
        self.errorZ_label = QtWidgets.QLabel(self.formLayoutWidget_5)
        self.errorZ_label.setObjectName("errorZ_label")
        self.formLayout_5.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.errorZ_label)
        self.formLayoutWidget_6 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_6.setGeometry(QtCore.QRect(10, 430, 111, 81))
        self.formLayoutWidget_6.setObjectName("formLayoutWidget_6")
        self.formLayout_6 = QtWidgets.QFormLayout(self.formLayoutWidget_6)
        self.formLayout_6.setContentsMargins(0, 0, 0, 0)
        self.formLayout_6.setObjectName("formLayout_6")
        self.label_17 = QtWidgets.QLabel(self.formLayoutWidget_6)
        self.label_17.setObjectName("label_17")
        self.formLayout_6.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_17)
        self.label_19 = QtWidgets.QLabel(self.formLayoutWidget_6)
        self.label_19.setObjectName("label_19")
        self.formLayout_6.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_19)
        self.label_21 = QtWidgets.QLabel(self.formLayoutWidget_6)
        self.label_21.setObjectName("label_21")
        self.formLayout_6.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_21)
        self.lowH_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_6)
        self.lowH_spinBox.setMaximum(255)
        self.lowH_spinBox.setSingleStep(20)
        self.lowH_spinBox.setObjectName("lowH_spinBox")
        self.formLayout_6.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.lowH_spinBox)
        self.lowS_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_6)
        self.lowS_spinBox.setMaximum(255)
        self.lowS_spinBox.setSingleStep(20)
        self.lowS_spinBox.setObjectName("lowS_spinBox")
        self.formLayout_6.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.lowS_spinBox)
        self.lowV_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_6)
        self.lowV_spinBox.setMaximum(255)
        self.lowV_spinBox.setSingleStep(20)
        self.lowV_spinBox.setObjectName("lowV_spinBox")
        self.formLayout_6.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.lowV_spinBox)
        self.formLayoutWidget_7 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_7.setGeometry(QtCore.QRect(130, 429, 111, 81))
        self.formLayoutWidget_7.setObjectName("formLayoutWidget_7")
        self.formLayout_7 = QtWidgets.QFormLayout(self.formLayoutWidget_7)
        self.formLayout_7.setContentsMargins(0, 0, 0, 0)
        self.formLayout_7.setObjectName("formLayout_7")
        self.label_22 = QtWidgets.QLabel(self.formLayoutWidget_7)
        self.label_22.setObjectName("label_22")
        self.formLayout_7.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_22)
        self.label_23 = QtWidgets.QLabel(self.formLayoutWidget_7)
        self.label_23.setObjectName("label_23")
        self.formLayout_7.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_23)
        self.label_24 = QtWidgets.QLabel(self.formLayoutWidget_7)
        self.label_24.setObjectName("label_24")
        self.formLayout_7.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_24)
        self.highH_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_7)
        self.highH_spinBox.setMaximum(255)
        self.highH_spinBox.setSingleStep(20)
        self.highH_spinBox.setObjectName("highH_spinBox")
        self.formLayout_7.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.highH_spinBox)
        self.highS_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_7)
        self.highS_spinBox.setMaximum(255)
        self.highS_spinBox.setSingleStep(20)
        self.highS_spinBox.setObjectName("highS_spinBox")
        self.formLayout_7.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.highS_spinBox)
        self.highV_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_7)
        self.highV_spinBox.setMaximum(255)
        self.highV_spinBox.setSingleStep(20)
        self.highV_spinBox.setObjectName("highV_spinBox")
        self.formLayout_7.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.highV_spinBox)
        self.formLayoutWidget_8 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_8.setGeometry(QtCore.QRect(250, 430, 111, 51))
        self.formLayoutWidget_8.setObjectName("formLayoutWidget_8")
        self.formLayout_8 = QtWidgets.QFormLayout(self.formLayoutWidget_8)
        self.formLayout_8.setContentsMargins(0, 0, 0, 0)
        self.formLayout_8.setObjectName("formLayout_8")
        self.label_25 = QtWidgets.QLabel(self.formLayoutWidget_8)
        self.label_25.setObjectName("label_25")
        self.formLayout_8.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_25)
        self.erode_size_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_8)
        self.erode_size_spinBox.setMaximum(200)
        self.erode_size_spinBox.setObjectName("erode_size_spinBox")
        self.formLayout_8.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.erode_size_spinBox)
        self.label_26 = QtWidgets.QLabel(self.formLayoutWidget_8)
        self.label_26.setObjectName("label_26")
        self.formLayout_8.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_26)
        self.dilate_size_spinBox = QtWidgets.QSpinBox(self.formLayoutWidget_8)
        self.dilate_size_spinBox.setMaximum(200)
        self.dilate_size_spinBox.setObjectName("dilate_size_spinBox")
        self.formLayout_8.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.dilate_size_spinBox)
        self.formLayoutWidget_9 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_9.setGeometry(QtCore.QRect(10, 516, 111, 81))
        self.formLayoutWidget_9.setObjectName("formLayoutWidget_9")
        self.formLayout_9 = QtWidgets.QFormLayout(self.formLayoutWidget_9)
        self.formLayout_9.setContentsMargins(0, 0, 0, 0)
        self.formLayout_9.setObjectName("formLayout_9")
        self.label_27 = QtWidgets.QLabel(self.formLayoutWidget_9)
        self.label_27.setObjectName("label_27")
        self.formLayout_9.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_27)
        self.label_28 = QtWidgets.QLabel(self.formLayoutWidget_9)
        self.label_28.setObjectName("label_28")
        self.formLayout_9.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_28)
        self.label_29 = QtWidgets.QLabel(self.formLayoutWidget_9)
        self.label_29.setObjectName("label_29")
        self.formLayout_9.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_29)
        self.lowH_spinBox_2 = QtWidgets.QSpinBox(self.formLayoutWidget_9)
        self.lowH_spinBox_2.setMaximum(255)
        self.lowH_spinBox_2.setSingleStep(20)
        self.lowH_spinBox_2.setObjectName("lowH_spinBox_2")
        self.formLayout_9.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.lowH_spinBox_2)
        self.lowS_spinBox_2 = QtWidgets.QSpinBox(self.formLayoutWidget_9)
        self.lowS_spinBox_2.setMaximum(255)
        self.lowS_spinBox_2.setSingleStep(20)
        self.lowS_spinBox_2.setObjectName("lowS_spinBox_2")
        self.formLayout_9.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.lowS_spinBox_2)
        self.lowV_spinBox_2 = QtWidgets.QSpinBox(self.formLayoutWidget_9)
        self.lowV_spinBox_2.setMaximum(255)
        self.lowV_spinBox_2.setSingleStep(20)
        self.lowV_spinBox_2.setObjectName("lowV_spinBox_2")
        self.formLayout_9.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.lowV_spinBox_2)
        self.formLayoutWidget_10 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_10.setGeometry(QtCore.QRect(130, 515, 111, 81))
        self.formLayoutWidget_10.setObjectName("formLayoutWidget_10")
        self.formLayout_10 = QtWidgets.QFormLayout(self.formLayoutWidget_10)
        self.formLayout_10.setContentsMargins(0, 0, 0, 0)
        self.formLayout_10.setObjectName("formLayout_10")
        self.label_30 = QtWidgets.QLabel(self.formLayoutWidget_10)
        self.label_30.setObjectName("label_30")
        self.formLayout_10.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_30)
        self.label_31 = QtWidgets.QLabel(self.formLayoutWidget_10)
        self.label_31.setObjectName("label_31")
        self.formLayout_10.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_31)
        self.label_32 = QtWidgets.QLabel(self.formLayoutWidget_10)
        self.label_32.setObjectName("label_32")
        self.formLayout_10.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_32)
        self.highH_spinBox_2 = QtWidgets.QSpinBox(self.formLayoutWidget_10)
        self.highH_spinBox_2.setMaximum(255)
        self.highH_spinBox_2.setSingleStep(20)
        self.highH_spinBox_2.setObjectName("highH_spinBox_2")
        self.formLayout_10.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.highH_spinBox_2)
        self.highS_spinBox_2 = QtWidgets.QSpinBox(self.formLayoutWidget_10)
        self.highS_spinBox_2.setMaximum(255)
        self.highS_spinBox_2.setSingleStep(20)
        self.highS_spinBox_2.setObjectName("highS_spinBox_2")
        self.formLayout_10.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.highS_spinBox_2)
        self.highV_spinBox_2 = QtWidgets.QSpinBox(self.formLayoutWidget_10)
        self.highV_spinBox_2.setMaximum(255)
        self.highV_spinBox_2.setSingleStep(20)
        self.highV_spinBox_2.setObjectName("highV_spinBox_2")
        self.formLayout_10.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.highV_spinBox_2)
        self.formLayoutWidget_11 = QtWidgets.QWidget(Form)
        self.formLayoutWidget_11.setGeometry(QtCore.QRect(250, 520, 111, 51))
        self.formLayoutWidget_11.setObjectName("formLayoutWidget_11")
        self.formLayout_11 = QtWidgets.QFormLayout(self.formLayoutWidget_11)
        self.formLayout_11.setContentsMargins(0, 0, 0, 0)
        self.formLayout_11.setObjectName("formLayout_11")
        self.label_33 = QtWidgets.QLabel(self.formLayoutWidget_11)
        self.label_33.setObjectName("label_33")
        self.formLayout_11.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_33)
        self.erode_size_spinBox_2 = QtWidgets.QSpinBox(self.formLayoutWidget_11)
        self.erode_size_spinBox_2.setMaximum(200)
        self.erode_size_spinBox_2.setObjectName("erode_size_spinBox_2")
        self.formLayout_11.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.erode_size_spinBox_2)
        self.label_34 = QtWidgets.QLabel(self.formLayoutWidget_11)
        self.label_34.setObjectName("label_34")
        self.formLayout_11.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_34)
        self.dilate_size_spinBox_2 = QtWidgets.QSpinBox(self.formLayoutWidget_11)
        self.dilate_size_spinBox_2.setMaximum(200)
        self.dilate_size_spinBox_2.setObjectName("dilate_size_spinBox_2")
        self.formLayout_11.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.dilate_size_spinBox_2)
        self.save_pushButton = QtWidgets.QPushButton(Form)
        self.save_pushButton.setGeometry(QtCore.QRect(770, 570, 75, 23))
        self.save_pushButton.setObjectName("save_pushButton")
        self.fps_label = QtWidgets.QLabel(Form)
        self.fps_label.setGeometry(QtCore.QRect(15, 15, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(13)
        self.fps_label.setFont(font)
        self.fps_label.setStyleSheet("color: rgb(255, 0, 0);")
        self.fps_label.setObjectName("fps_label")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.imageLabel.setText(_translate("Form", "TextLabel"))
        self.label.setText(_translate("Form", "Px"))
        self.label_2.setText(_translate("Form", "Ix"))
        self.label_3.setText(_translate("Form", "Dx"))
        self.label_4.setText(_translate("Form", "Mp x"))
        self.label_5.setText(_translate("Form", "Py"))
        self.label_6.setText(_translate("Form", "Iy"))
        self.label_7.setText(_translate("Form", "Dy"))
        self.label_8.setText(_translate("Form", "Mp y"))
        self.label_9.setText(_translate("Form", "Pz"))
        self.label_10.setText(_translate("Form", "Iz"))
        self.label_11.setText(_translate("Form", "Dz"))
        self.label_12.setText(_translate("Form", "Mp z"))
        self.label_13.setText(_translate("Form", "Offset X"))
        self.label_14.setText(_translate("Form", "Offset Y"))
        self.label_15.setText(_translate("Form", "Offset Z"))
        self.label_16.setText(_translate("Form", "Error X"))
        self.errorX_label.setText(_translate("Form", "TextLabel"))
        self.label_18.setText(_translate("Form", "Error Y"))
        self.errorY_label.setText(_translate("Form", "TextLabel"))
        self.label_20.setText(_translate("Form", "Error Z"))
        self.errorZ_label.setText(_translate("Form", "TextLabel"))
        self.label_17.setText(_translate("Form", "Low H"))
        self.label_19.setText(_translate("Form", "Low S"))
        self.label_21.setText(_translate("Form", "Low V"))
        self.label_22.setText(_translate("Form", "High H"))
        self.label_23.setText(_translate("Form", "High S"))
        self.label_24.setText(_translate("Form", "High V"))
        self.label_25.setText(_translate("Form", "Erode"))
        self.label_26.setText(_translate("Form", "Dilate"))
        self.label_27.setText(_translate("Form", "Low H"))
        self.label_28.setText(_translate("Form", "Low S"))
        self.label_29.setText(_translate("Form", "Low V"))
        self.label_30.setText(_translate("Form", "High H"))
        self.label_31.setText(_translate("Form", "High S"))
        self.label_32.setText(_translate("Form", "High V"))
        self.label_33.setText(_translate("Form", "Erode"))
        self.label_34.setText(_translate("Form", "Dilate"))
        self.save_pushButton.setText(_translate("Form", "save"))
        self.fps_label.setText(_translate("Form", "TextLabel"))
