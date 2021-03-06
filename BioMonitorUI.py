# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'BioMonitorUI.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(763, 537)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setObjectName("gridLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.label_14 = QtWidgets.QLabel(Form)
        font = QtGui.QFont()
        font.setFamily("SimSun-ExtB")
        font.setPointSize(11)
        self.label_14.setFont(font)
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.verticalLayout_7.addWidget(self.label_14)
        self.verticalLayout_2.addLayout(self.verticalLayout_7)

        self.temp_graphicsView = pg.GraphicsLayoutWidget(Form)
        self.verticalLayout_2.addWidget(self.temp_graphicsView)

        self.ecg_graphicsView = pg.GraphicsLayoutWidget(Form)
        self.verticalLayout_2.addWidget(self.ecg_graphicsView)
        # self.ecg_graphicsView = QtWidgets.QGraphicsView(Form)
        # self.ecg_graphicsView.setObjectName("ecg_graphicsView")
        # self.verticalLayout_2.addWidget(self.ecg_graphicsView)

        self.pulse_graphicsView = pg.GraphicsLayoutWidget(Form)
        self.verticalLayout_2.addWidget(self.pulse_graphicsView)

        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.line_2 = QtWidgets.QFrame(Form)
        self.line_2.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.horizontalLayout.addWidget(self.line_2)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_12 = QtWidgets.QLabel(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_12.sizePolicy().hasHeightForWidth())
        self.label_12.setSizePolicy(sizePolicy)
        self.label_12.setMaximumSize(QtCore.QSize(150, 20))
        font = QtGui.QFont()
        font.setFamily("SimSun-ExtB")
        font.setPointSize(11)
        self.label_12.setFont(font)
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.verticalLayout_5.addWidget(self.label_12)
        self.verticalLayout.addLayout(self.verticalLayout_5)
        self.horizontalLayout_18 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_18.setObjectName("horizontalLayout_18")
        self.horizontalLayout_22 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_22.setObjectName("horizontalLayout_22")
        self.label_6 = QtWidgets.QLabel(Form)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_22.addWidget(self.label_6)
        self.temp_lineEdit = QtWidgets.QLineEdit(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.temp_lineEdit.sizePolicy().hasHeightForWidth())
        self.temp_lineEdit.setSizePolicy(sizePolicy)
        self.temp_lineEdit.setMaximumSize(QtCore.QSize(70, 16777215))
        self.temp_lineEdit.setObjectName("temp_lineEdit")
        self.horizontalLayout_22.addWidget(self.temp_lineEdit)
        self.horizontalLayout_18.addLayout(self.horizontalLayout_22)
        self.verticalLayout.addLayout(self.horizontalLayout_18)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.horizontalLayout_16 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_16.setObjectName("horizontalLayout_16")
        self.label_8 = QtWidgets.QLabel(Form)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_16.addWidget(self.label_8)
        self.heartRate_lineEdit = QtWidgets.QLineEdit(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.heartRate_lineEdit.sizePolicy().hasHeightForWidth())
        self.heartRate_lineEdit.setSizePolicy(sizePolicy)
        self.heartRate_lineEdit.setMaximumSize(QtCore.QSize(70, 16777215))
        self.heartRate_lineEdit.setObjectName("heartRate_lineEdit")
        self.horizontalLayout_16.addWidget(self.heartRate_lineEdit)
        self.horizontalLayout_9.addLayout(self.horizontalLayout_16)
        self.verticalLayout.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.horizontalLayout_23 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_23.setObjectName("horizontalLayout_23")
        self.label_9 = QtWidgets.QLabel(Form)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_23.addWidget(self.label_9)
        self.spo2_lineEdit = QtWidgets.QLineEdit(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.spo2_lineEdit.sizePolicy().hasHeightForWidth())
        self.spo2_lineEdit.setSizePolicy(sizePolicy)
        self.spo2_lineEdit.setMaximumSize(QtCore.QSize(70, 16777215))
        self.spo2_lineEdit.setObjectName("spo2_lineEdit")
        self.horizontalLayout_23.addWidget(self.spo2_lineEdit)
        self.horizontalLayout_10.addLayout(self.horizontalLayout_23)
        self.verticalLayout.addLayout(self.horizontalLayout_10)
        self.horizontalLayout_26 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_26.setObjectName("horizontalLayout_26")
        self.horizontalLayout_27 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_27.setObjectName("horizontalLayout_27")
        self.label_11 = QtWidgets.QLabel(Form)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_27.addWidget(self.label_11)
        self.BP_lineEdit = QtWidgets.QLineEdit(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.BP_lineEdit.sizePolicy().hasHeightForWidth())
        self.BP_lineEdit.setSizePolicy(sizePolicy)
        self.BP_lineEdit.setMaximumSize(QtCore.QSize(70, 16777215))
        self.BP_lineEdit.setObjectName("BP_lineEdit")
        self.horizontalLayout_27.addWidget(self.BP_lineEdit)
        self.horizontalLayout_26.addLayout(self.horizontalLayout_27)
        self.verticalLayout.addLayout(self.horizontalLayout_26)
        self.line = QtWidgets.QFrame(Form)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout.addWidget(self.line)
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.label_13 = QtWidgets.QLabel(Form)
        self.label_13.setMaximumSize(QtCore.QSize(150, 20))
        font = QtGui.QFont()
        font.setFamily("SimSun-ExtB")
        font.setPointSize(11)
        self.label_13.setFont(font)
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.verticalLayout_6.addWidget(self.label_13)
        self.verticalLayout.addLayout(self.verticalLayout_6)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label = QtWidgets.QLabel(Form)
        self.label.setObjectName("label")
        self.horizontalLayout_8.addWidget(self.label)
        self.portDetection_pushButton = QtWidgets.QPushButton(Form)
        self.portDetection_pushButton.setObjectName("portDetection_pushButton")
        self.horizontalLayout_8.addWidget(self.portDetection_pushButton)
        self.verticalLayout.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_11.addWidget(self.label_2)
        self.portSelect_comboBox = QtWidgets.QComboBox(Form)
        self.portSelect_comboBox.setObjectName("portSelect_comboBox")
        self.horizontalLayout_11.addWidget(self.portSelect_comboBox)
        self.horizontalLayout_7.addLayout(self.horizontalLayout_11)
        self.verticalLayout.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_3 = QtWidgets.QLabel(Form)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_12.addWidget(self.label_3)
        self.baudRate_comboBox = QtWidgets.QComboBox(Form)
        self.baudRate_comboBox.setObjectName("baudRate_comboBox")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.baudRate_comboBox.addItem("")
        self.horizontalLayout_12.addWidget(self.baudRate_comboBox)
        self.horizontalLayout_6.addLayout(self.horizontalLayout_12)
        self.verticalLayout.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.label_4 = QtWidgets.QLabel(Form)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_13.addWidget(self.label_4)
        self.dataBit_comboBox = QtWidgets.QComboBox(Form)
        self.dataBit_comboBox.setObjectName("dataBit_comboBox")
        self.dataBit_comboBox.addItem("")
        self.dataBit_comboBox.addItem("")
        self.dataBit_comboBox.addItem("")
        self.dataBit_comboBox.addItem("")
        self.horizontalLayout_13.addWidget(self.dataBit_comboBox)
        self.horizontalLayout_5.addLayout(self.horizontalLayout_13)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.label_5 = QtWidgets.QLabel(Form)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_14.addWidget(self.label_5)
        self.check_comboBox = QtWidgets.QComboBox(Form)
        self.check_comboBox.setObjectName("check_comboBox")
        self.check_comboBox.addItem("")
        self.check_comboBox.addItem("")
        self.check_comboBox.addItem("")
        self.horizontalLayout_14.addWidget(self.check_comboBox)
        self.horizontalLayout_2.addLayout(self.horizontalLayout_14)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.label_7 = QtWidgets.QLabel(Form)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_15.addWidget(self.label_7)
        self.stopBit_comboBox = QtWidgets.QComboBox(Form)
        self.stopBit_comboBox.setObjectName("stopBit_comboBox")
        self.stopBit_comboBox.addItem("")
        self.stopBit_comboBox.addItem("")
        self.horizontalLayout_15.addWidget(self.stopBit_comboBox)
        self.horizontalLayout_3.addLayout(self.horizontalLayout_15)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_25 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_25.setObjectName("horizontalLayout_25")
        self.label_10 = QtWidgets.QLabel(Form)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_25.addWidget(self.label_10)
        self.portStatus_lineEdit = QtWidgets.QLineEdit(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.portStatus_lineEdit.sizePolicy().hasHeightForWidth())
        self.portStatus_lineEdit.setSizePolicy(sizePolicy)
        self.portStatus_lineEdit.setMaximumSize(QtCore.QSize(70, 16777215))
        self.portStatus_lineEdit.setObjectName("portStatus_lineEdit")
        self.horizontalLayout_25.addWidget(self.portStatus_lineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout_25)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.openPort_pushButton = QtWidgets.QPushButton(Form)
        self.openPort_pushButton.setObjectName("openPort_pushButton")
        self.verticalLayout_4.addWidget(self.openPort_pushButton)
        self.closePort_pushButton = QtWidgets.QPushButton(Form)
        self.closePort_pushButton.setEnabled(False)
        self.closePort_pushButton.setObjectName("closePort_pushButton")
        self.verticalLayout_4.addWidget(self.closePort_pushButton)
        self.verticalLayout.addLayout(self.verticalLayout_4)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.gridLayout.addLayout(self.horizontalLayout, 0, 0, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label_14.setText(_translate("Form", "????????????"))
        self.label_12.setText(_translate("Form", "????????????"))
        self.label_6.setText(_translate("Form", "?????????/??????"))
        self.label_8.setText(_translate("Form", "?????????/bpm???"))
        self.label_9.setText(_translate("Form", "??????????????????"))
        self.label_11.setText(_translate("Form", "??????/mmHg???"))
        self.label_13.setText(_translate("Form", "????????????"))
        self.label.setText(_translate("Form", "???????????????"))
        self.portDetection_pushButton.setText(_translate("Form", "??????"))
        self.label_2.setText(_translate("Form", "???????????????"))
        self.label_3.setText(_translate("Form", "????????????"))
        self.baudRate_comboBox.setItemText(0, _translate("Form", "115200"))
        self.baudRate_comboBox.setItemText(1, _translate("Form", "1200"))
        self.baudRate_comboBox.setItemText(2, _translate("Form", "2400"))
        self.baudRate_comboBox.setItemText(3, _translate("Form", "4800"))
        self.baudRate_comboBox.setItemText(4, _translate("Form", "9600"))
        self.baudRate_comboBox.setItemText(5, _translate("Form", "14400"))
        self.baudRate_comboBox.setItemText(6, _translate("Form", "19200"))
        self.baudRate_comboBox.setItemText(7, _translate("Form", "38400"))
        self.baudRate_comboBox.setItemText(8, _translate("Form", "56000"))
        self.baudRate_comboBox.setItemText(9, _translate("Form", "56700"))
        self.baudRate_comboBox.setItemText(10, _translate("Form", "128000"))
        self.baudRate_comboBox.setItemText(11, _translate("Form", "256000"))
        self.label_4.setText(_translate("Form", "????????????"))
        self.dataBit_comboBox.setItemText(0, _translate("Form", "8"))
        self.dataBit_comboBox.setItemText(1, _translate("Form", "5"))
        self.dataBit_comboBox.setItemText(2, _translate("Form", "6"))
        self.dataBit_comboBox.setItemText(3, _translate("Form", "7"))
        self.label_5.setText(_translate("Form", "????????????"))
        self.check_comboBox.setItemText(0, _translate("Form", "N"))
        self.check_comboBox.setItemText(1, _translate("Form", "?????????"))
        self.check_comboBox.setItemText(2, _translate("Form", "?????????"))
        self.label_7.setText(_translate("Form", "????????????"))
        self.stopBit_comboBox.setItemText(0, _translate("Form", "1"))
        self.stopBit_comboBox.setItemText(1, _translate("Form", "2"))
        self.label_10.setText(_translate("Form", "????????????:"))
        self.openPort_pushButton.setText(_translate("Form", "????????????"))
        self.closePort_pushButton.setText(_translate("Form", "????????????"))
