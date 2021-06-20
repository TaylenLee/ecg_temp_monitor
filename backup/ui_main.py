# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'bioMonitor.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import numpy as np

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1280, 535)
        font = QtGui.QFont()
        font.setFamily("宋体")
        Form.setFont(font)
        self.groupBox = QtWidgets.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(30, 410, 171, 60))
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.groupBox.setFont(font)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.groupBox)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_7 = QtWidgets.QLabel(self.groupBox)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_7.addWidget(self.label_7)
        self.lineEdit = QtWidgets.QLineEdit(self.groupBox)
        self.lineEdit.setObjectName("lineEdit")
        self.horizontalLayout_7.addWidget(self.lineEdit)
        self.verticalLayout_4.addLayout(self.horizontalLayout_7)
        self.widget = QtWidgets.QWidget(Form)
        self.widget.setGeometry(QtCore.QRect(30, 42, 173, 341))
        self.widget.setObjectName("widget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.widget)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.pushButton = QtWidgets.QPushButton(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout.addWidget(self.pushButton)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.comboBox = QtWidgets.QComboBox(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.comboBox.setFont(font)
        self.comboBox.setObjectName("comboBox")
        self.horizontalLayout_2.addWidget(self.comboBox)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_3 = QtWidgets.QLabel(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_3.addWidget(self.label_3)
        self.comboBox_2 = QtWidgets.QComboBox(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.comboBox_2.setFont(font)
        self.comboBox_2.setObjectName("comboBox_2")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.comboBox_2.addItem("")
        self.horizontalLayout_3.addWidget(self.comboBox_2)
        self.verticalLayout_3.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_4 = QtWidgets.QLabel(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_4.addWidget(self.label_4)
        self.comboBox_3 = QtWidgets.QComboBox(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.comboBox_3.setFont(font)
        self.comboBox_3.setObjectName("comboBox_3")
        self.comboBox_3.addItem("")
        self.comboBox_3.addItem("")
        self.comboBox_3.addItem("")
        self.comboBox_3.addItem("")
        self.horizontalLayout_4.addWidget(self.comboBox_3)
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_5 = QtWidgets.QLabel(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_5.addWidget(self.label_5)
        self.comboBox_4 = QtWidgets.QComboBox(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.comboBox_4.setFont(font)
        self.comboBox_4.setObjectName("comboBox_4")
        self.comboBox_4.addItem("")
        self.horizontalLayout_5.addWidget(self.comboBox_4)
        self.verticalLayout_3.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_6 = QtWidgets.QLabel(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_6.addWidget(self.label_6)
        self.comboBox_5 = QtWidgets.QComboBox(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.comboBox_5.setFont(font)
        self.comboBox_5.setObjectName("comboBox_5")
        self.comboBox_5.addItem("")
        self.horizontalLayout_6.addWidget(self.comboBox_5)
        self.verticalLayout_3.addLayout(self.horizontalLayout_6)
        self.pushButton_2 = QtWidgets.QPushButton(self.widget)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout_3.addWidget(self.pushButton_2)
        self.pushButton_3 = QtWidgets.QPushButton(self.widget)
        self.pushButton_3.setEnabled(False)
        font = QtGui.QFont()
        font.setFamily("宋体")
        font.setPointSize(12)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setObjectName("pushButton_3")
        self.verticalLayout_3.addWidget(self.pushButton_3)
        self.widget1 = QtWidgets.QWidget(Form)
        self.widget1.setGeometry(QtCore.QRect(220, 40, 1041, 451))
        self.widget1.setObjectName("widget1")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.widget1)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.graphicsView = QtWidgets.QGraphicsView(self.widget1)
        self.graphicsView.setObjectName("graphicsView")
        self.horizontalLayout_8.addWidget(self.graphicsView)




        self.graphicsView_2 = QtWidgets.QGraphicsView(self.widget1)
        self.graphicsView_2.setObjectName("graphicsView_2")
        self.horizontalLayout_8.addWidget(self.graphicsView_2)
        self.verticalLayout_5.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.graphicsView_3 = QtWidgets.QGraphicsView(self.widget1)
        self.graphicsView_3.setObjectName("graphicsView_3")
        self.horizontalLayout_9.addWidget(self.graphicsView_3)
        self.graphicsView_4 = QtWidgets.QGraphicsView(self.widget1)
        self.graphicsView_4.setObjectName("graphicsView_4")
        self.horizontalLayout_9.addWidget(self.graphicsView_4)
        self.verticalLayout_5.addLayout(self.horizontalLayout_9)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "bioMonitor"))
        self.groupBox.setTitle(_translate("Form", "串口状态：关闭"))
        self.label_7.setText(_translate("Form", "已接收："))
        self.lineEdit.setPlaceholderText(_translate("Form", "0"))
        self.label.setText(_translate("Form", "串口检测："))
        self.pushButton.setText(_translate("Form", "检测串口"))
        self.label_2.setText(_translate("Form", "串口选择："))
        self.label_3.setText(_translate("Form", "波特率："))
        self.comboBox_2.setItemText(0, _translate("Form", "115200"))
        self.comboBox_2.setItemText(1, _translate("Form", "2400"))
        self.comboBox_2.setItemText(2, _translate("Form", "4800"))
        self.comboBox_2.setItemText(3, _translate("Form", "9600"))
        self.comboBox_2.setItemText(4, _translate("Form", "12800"))
        self.comboBox_2.setItemText(5, _translate("Form", "14400"))
        self.comboBox_2.setItemText(6, _translate("Form", "19200"))
        self.comboBox_2.setItemText(7, _translate("Form", "38400"))
        self.comboBox_2.setItemText(8, _translate("Form", "57600"))
        self.comboBox_2.setItemText(9, _translate("Form", "76800"))
        self.label_4.setText(_translate("Form", "数据位："))
        self.comboBox_3.setItemText(0, _translate("Form", "8"))
        self.comboBox_3.setItemText(1, _translate("Form", "5"))
        self.comboBox_3.setItemText(2, _translate("Form", "6"))
        self.comboBox_3.setItemText(3, _translate("Form", "7"))
        self.label_5.setText(_translate("Form", "检验为："))
        self.comboBox_4.setItemText(0, _translate("Form", "N"))
        self.label_6.setText(_translate("Form", "停止位："))
        self.comboBox_5.setItemText(0, _translate("Form", "1"))
        self.pushButton_2.setText(_translate("Form", "打开串口"))
        self.pushButton_3.setText(_translate("Form", "关闭串口"))

