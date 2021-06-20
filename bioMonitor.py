import sys
import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import numpy as np
from BioMonitorUI import Ui_Form

class Pyqt5_Serial(QtWidgets.QWidget, Ui_Form):
    def __init__(self):
        super(Pyqt5_Serial, self).__init__()
        self.setupUi(self)
        self.init()
        self.setWindowTitle("BioMonitor")
        self.ser = serial.Serial()
        self.port_check()

        # 初始化体温显示图表
        self.temp_graphics = self.temp_graphicsView.addPlot(title="temp",pen=(200,200,200), symbolBrush=(255,0,0), symbolPen='w')
        self.tempDisplayLen = 50
        self.temp_graphics.setRange(xRange=[0, self.tempDisplayLen - 1], padding=0)
        self.temp_curve = self.temp_graphics.plot()
        self.tempDataDisplay = []

        # 初始化心电显示图表
        self.ecg_graphics = self.ecg_graphicsView.addPlot(title="ecg")
        self.ecgDisplayLen = 4000
        self.ecg_graphics.setRange(xRange=[0, self.ecgDisplayLen-1], padding=0)
        self.ecg_curve = self.ecg_graphics.plot()
        self.ecgDataDisplay = []

        # 初始化b脉搏波显示图表
        self.pluse_graphics = self.pulse_graphicsView.addPlot(title="pluse")
        self.pluseDisplayLen = 4000
        self.pluse_graphics.setRange(xRange=[0, self.pluseDisplayLen - 1], padding=0)
        self.pluse_curve = self.pluse_graphics.plot()
        self.pluseDataDisplay = []

        # 初始化数据解析状态机
        self.stateMachine = StateMachine()


    def init(self):
        # 串口检测按钮
        self.portDetection_pushButton.clicked.connect(self.port_check)

        # 打开串口按钮
        self.openPort_pushButton.clicked.connect(self.port_open)

        # 关闭串口按钮
        self.closePort_pushButton.clicked.connect(self.port_close)

        # 定时器接收数据
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.data_receive)

        # 更新temp波形数据
        self.timer_tempDisplay = QTimer(self)
        self.timer_tempDisplay.timeout.connect(self.plotTempData)
        self.timer_tempDisplay.start(5000)

        # 更新ecg波形数据
        self.timer_ecgDisplay = QTimer(self)
        self.timer_ecgDisplay.timeout.connect(self.plotEcgData)
        self.timer_ecgDisplay.start(2)


    # 串口检测
    def port_check(self):
        # 检测所有存在的串口，将信息存储在字典中
        self.Com_Dict = {}
        port_list = list(serial.tools.list_ports.comports())
        self.portSelect_comboBox.clear()
        for port in port_list:
            self.Com_Dict["%s" % port[0]] = "%s" % port[1]
            self.portSelect_comboBox.addItem(port[0])

    # 打开串口
    def port_open(self):
        self.ser.port = self.portSelect_comboBox.currentText()
        self.ser.baudrate = int(self.baudRate_comboBox.currentText())
        self.ser.bytesize = int(self.dataBit_comboBox.currentText())
        self.ser.stopbits = int(self.stopBit_comboBox.currentText())
        self.ser.parity = self.check_comboBox.currentText()

        try:
            self.ser.open()
        except:
            QMessageBox.critical(self, "Port Error", "此串口不能被打开！")
            return None

        # 打开串口接收定时器，周期为2ms
        self.timer.start(2)

        if self.ser.isOpen():
            self.openPort_pushButton.setEnabled(False)
            self.closePort_pushButton.setEnabled(True)
            self.portStatus_lineEdit.setText("已打开")

    # 关闭串口
    def port_close(self):
        # self.timer.stop()
        try:
            self.ser.close()
        except:
            pass
        self.openPort_pushButton.setEnabled(True)
        self.closePort_pushButton.setEnabled(False)
        self.portStatus_lineEdit.setText("已关闭")

    # 接收数据
    def data_receive(self):
        try:
            num = self.ser.inWaiting()
        except:
            self.port_close()
            return None
        if num > 0:
            data = self.ser.read(num)
            num = len(data)
            # hex显
            out_s = ''
            for i in range(0, len(data)):
                out_s = out_s + '{:02X}'.format(data[i]) + ' '
                self.stateMachine.ParsingData(data[i])
                # print(data[i])
        else:
            pass

    def plotEcgData(self):
        if len(self.ecgDataDisplay) < self.ecgDisplayLen and len(self.stateMachine.EcgDataList) > 0:
            self.ecgDataDisplay.append(self.stateMachine.EcgDataList.pop(0))
            self.ecg_curve.setData(self.ecgDataDisplay)
        elif len(self.ecgDataDisplay) == self.ecgDisplayLen and len(self.stateMachine.EcgDataList) > 0:
            self.ecgDataDisplay[:-1] = self.ecgDataDisplay[1:]
            self.ecgDataDisplay[-1] = self.stateMachine.EcgDataList.pop(0)
            self.ecg_curve.setData(self.ecgDataDisplay)

    def plotTempData(self):
        if len(self.tempDataDisplay) < self.tempDisplayLen and len(self.stateMachine.TempDataList) > 0:
            temp = self.stateMachine.TempDataList.pop(0)
            self.tempDataDisplay.append(temp)
            self.temp_curve.setData(self.tempDataDisplay,pen=(200,200,200), symbolBrush=(255,0,0), symbolPen='w')
            self.temp_lineEdit.setText(str(temp))
        elif len(self.tempDataDisplay) == self.tempDisplayLen and len(self.stateMachine.TempDataList) > 0:
            self.tempDataDisplay[:-1] = self.tempDataDisplay[1:]
            temp = self.stateMachine.TempDataList.pop(0)
            self.tempDataDisplay[-1] = temp
            self.temp_curve.setData(self.tempDataDisplay)
            self.temp_lineEdit.setText(str(temp))


class StateMachine:
    def __init__(self):
        # 有限状态机状态码
        self.SyncHeader1 = 0
        self.SyncHeader2 = 1
        self.DataLen = 2
        self.Opcode = 3
        self.GetDataCategory = 4
        self.GetData = 5
        self.CheckCode = 6
        # 协议包数据
        self.PacketHeader1 = 0xaa
        self.PacketHeader2 = 0xaa
        self.PaketDataLen = 0x06
        self.PacketOpcode = 0x80
        self.EcgCategory = 0x02
        self.TemperCategory = 0x04
        # 数据类型flag,1为ecg,2为temper
        self.DataCategory = 0
        # 状态机状态初始化
        self.receive_state = self.SyncHeader1
        self.DataLength = 0
        self.DataHighByte = 0
        # 数据缓存
        self.EcgDataList = []
        self.TempDataList = []
        # 原始数据
        # self.ecg_data = 0.0

    def ParsingData(self, rx_data):
        if self.receive_state == self.SyncHeader1:
            if rx_data == self.PacketHeader1:
                self.receive_state = self.SyncHeader2
        elif self.receive_state == self.SyncHeader2:
            if rx_data == self.PacketHeader2:
                self.receive_state = self.DataLen
            else:
                self.receive_state = self.SyncHeader1
        elif self.receive_state == self.DataLen:
            if rx_data == self.PaketDataLen:
                self.receive_state = self.Opcode
            else:
                self.receive_state = self.SyncHeader1
        elif self.receive_state == self.Opcode:
            if rx_data == self.PacketOpcode:
                self.receive_state = self.GetDataCategory
            else:
                self.receive_state = self.SyncHeader1
        elif self.receive_state == self.GetDataCategory:
            if rx_data == self.EcgCategory:
                self.receive_state = self.GetData
                self.DataCategory = 1
            elif rx_data == self.TemperCategory:
                self.receive_state = self.GetData
                self.DataCategory = 2
            else:
                self.receive_state = self.SyncHeader1
        elif self.receive_state == self.GetData:
            if self.DataLength == 1 or self.DataLength == 3:
                # ecg_data = self.DataHighByte * 256.0 + rx_data
                # if ecg_data > 32767:
                #     ecg_data -= 65536
                if self.DataCategory == 1:
                    ecg_data = self.DataHighByte * 256.0 + rx_data
                    if ecg_data > 32767:
                        ecg_data -= 65536
                    ecg_data = round((ecg_data * 18.3) / 128.0 / 1000.0, 3)
                    self.EcgDataList.append(ecg_data)
                    # print(ecg_data)
                    print(len(self.EcgDataList))
                elif self.DataCategory == 2 and self.DataLength == 3:
                    temp_data = self.DataHighByte + rx_data / 100.0
                    self.TempDataList.append(temp_data)
                    # print(len(self.TempDataList))
                self.DataLength += 1
                if self.DataLength == 4:
                    self.receive_state = self.CheckCode
                    self.DataLength = 0
            else:
                self.DataLength += 1
                self.DataHighByte = rx_data
        elif self.receive_state == self.CheckCode:
            self.receive_state = self.SyncHeader1
            self.DataLength = 0
            self.DataCategory = 0

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    myshow = Pyqt5_Serial()
    myshow.show()
    sys.exit(app.exec_())




