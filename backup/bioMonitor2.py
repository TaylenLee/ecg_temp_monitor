import sys
import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import numpy as np
from backup.ui_main import Ui_Form

class Pyqt5_Serial(QtWidgets.QWidget, Ui_Form):
    def __init__(self):
        super(Pyqt5_Serial, self).__init__()
        self.setupUi(self)
        self.init()
        self.setWindowTitle("串口小助手")
        self.ser = serial.Serial()
        self.port_check()

        # 接收数据和发送数据数目置零
        self.data_num_received = 0
        self.lineEdit.setText(str(self.data_num_received))

        self.pha = 0
        self.t = np.arange(1024) / 1024.0
        self.ecggraph = pg.GraphicsLayoutWidget(self.graphicsView)
        self.ecggraph.resize(516, 220)
        self.p1 = self.ecggraph.addPlot(title="ecg")
        self.p1.setRange(xRange=[0, 2000-1], padding=0)
        self.curve1 = self.p1.plot()
        self.stateMachine = StateMachine()

        self.ecgDataDisplay = []
        self.ecgDisplayLen = 2000


    def init(self):
        # 串口检测按钮
        self.pushButton.clicked.connect(self.port_check)

        # 打开串口按钮
        self.pushButton_2.clicked.connect(self.port_open)

        # 关闭串口按钮
        self.pushButton_3.clicked.connect(self.port_close)

        # 定时器接收数据
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.data_receive)

        # 更新波形数据
        self.timer1 = QTimer(self)
        self.timer1.timeout.connect(self.plotData)
        self.timer1.start(2)


    # 串口检测
    def port_check(self):
        # 检测所有存在的串口，将信息存储在字典中
        self.Com_Dict = {}
        port_list = list(serial.tools.list_ports.comports())
        self.comboBox.clear()
        for port in port_list:
            self.Com_Dict["%s" % port[0]] = "%s" % port[1]
            self.comboBox.addItem(port[0])
        if len(self.Com_Dict) == 0:
            self.state_label.setText(" 无串口")

    # 打开串口
    def port_open(self):
        self.ser.port = self.comboBox.currentText()
        self.ser.baudrate = int(self.comboBox_2.currentText())
        self.ser.bytesize = int(self.comboBox_3.currentText())
        self.ser.stopbits = int(self.comboBox_5.currentText())
        self.ser.parity = self.comboBox_4.currentText()

        try:
            self.ser.open()
        except:
            QMessageBox.critical(self, "Port Error", "此串口不能被打开！")
            return None

        # 打开串口接收定时器，周期为2ms
        self.timer.start(2)

        if self.ser.isOpen():
            self.pushButton_2.setEnabled(False)
            self.pushButton_3.setEnabled(True)
            self.groupBox.setTitle("串口状态（已开启）")

    # 关闭串口
    def port_close(self):
        # self.timer.stop()
        try:
            self.ser.close()
        except:
            pass
        self.pushButton_2.setEnabled(True)
        self.pushButton_3.setEnabled(False)
        # 接收数据和发送数据数目置零
        self.data_num_received = 0
        self.lineEdit.setText(str(self.data_num_received))
        self.groupBox.setTitle("串口状态（已关闭）")

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
            # 统计接收字符的数量
            self.data_num_received += num
            if self.data_num_received >= 768000:
                self.data_num_received = 0
            self.lineEdit.setText(str(self.data_num_received))
        else:
            pass

    def plotData(self):
        if len(self.ecgDataDisplay) < self.ecgDisplayLen and len(self.stateMachine.EcgDataList) > 0:
            self.ecgDataDisplay.append(self.stateMachine.EcgDataList.pop(0))
            self.curve1.setData(self.ecgDataDisplay)
        elif len(self.ecgDataDisplay) == self.ecgDisplayLen and len(self.stateMachine.EcgDataList) > 0:
            self.ecgDataDisplay[:-1] = self.ecgDataDisplay[1:]
            self.ecgDataDisplay[-1] = self.stateMachine.EcgDataList.pop(0)
            self.curve1.setData(self.ecgDataDisplay)

class Waveform_Display(QtWidgets.QWidget, Ui_Form):
    def __init__(self):
        super(Waveform_Display, self).__init__()
        self.setupUi(self)
        # self.init()
        self.pha = 0
        self.t = np.arange(1024)/1024.0
        self.ecggraph = pg.GraphicsLayoutWidget(self.graphicsView)
        self.ecggraph.resize(516, 220)
        self.p1 = self.ecggraph.addPlot(title="ecg")
        self.curve1 = self.p1.plot()
        self.init()


    def init(self):

        # 更新波形数据
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.plotData)
        self.timer.start(1000)

    def plotData(self):
        self.pha += 10
        self.curve1.setData(self.t, np.sin(2*np.pi*4.0*self.t+self.pha*np.pi/180.0))

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
        self.TemperDataList = []
        # 原始数据
        self.data = 0.0

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
                self.data = self.DataHighByte * 256.0 + rx_data
                if self.data > 32767:
                    self.data -= 65536
                if self.DataCategory == 1:
                    self.data = round((self.data * 18.3) / 128.0 / 1000.0, 3)
                    self.EcgDataList.append(self.data)
                    # print(self.data)
                    print(len(self.EcgDataList))
                elif self.DataCategory == 2:
                    self.TemperDataList.append(self.data)
                    if self.TemperDataList.count() == 100:
                        self.TemperDataList.clear()
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
    # wave_disp = Waveform_Display()
    myshow.show()
    # wave_disp.show()
    sys.exit(app.exec_())




