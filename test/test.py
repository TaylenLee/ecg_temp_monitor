from PySide2.QtWidgets import QApplication, QMainWindow, QPushButton,  QPlainTextEdit
import pyqtgraph as pg
import numpy as np
app = QApplication([])

window = QMainWindow()
window.resize(500, 400)
window.move(300, 310)
window.setWindowTitle('薪资统计')

ecggraph = pg.GraphicsLayoutWidget(window)
ecggraph.resize(500, 400)
ecggraph.addPlot(title="Basic array plotting", y=np.random.normal(size=100))


window.show()
app.exec_()