#!/usr/bin/env python3

# PYQT Stuff
from PyQt5.QtWidgets import QApplication, QPushButton, QLineEdit, QLabel, QWidget, QSlider, QSpinBox, QComboBox, QMainWindow
from PyQt5.QtGui import QPainter, QColor, QPen, QFont, QBrush
from PyQt5.QtCore import Qt, QTimer
from PyQt5 import QtWidgets
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg

# Libraries
import rospy
import threading
import os
import sys

# Messages
from current_sense_node.msg import motor_currents

# Services
from current_sense_node.srv import max40080_service

adc_sample_rate_arr = [15, 23.45, 30, 37.5, 47.1, 60, 93.5, 120, 150, 234.5, 375, 468.5, 750, 1000]
digital_filter_arr  = [0, 8, 16, 32, 64, 128]
x_points_current    = []
current_data        = []
batch_size          = 500
counter             = 0
filter_size         = 0
adc_sample_rate     = 0
input_range         = 0
filter_set          = 0
adc_set             = 15.0
range_set           = 1

# Design Variables
borderColor = QColor(180, 180, 180)
borderWidth = 2

class GUIWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.title = 'MAX40080 Live Data'
        self.left = 200
        self.top = 200
        self.width = 1920
        self.height = 1000
        self.qTimer = QTimer()
        self.qTimer.setInterval(50)    
        self.qTimer.start()
        self.qTimer.timeout.connect(self.update_plot_data)
        self.qTimer.timeout.connect(self.updateEvent)
        self.initUI()

    def initUI(self):
        global x_points_current, current_data, filter_size, adc_sample_rate, input_range

        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height) 
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QColor(20, 30, 35))
        self.setPalette(p)

        # Graph Widger
        self.current_graph = pg.PlotWidget(self)
        self.current_graph.setGeometry(600, 20, 1300, 960)
        self.current_graph.setBackground(None)
        self.current_graph.setYRange(0, 900, padding=0.04)

        data_pen = pg.mkPen(color = (255, 7, 58), width = 2)
        self.current_dataline = self.current_graph.plot(x_points_current, current_data, pen = data_pen)

        # Labels for the current configuration [cc]
        self.label_cc_title = QLabel(self)
        self.label_cc_title.setText('Current configuration')
        self.label_cc_title.setGeometry(20, 20, 560, 60)
        self.label_cc_title.setFont(QFont("Arial", 28))
        self.label_cc_title.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_cc_title.setAlignment(QtCore.Qt.AlignCenter)

        self.label_ccn_filter = QLabel(self)
        self.label_ccn_filter.setText('Filter Size')
        self.label_ccn_filter.setGeometry(20, 80, 280, 60)
        self.label_ccn_filter.setFont(QFont("Arial", 20))
        self.label_ccn_filter.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_ccn_filter.setAlignment(QtCore.Qt.AlignCenter)

        self.label_ccn_adc = QLabel(self)
        self.label_ccn_adc.setText('ADC Sample Rate')
        self.label_ccn_adc.setGeometry(20, 140, 280, 60)
        self.label_ccn_adc.setFont(QFont("Arial", 20))
        self.label_ccn_adc.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_ccn_adc.setAlignment(QtCore.Qt.AlignCenter)

        self.label_ccn_range = QLabel(self)
        self.label_ccn_range.setText('Input Range')
        self.label_ccn_range.setGeometry(20, 200, 280, 60)
        self.label_ccn_range.setFont(QFont("Arial", 20))
        self.label_ccn_range.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_ccn_range.setAlignment(QtCore.Qt.AlignCenter)

        # Labels containing the data
        self.label_cc_filter = QLabel(self)
        self.label_cc_filter.setText(str(filter_size))
        self.label_cc_filter.setGeometry(300, 80, 280, 60)
        self.label_cc_filter.setFont(QFont("Arial", 20))
        self.label_cc_filter.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_cc_filter.setAlignment(QtCore.Qt.AlignCenter)

        self.label_cc_adc = QLabel(self)
        self.label_cc_adc.setText(str(adc_sample_rate) + 'ksps')
        self.label_cc_adc.setGeometry(300, 140, 280, 60)
        self.label_cc_adc.setFont(QFont("Arial", 20))
        self.label_cc_adc.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_cc_adc.setAlignment(QtCore.Qt.AlignCenter)

        self.label_cc_range = QLabel(self)
        self.label_cc_range.setText(str(input_range) + 'A')
        self.label_cc_range.setGeometry(300, 200, 280, 60)
        self.label_cc_range.setFont(QFont("Arial", 20))
        self.label_cc_range.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_cc_range.setAlignment(QtCore.Qt.AlignCenter)

        #
        # Labels for the update configuration [uc]
        #
        self.label_uc_title = QLabel(self)
        self.label_uc_title.setText('Set New Configuration')
        self.label_uc_title.setGeometry(20, 280, 560, 60)
        self.label_uc_title.setFont(QFont("Arial", 28))
        self.label_uc_title.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_uc_title.setAlignment(QtCore.Qt.AlignCenter)

        self.label_ucn_filter = QLabel(self)
        self.label_ucn_filter.setText('Filter')
        self.label_ucn_filter.setGeometry(20, 340, 160, 90)
        self.label_ucn_filter.setFont(QFont("Arial", 24))
        self.label_ucn_filter.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_ucn_filter.setAlignment(QtCore.Qt.AlignCenter)

        self.label_ucn_adc = QLabel(self)
        self.label_ucn_adc.setText('ADC')
        self.label_ucn_adc.setGeometry(20, 430, 160, 90)
        self.label_ucn_adc.setFont(QFont("Arial", 24))
        self.label_ucn_adc.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_ucn_adc.setAlignment(QtCore.Qt.AlignCenter)

        self.label_ucn_range = QLabel(self)
        self.label_ucn_range.setText('Range')
        self.label_ucn_range.setGeometry(20, 520, 160, 90)
        self.label_ucn_range.setFont(QFont("Arial", 24))
        self.label_ucn_range.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_ucn_range.setAlignment(QtCore.Qt.AlignCenter)

        # Filter value slider and label
        self.slider_uc_filter = QSlider(Qt.Horizontal, self)
        self.slider_uc_filter.setGeometry(190, 350, 300, 70)
        self.slider_uc_filter.setRange(0,5)
        self.slider_uc_filter.valueChanged[int].connect(self.changeFilterSlider)

        self.label_uc_filter = QLabel(self)
        self.label_uc_filter.setText('0')
        self.label_uc_filter.setGeometry(490, 340, 100, 90)
        self.label_uc_filter.setFont(QFont("Arial", 20))
        self.label_uc_filter.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_uc_filter.setAlignment(QtCore.Qt.AlignCenter)

        # ADC value slider and label
        self.slider_uc_adc = QSlider(Qt.Horizontal, self)
        self.slider_uc_adc.setGeometry(190, 440, 300, 70)
        self.slider_uc_adc.setRange(0,13)
        self.slider_uc_adc.valueChanged[int].connect(self.changeADCSlider)

        self.label_uc_adc = QLabel(self)
        self.label_uc_adc.setText('15')
        self.label_uc_adc.setGeometry(490, 430, 100, 90)
        self.label_uc_adc.setFont(QFont("Arial", 20))
        self.label_uc_adc.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_uc_adc.setAlignment(QtCore.Qt.AlignCenter)

        # Range value slider and label
        self.slider_uc_range = QSlider(Qt.Horizontal, self)
        self.slider_uc_range.setGeometry(190, 530, 300, 70)
        self.slider_uc_range.setRange(0,1)
        self.slider_uc_range.valueChanged[int].connect(self.changeRangeSlider)

        self.label_uc_range = QLabel(self)
        self.label_uc_range.setText('1')
        self.label_uc_range.setGeometry(490, 520, 100, 90)
        self.label_uc_range.setFont(QFont("Arial", 20))
        self.label_uc_range.setStyleSheet("QLabel {color : rgb(150, 150, 150)}")
        self.label_uc_range.setAlignment(QtCore.Qt.AlignCenter)

        # Update Config button
        self.button_uc = QPushButton("Update Configuration", self)
        self.button_uc.resize(360, 50)
        self.button_uc.move(120, 620)
        self.button_uc.clicked.connect(self.buttonUpdateFunction)
        self.button_uc.setFont(QFont("Arial", 20))
        self.button_uc.setStyleSheet("QPushButton {background-color : rgb(35, 45, 55); color : rgb(150, 150, 150); font-weight: bold}")

        self.show()

    def updateEvent(self):
        global filter_size, adc_sample_rate, input_range

        self.label_cc_filter.setText(str(filter_size))
        self.label_cc_adc.setText(str(adc_sample_rate) + 'ksps')
        self.label_cc_range.setText(str(input_range) + 'A')

        self.update()

    def update_plot_data(self):
        global x_points_current, current_data, batch_size

        if len(x_points_current) > batch_size:
            self.current_dataline.setData(x_points_current[len(x_points_current) - batch_size:], current_data[len(x_points_current) - batch_size:])
        else:
            self.current_dataline.setData(x_points_current, current_data)

    def paintEvent(self, event):
        global borderColor, borderWidth

        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing)

        painter.setPen(QPen(borderColor, borderWidth))

        # Current configuration rectnagle
        painter.drawRect(20, 20, 560, 240)
        painter.drawLine(20, 80, 580, 80)
        painter.drawRect(20, 280, 560, 400)
        painter.drawLine(20, 340, 580, 340)
        painter.drawLine(180, 340, 180, 610)
        painter.drawLine(20, 610, 580, 610)

        # Lines
        painter.setPen(QPen(borderColor, borderWidth - 1))
        painter.drawLine(20, 140, 580, 140)
        painter.drawLine(20, 200, 580, 200)
        painter.drawLine(290, 80, 290, 260)
        painter.drawLine(20, 430, 580, 430)
        painter.drawLine(20, 520, 580, 520)

    def changeFilterSlider(self, value):
        global filter_set, digital_filter_arr
        filter_set = digital_filter_arr[value]
        self.label_uc_filter.setText(str(filter_set))

    def changeADCSlider(self, value):
        global adc_set, adc_sample_rate_arr
        adc_set = adc_sample_rate_arr[value]
        self.label_uc_adc.setText(str(adc_set))

    def changeRangeSlider(self, value):
        global range_set
        if value == 0:
            range_set = 1
        else:
            range_set = 5
        self.label_uc_range.setText(str(range_set))

    def buttonUpdateFunction(self):
        global filter_set, adc_set, range_set, filter_size, adc_sample_rate, input_range
        resp = srv('config', 1, filter_set, adc_set, range_set)
        if resp.response == True:
            filter_size = resp.filter_size
            adc_sample_rate = resp.adc_sample_rate
            input_range = resp.range
        


class Controller:

    def __init__(self):
        pass

    def show_gui_window(self):
        self.gui_window = GUIWindow()
        self.gui_window.show()

def sub_callback(req):
    global x_points_current, current_data, counter
    new_measurement = req.motor_current[0]
    current_data.append(new_measurement)
    x_points_current.append(counter)
    counter += 1

# Main function
if __name__ == "__main__":
    # Configuring the ROS Node
    rospy.init_node('MAX40080_GUI', anonymous = False)
    sub = rospy.Subscriber('/sensor/motor_current', motor_currents, sub_callback)
    srv = rospy.ServiceProxy('/sensor/max40080', max40080_service)

    # On startup, we need to read the current sensor configuration
    resp = srv('read', 1, 0, 0, 0)

    # If the response of the reading was True, we can update the data
    if resp.response == True:
        filter_size     = resp.filter_size
        adc_sample_rate = resp.adc_sample_rate
        input_range     = resp.range

    # Turning on the GUI
    app = QApplication(sys.argv)
    controller = Controller()
    controller.show_gui_window()
    sys.exit(app.exec_())