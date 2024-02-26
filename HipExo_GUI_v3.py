'''
Entry 02/19/2024
This is the code for the Hip Exoskeleton GUI.
Currently it can connect by Bluetooth with the exoskeleton and recieve data.
This GUI can display data in real-time.
Hoever, it has a delay problem that need to be fixed.
'''
import serial
from serial.tools import list_ports
import struct
import time
import datetime as dt
from math import *
import csv
from PyQt5 import QtWidgets, QtCore, QtGui
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
#from PyQt5.QtWidgets import QComboBox, QApplication, QWidget, QPushButton, QLabel, QLineEdit, QTabWidget, QGridLayout, QVBoxLayout, QHBoxLayout
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys

#### Setting up the serial communication port and baudrate ###
serial_port = 'COM8'
baud_rate = 115200
############################################

### Defining the size of the recieved packages ###
ble_datalength   = 32
rs232_datalength = 20
data_length      = ble_datalength - 3
decoded_data     = [0]*data_length
decode_i = 0
##################################################

win_size  = 200 # Quantity of data points to be displayed in the GUI when real-time plotting
t_buffer                = list([0] * win_size)
L_IMU_buffer            = t_buffer.copy()
R_IMU_buffer            = t_buffer.copy()
L_motor_torque_buffer   = t_buffer.copy()
R_motor_torque_buffer   = t_buffer.copy()
L_motor_torque_d_buffer = t_buffer.copy()
R_motor_torque_d_buffer = t_buffer.copy()

L_leg_IMU_angle = 0
R_leg_IMU_angle = 0
L_motor_torque = 0
R_motor_torque = 0
L_motor_torque_desired = 0
R_motor_torque_desired = 0

red  = pg.mkPen(color=(255, 0, 0), width = 2)
blue = pg.mkPen(color=(0, 0, 255), width = 2)

Connection_Flag   = False
LogginButton_Flag = False
TRefBox_Flag      = False
t_minus_1 = 0
t_0   = 0
tau_d = 0
q_d   = 0

class MainWindow(QWidget):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        global t_buffer, tau_d_buffer, tau_buffer, R_IMU_buffer, L_motor_torque_buffer, R_motor_torque_buffer, L_motor_torque_d_buffer, R_motor_torque_d_buffer,\
            t_0, t_minus_1,\
            tau_d, q_d,\
            ConnectButton, LoggingButton, TRefBox,\
            Connection_Flag

        self.setWindowTitle('Hip Exoskeleton GUI v1.0')
        
        # Layout definition (Nested layout: 3 vert. layouts nested in a hori. layout)
        MainLayout     = QHBoxLayout()
        Left_VLayout   = QVBoxLayout()
        Center_VLayout = QVBoxLayout()
        Right_VLayout  = QVBoxLayout()

        # Creating the Plot objects (Real-time data displays)
        L_IMU_plot          = pg.PlotWidget()
        R_IMU_plot          = pg.PlotWidget()
        L_motor_torque_plot = pg.PlotWidget()
        R_motor_torque_plot = pg.PlotWidget()
        #L_motor_torque_d_plot = pg.PlotWidget()
        
        # Creating the interactive objects in the GUI
        ConnectButton  = QPushButton("Connect Bluetooth")
        TRefButton     = QPushButton("Send Position command")
        LoggingButton  = QPushButton("Start data Logging")
        TRefBox        = QLineEdit(self)

        # Adding functions to the interactive objects in the GUI
        ConnectButton.clicked.connect(Connect_Clicked)       
        TRefButton.clicked.connect(TorqueReference_Clicked)
        LoggingButton.clicked.connect(LogginButton_Clicked)

        # Setup of the widgets on each layout
            # Widgets on the Left vert. layout
        Left_VLayout.addWidget(TRefButton)
        Left_VLayout.addWidget(QPushButton("Left Mid Button"))
        Left_VLayout.addWidget(LoggingButton)
            # Widgets in the center vert. layout
        Center_VLayout.addWidget(TRefBox)
        Center_VLayout.addWidget(QPushButton("Center Mid Button"))
        Center_VLayout.addWidget(ConnectButton)
            # Widgets in the right vert. layout
        Right_VLayout.addWidget(L_IMU_plot)
        Right_VLayout.addWidget(R_IMU_plot)
        Right_VLayout.addWidget(L_motor_torque_plot)
        Right_VLayout.addWidget(R_motor_torque_plot)
        #Right_VLayout.addWidget(L_motor_torque_d_plot)

        # Adding the vert. layouts to the MainLayout (horizontal)
        MainLayout.addLayout(Left_VLayout)
        MainLayout.addLayout(Center_VLayout, stretch=1)
        MainLayout.addLayout(Right_VLayout, stretch=10)

        # Set the Main window layout
        self.setLayout(MainLayout)
        
        # Configuring the look of the plots
        label_style = {"font-size": "16px"}
        title_style = {"color": "black", "font-size": "20px"}

        L_IMU_plot.setTitle("Left IMU", **title_style)
        L_IMU_plot.setLabel('left', "Angle [deg]", **label_style)
        L_IMU_plot.setLabel('bottom', "Time [s]", **label_style)
        L_IMU_plot.addLegend()
        L_IMU_plot.setBackground('w')
        L_IMU_plot.showGrid(x=True, y=True)

        R_IMU_plot.setTitle("Right IMU", **title_style)
        R_IMU_plot.setLabel('left', "Angle [deg]", **label_style)
        R_IMU_plot.setLabel('bottom', "Time [s]", **label_style)
        R_IMU_plot.setBackground('w')
        R_IMU_plot.showGrid(x=True, y=True)

        L_motor_torque_plot.setTitle("Left Motor", **title_style)
        L_motor_torque_plot.setLabel('left', "Torque [Nm]", **label_style)
        L_motor_torque_plot.setLabel('bottom', "Time [s]", **label_style)
        L_motor_torque_plot.setBackground('w')
        L_motor_torque_plot.showGrid(x=True, y=True)

        R_motor_torque_plot.setTitle("Right Motor", **title_style)
        R_motor_torque_plot.setLabel('left', "Torque [Nm]", **label_style)
        R_motor_torque_plot.setLabel('bottom', "Time [s]", **label_style)
        R_motor_torque_plot.setBackground('w')
        R_motor_torque_plot.showGrid(x=True, y=True)

        '''L_motor_torque_d_plot.setTitle("Motor's shaft angular position", **title_style)
        L_motor_torque_d_plot.setLabel('left', "Angular position [deg]", **label_style)
        L_motor_torque_d_plot.setLabel('bottom', "Time [s]", **label_style)
        L_motor_torque_d_plot.addLegend()
        L_motor_torque_d_plot.setBackground('w')
        L_motor_torque_d_plot.showGrid(x=True, y=True)'''
        
        
        # Torque plot lines
        self.data_line1 = L_IMU_plot.plot(t_buffer, L_IMU_buffer, name = "Left IMU", pen = red)
        #self.data_line2 = L_IMU_plot.plot(t_buffer, tau_d_buffer, name = "Reference torque", pen = blue)
        # Gearbox plot lines
        self.data_line3 = R_IMU_plot.plot(t_buffer, R_IMU_buffer, name = "Right IMU", pen = red)
        # Current plot line
        self.data_line4 = L_motor_torque_plot.plot(t_buffer, L_motor_torque_buffer, pen = red)
        # Voltage plot line
        self.data_line5 = R_motor_torque_plot.plot(t_buffer, R_motor_torque_buffer, pen = red)
        # Angular position plot line
        #self.data_line6 = L_motor_torque_d_plot.plot(t_buffer, L_motor_torque_d_buffer, name = "Left Tau_d", pen = red)
        #self.data_line7 = L_motor_torque_d_plot.plot(t_buffer, R_motor_torque_d_buffer, name = "Right Tau_d", pen = blue)

        # Initialization of variables
        self.timer = QtCore.QTimer()
        self.timer.setInterval(20) # Set the refresh time-rate for the plotted data in the GUI
        #t_0 = time.time() # Set the initial time
        #t_minus_1 = t_0 # Time variable to ensure that no repeated data is ploted
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    
    def update_plot_data(self):
        global t_buffer, L_IMU_buffer, R_IMU_buffer, R_IMU_buffer, L_motor_torque_buffer, R_motor_torque_buffer, L_motor_torque_d_buffer, R_motor_torque_d_buffer,\
            L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque, L_motor_torque_desired, R_motor_torque_desired,\
            t_0, t_minus_1,\
            tau_d, q_d,\
            Connection_Flag, LogginButton_Flag, TRefBox_Flag,\
            csv_file_name, DataHeaders,\
            TRefBox_Command

        if Connection_Flag == True:

            t = time.time() - t_0
                
            if t_minus_1 != t:
                        
                if TRefBox_Flag != False:
                    #tau_d = float(eval(TRefBox_Command))
                    q_d = float(eval(TRefBox_Command))/360

                # Input the torque command
                #axis.controller.input_torque = tau_d
                '''axis.controller.input_pos = q_d
                tau = axis.motor.torque_estimate
                gearbox_torque = gear_ratio*tau
                V = odrv.vbus_voltage
                I = axis.motor.foc.Iq_measured
                q = axis.pos_vel_mapper.pos_abs*(360)
                dot_q = axis.pos_vel_mapper.vel * 60 * 6 # Angular velocity in deg/s (CCW+)'''

                Recieve_data()

                t_buffer = t_buffer[1:]
                t_buffer.append(t)

                L_IMU_buffer = L_IMU_buffer[1:]
                L_IMU_buffer.append(L_leg_IMU_angle)

                R_IMU_buffer = R_IMU_buffer[1:]
                R_IMU_buffer.append(R_leg_IMU_angle)

                L_motor_torque_buffer = L_motor_torque_buffer[1:]
                L_motor_torque_buffer.append(L_motor_torque)

                R_motor_torque_buffer = R_motor_torque_buffer[1:]
                R_motor_torque_buffer.append(R_motor_torque)

                L_motor_torque_d_buffer = L_motor_torque_d_buffer[1:]
                L_motor_torque_d_buffer.append(L_motor_torque_desired)

                R_motor_torque_d_buffer = R_motor_torque_d_buffer[1:]
                R_motor_torque_d_buffer.append(R_motor_torque_desired)
            
                self.data_line1.setData(t_buffer, L_IMU_buffer)
                #self.data_line2.setData(t_buffer, tau_d_buffer)

                self.data_line3.setData(t_buffer, R_IMU_buffer)

                self.data_line4.setData(t_buffer, L_motor_torque_buffer)

                self.data_line5.setData(t_buffer, R_motor_torque_buffer)

                #self.data_line6.setData(t_buffer, L_motor_torque_d_buffer)
                #self.data_line7.setData(t_buffer, R_motor_torque_d_buffer)

                if LogginButton_Flag == True:
                    LoggedData = {
                        "time": t,
                        "tau_d": tau_d,
                        "tau": tau,
                        "gearbox_torque": gearbox_torque,
                        "current": I,
                        "bat_volt": V,
                        "ang_pos": q                    
                        }

                    with open(csv_file_name, mode="a", newline="") as file:
                        writer = csv.DictWriter(file, fieldnames = DataHeaders)
                        writer.writerow(LoggedData)
            t_minus_1 = t
        else:
            print("NOT Connected")

def Recieve_data():
    global ser, L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque, L_motor_torque_desired, R_motor_torque_desired

    if ser.read(1) == b'\xA5':  # 165 in uint8
            second_data_byte = ser.read(1)
            if second_data_byte == b'\x5A':  # 90 in uint8
                if ser.read(1) == bytes([ble_datalength]):
                    coded_data = ser.read(data_length)
                    decode_i = 0
                    for i in range(1,data_length,2):
                        var = coded_data[i-1] + coded_data[i]*256
                        var = (var - 65536)/100.0 if var > 32767 else var/100.0
                        decoded_data[decode_i] = var
                        decode_i += 1
                    
                    L_leg_IMU_angle        = decoded_data[0]
                    R_leg_IMU_angle        = decoded_data[1]
                    L_motor_torque         = decoded_data[2]
                    R_motor_torque         = decoded_data[3]
                    L_motor_torque_desired = decoded_data[4]
                    R_motor_torque_desired = decoded_data[5]

                    print(L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque)
    else:
            print('no data recieved')

def Connect_Clicked():
    global ser, Connection_Flag, t_0, serial_port, baud_rate

    ### Stablish the serial connection ###
    ser = serial.Serial(port=serial_port, baudrate=baud_rate)
    ser.timeout = 2 # set read timeout
    ######################################
    while not ser.is_open:
        print('Serial port not open')
    if ser.is_open:
        print('Serial port opened')
        Connection_Flag = True
        t_0 = time.time() # Set the initial time

def TorqueReference_Clicked():
    global TRefBox, TRefBox_Flag, TRefBox_Command
    TRefBox_Flag = True
    TRefBox_Command = str(TRefBox.text())

def LogginButton_Clicked():
    global LogginButton_Flag, LoggingButton, csv_file_name, DataHeaders, t_0
    LogginButton_Flag = True
    t_0 = time.time()
    LoggingButton.setText("Logging data")
    LoggingButton.setStyleSheet("background-color : green")
    csv_file_name = "GUI_Logger_" + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
    DataHeaders = ["time", "tau_d", "tau", "gearbox_torque", "current", "bat_volt", "ang_pos"]

    # Create the CSV file and write the header
    with open(csv_file_name, mode="w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames = DataHeaders)
        writer.writeheader()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    Window = MainWindow()
    Window.show()
    sys.exit(app.exec_())