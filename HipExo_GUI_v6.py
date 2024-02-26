'''
Entry 02/21/2024
This is an improvement of the runnig version of "HipExo_GUI_004.py".
THE DELAY ISSUE RETURNED. STILL DONT KNOW THE REASON.

UPDATE: The delay is due to the plotting command.
        It happens when plotting too many signals (>6).

The following was added:
1. Modified the GUI structure to display data from Left and Rigth Motors
2. The Connect button now displays different text based on connection status and change its color

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
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys

def find_available_ports():
    connected_ports = []

    # Get a list of all available ports
    available_ports = list(list_ports.comports())

    for port, desc, hwid in available_ports:
        try:
            # Attempt to open the serial port
            ser = serial.Serial(port)
            
            # Check if there is something connected to the port
            if ser.readable():
                connected_ports.append(port)
                
            # Close the serial port
            ser.close()
        except serial.SerialException:
            pass

    return connected_ports

win_size  = 150 # Quantity of data points to be displayed in the GUI when real-time plotting
t_buffer                = list([0] * win_size)
L_IMU_buffer            = t_buffer.copy()
R_IMU_buffer            = t_buffer.copy()
BattVolt_buffer         = t_buffer.copy()
L_motor_torque_buffer   = t_buffer.copy()
R_motor_torque_buffer   = t_buffer.copy()
L_motor_torque_d_buffer = t_buffer.copy()
R_motor_torque_d_buffer = t_buffer.copy()
L_motor_angpos_buffer   = t_buffer.copy()
R_motor_angpos_buffer   = t_buffer.copy()

L_leg_IMU_angle = 0
R_leg_IMU_angle = 0
L_motor_torque = 0
R_motor_torque = 0
L_motor_torque_desired = 0
R_motor_torque_desired = 0
L_motor_angpos = 0
R_motor_angpos = 0

red  = pg.mkPen(color=(255, 0, 0), width = 2)
blue = pg.mkPen(color=(0, 0, 255), width = 2)

Connection_Flag   = False
LogginButton_Flag = False
TRefBox_Flag      = False
t_teensy = 0
t_minus_1 = 0
t_0   = 0
tau_d = 0
q_d   = 0

class MainWindow(QWidget):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        global t_buffer, L_IMU_buffer, R_IMU_buffer, BattVolt_buffer, L_motor_torque_buffer, R_motor_torque_buffer,\
            L_motor_torque_d_buffer, R_motor_torque_d_buffer, L_motor_angpos_buffer, R_motor_angpos_buffer,\
            t_0, t_minus_1,\
            tau_d, q_d,\
            ConnectButton, TRefButton, LoggingButton, TRefBox, SerialComboBox,\
            Connection_Flag, connected_ports, recieve_data_flag
        
        recieve_data_flag = False
        
        self.setWindowTitle('Exoskeleton GUI v1.0')
        
        connected_ports = find_available_ports()

        # Layout definition
        MainLayout    = QVBoxLayout()
        TopLayout     = QHBoxLayout()
        BottomLayout  = QHBoxLayout()
        GenLeftLayout = QVBoxLayout()
        LMotorLayout  = QVBoxLayout()
        RMotorLayout  = QVBoxLayout()

        # Adding the sublayouts to the Main Layout
        MainLayout.addLayout(TopLayout)
        MainLayout.addLayout(BottomLayout, stretch=5)

        # Adding the subsublayouts to the Bottom Layouts
        BottomLayout.addLayout(GenLeftLayout)
        BottomLayout.addLayout(LMotorLayout, stretch=5)
        BottomLayout.addLayout(RMotorLayout, stretch=5)

        # Set the Main window layout
        self.setLayout(MainLayout)

        # Creating the Plot objects (Real-time data displays)
        LnR_IMU_plot        = pg.PlotWidget()
        BattVolt_plot       = pg.PlotWidget()
        L_Motor_TnTd_plot   = pg.PlotWidget()
        L_Motor_AngPos_plot = pg.PlotWidget()
        R_Motor_TnTd_plot   = pg.PlotWidget()
        R_Motor_AngPos_plot = pg.PlotWidget()
        
        # Creating the interactive objects in the GUI
        #   Buttons
        ConnectButton = QPushButton("Connect Bluetooth")
        TRefButton    = QPushButton("Send Position command")
        LoggingButton = QPushButton("Start data Logging")

        #   Text capture box
        TRefBox = QLineEdit(self)

        #   Combo box
        SerialComboBox = QComboBox()

        # Adding functions to the interactive objects in the GUI
        ConnectButton.clicked.connect(Connect_Clicked)
        TRefButton.clicked.connect(TorqueReference_Clicked)
        LoggingButton.clicked.connect(LogginButton_Clicked)


        # Setup of the widgets on each layout
        #   TOP LAYOUT Widgets
        TopLayout.addWidget(QLabel("ComPort:"))
        TopLayout.addWidget(SerialComboBox)
        SerialComboBox.addItems(connected_ports)
        TopLayout.addWidget(ConnectButton)
        TopLayout.addWidget(LoggingButton)

        #   BOTTOM GENERAL LAYOUT Widgets
        GenLeftLayout.addWidget(QLabel("Command"))
        GenLeftLayout.addWidget(TRefBox)
        GenLeftLayout.addWidget(TRefButton)
        GenLeftLayout.addWidget(LnR_IMU_plot)
        GenLeftLayout.addWidget(BattVolt_plot) 

        #   BOTTOM LEFT MOTOR LAYOUT Widgets
        LMotorLayout.addWidget(L_Motor_TnTd_plot)
        LMotorLayout.addWidget(L_Motor_AngPos_plot) 

        #   BOTTOM RIGHT MOTOR LAYOUT Widgets
        RMotorLayout.addWidget(R_Motor_TnTd_plot)
        RMotorLayout.addWidget(R_Motor_AngPos_plot)

               
        # Configuring the look of the plots
        label_style = {"font-size": "16px"}
        title_style = {"color": "black", "font-size": "20px"}

        LnR_IMU_plot.setTitle("IMU", **title_style)
        LnR_IMU_plot.setLabel('left', "Angle [deg]", **label_style)
        LnR_IMU_plot.setLabel('bottom', "Time [s]", **label_style)
        LnR_IMU_plot.addLegend()
        LnR_IMU_plot.setBackground('w')
        LnR_IMU_plot.showGrid(x=True, y=True)
        self.L_IMU_line = LnR_IMU_plot.plot(t_buffer, L_IMU_buffer, name = "Left IMU", pen = red)
        self.R_IMU_line = LnR_IMU_plot.plot(t_buffer, R_IMU_buffer, name = "Right IMU", pen = blue)

        BattVolt_plot.setTitle("Battery", **title_style)
        BattVolt_plot.setLabel('left', "Voltage [v]", **label_style)
        BattVolt_plot.setLabel('bottom', "Time [s]", **label_style)
        BattVolt_plot.setBackground('w')
        BattVolt_plot.showGrid(x=True, y=True)
        self.BattVolt_line = BattVolt_plot.plot(t_buffer, BattVolt_buffer, pen = red)

        L_Motor_TnTd_plot.setTitle("M1 Torque (Left)", **title_style)
        L_Motor_TnTd_plot.setLabel('left', "Torque [Nm]", **label_style)
        L_Motor_TnTd_plot.setLabel('bottom', "Time [s]", **label_style)
        L_Motor_TnTd_plot.addLegend()
        L_Motor_TnTd_plot.setBackground('w')
        L_Motor_TnTd_plot.showGrid(x=True, y=True)
        self.L_Motor_Taud_line = L_Motor_TnTd_plot.plot(t_buffer, L_motor_torque_d_buffer, name = "Command", pen = blue)
        self.L_Motor_Tau_line  = L_Motor_TnTd_plot.plot(t_buffer, L_motor_torque_buffer, name = "Actual", pen = red)
        
        L_Motor_AngPos_plot.setTitle("M1 Position (Left)", **title_style)
        L_Motor_AngPos_plot.setLabel('left', "Ang. Position [deg]", **label_style)
        L_Motor_AngPos_plot.setLabel('bottom', "Time [s]", **label_style)
        L_Motor_AngPos_plot.setBackground('w')
        L_Motor_AngPos_plot.showGrid(x=True, y=True)
        self.L_motor_angpos_line = L_Motor_AngPos_plot.plot(t_buffer, L_motor_angpos_buffer, pen = red)

        R_Motor_TnTd_plot.setTitle("M2 Torque (Right)", **title_style)
        R_Motor_TnTd_plot.setLabel('left', "Torque [Nm]", **label_style)
        R_Motor_TnTd_plot.setLabel('bottom', "Time [s]", **label_style)
        R_Motor_TnTd_plot.addLegend()
        R_Motor_TnTd_plot.setBackground('w')
        R_Motor_TnTd_plot.showGrid(x=True, y=True)
        self.R_Motor_Taud_line = R_Motor_TnTd_plot.plot(t_buffer, R_motor_torque_d_buffer, name = "Command", pen = blue)
        self.R_Motor_Tau_line  = R_Motor_TnTd_plot.plot(t_buffer, R_motor_torque_buffer, name = "Actual", pen = red)
        
        R_Motor_AngPos_plot.setTitle("M2 Position (Right)", **title_style)
        R_Motor_AngPos_plot.setLabel('left', "Ang. Position [deg]", **label_style)
        R_Motor_AngPos_plot.setLabel('bottom', "Time [s]", **label_style)
        R_Motor_AngPos_plot.setBackground('w')
        R_Motor_AngPos_plot.showGrid(x=True, y=True)
        self.R_motor_angpos_line = R_Motor_AngPos_plot.plot(t_buffer, R_motor_angpos_buffer, pen = red)

        # Creation of the timer for executing the function repetitively
        self.timer = QtCore.QTimer()
        self.timer.setInterval(20) # Set the refresh time-rate for the plotted data in the GUI
        self.timer.timeout.connect(self.all) # This function is called 20 times per second [50Hz] (Fastests stable)
        self.timer.start()


    def all(self):
        global Connection_Flag, recieve_data_flag
        recieve_data_flag = False
        if Connection_Flag:
            self.Recieve_data()
            if recieve_data_flag:
                self.update_plot_data()


    def Recieve_data(self):
        global ser, ble_datalength, data_length, decoded_data, recieve_data_flag,\
            L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque,\
            L_motor_torque_desired, R_motor_torque_desired, t_teensy, L_motor_angpos, R_motor_angpos
        
        if ser.read(1) == b'\xA5':  # 165 in uint8
            second_data_byte = ser.read(1)
            if second_data_byte == b'\x5A':  # 90 in uint8
                if ser.read(1) == bytes([ble_datalength]):

                    recieve_data_flag = True
                    ConnectButton.setText("Connected")
                    ConnectButton.setStyleSheet("background-color : green")

                    coded_data = ser.read(data_length)
                    decode_i = 0
                    for i in range(1, data_length, 2):
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
                    t_teensy               = decoded_data[6]
                    L_motor_angpos         = decoded_data[7]
                    R_motor_angpos         = decoded_data[8]

                    print(L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque)
        else:
            print('no data received')
            ConnectButton.setText("Searching Hip Exoskeleton")
            ConnectButton.setStyleSheet("background-color : orange")

    
    def update_plot_data(self):
        global t_buffer, L_IMU_buffer, R_IMU_buffer, BattVolt_buffer, L_motor_torque_buffer, R_motor_torque_buffer,\
            L_motor_torque_d_buffer, R_motor_torque_d_buffer, L_motor_angpos_buffer, R_motor_angpos_buffer,\
            L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque,\
            L_motor_torque_desired, R_motor_torque_desired, t_teensy, L_motor_angpos, R_motor_angpos,\
            t_0, t_minus_1,\
            tau_d, q_d,\
            Connection_Flag, LogginButton_Flag, TRefBox_Flag, recieve_data_flag,\
            csv_file_name, DataHeaders,\
            TRefBox_Command

        if Connection_Flag == True:

            t = time.time() - t_0
                
            if t_minus_1 != t:
                        
                if TRefBox_Flag != False:
                    #tau_d = float(eval(TRefBox_Command))
                    q_d = float(eval(TRefBox_Command))/360

                self.Recieve_data()

                t_buffer = t_buffer[1:]
                t_buffer.append(t)

                L_IMU_buffer = L_IMU_buffer[1:]
                L_IMU_buffer.append(L_leg_IMU_angle)

                R_IMU_buffer = R_IMU_buffer[1:]
                R_IMU_buffer.append(R_leg_IMU_angle)

                BattVolt_buffer = BattVolt_buffer[1:] # This is just for testing not true batt voltage
                BattVolt_buffer.append(t_teensy)

                L_motor_torque_buffer = L_motor_torque_buffer[1:]
                L_motor_torque_buffer.append(L_motor_torque)

                L_motor_torque_d_buffer = L_motor_torque_d_buffer[1:]
                L_motor_torque_d_buffer.append(L_motor_torque_desired)

                L_motor_angpos_buffer = L_motor_angpos_buffer[1:]
                L_motor_angpos_buffer.append(L_motor_angpos)

                R_motor_torque_buffer = R_motor_torque_buffer[1:]
                R_motor_torque_buffer.append(R_motor_torque)

                R_motor_torque_d_buffer = R_motor_torque_d_buffer[1:]
                R_motor_torque_d_buffer.append(R_motor_torque_desired)

                R_motor_angpos_buffer = R_motor_angpos_buffer[1:]
                R_motor_angpos_buffer.append(R_motor_angpos)
            
                self.L_IMU_line.setData(t_buffer, L_IMU_buffer)
                self.R_IMU_line.setData(t_buffer, R_IMU_buffer)

                #self.BattVolt_line.setData(t_buffer, BattVolt_buffer)

                self.L_Motor_Taud_line.setData(t_buffer, L_motor_torque_d_buffer)
                self.L_Motor_Tau_line.setData(t_buffer, L_motor_torque_buffer)

                #self.L_motor_angpos_line.setData(t_buffer, L_motor_angpos_buffer)

                self.R_Motor_Taud_line.setData(t_buffer, R_motor_torque_d_buffer)
                self.R_Motor_Tau_line.setData(t_buffer, R_motor_torque_buffer)

                #self.R_motor_angpos_line.setData(t_buffer, R_motor_angpos_buffer)

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


def Connect_Clicked():
        global ConnectButton, ser, Connection_Flag, t_0, ble_datalength, data_length, decoded_data

        ### Defining the size of the received packages ###
        ble_datalength   = 32 # Recieved package data size
        rs232_datalength = 20 # Transmited package data size
        data_length      = ble_datalength - 3
        decoded_data     = [0]*data_length
        ##################################################

        #### Setting up the serial communication port and baudrate ###
        serial_port = SerialComboBox.currentText()
        baud_rate   = 115200
        ############################################

        ### Stablish the serial connection ###
        ser = serial.Serial(port=serial_port, baudrate=baud_rate)
        ser.timeout = 1 # set read timeout
        ######################################

        while not ser.is_open:
            print('Serial port not open')
        if ser.is_open:
            print('Serial port opened')
            ConnectButton.setText("Bluetooth activated")
            ConnectButton.setStyleSheet("background-color : yellow")
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