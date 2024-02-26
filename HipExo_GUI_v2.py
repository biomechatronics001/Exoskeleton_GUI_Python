import serial
import struct
import time

serial_port = 'COM8'
baud_rate = 115200

ble_datalength = 32
rs232_datalength = 20
data_length = ble_datalength - 3
decoded_data = [0]*data_length
decode_i = 0

ser = serial.Serial(port=serial_port, baudrate=baud_rate)
ser.timeout = 2 # set read timeout

def Recieve_data():
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
                    L_motor_torque_desired = decoded_data[5]

                    print(L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque)
    else:
            print('no data recieved')

if ser.is_open:
    print('Serial port opened')
    while ser.is_open == True:
        Recieve_data()
else:
    print('Serial port not open')