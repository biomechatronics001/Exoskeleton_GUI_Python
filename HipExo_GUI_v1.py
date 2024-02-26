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

if ser.is_open:
    print('Serial port opened')
    while ser.is_open == True:
        Recieve_data()
else:
    print('Serial port not open')

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

                    '''L_leg_IMU_angle = coded_data[0] + coded_data[1]*256
                    L_leg_IMU_angle = L_leg_IMU_angle - 65536 if L_leg_IMU_angle > 32767 else L_leg_IMU_angle
                    L_leg_IMU_angle = L_leg_IMU_angle / 100.0
                    
                    R_leg_IMU_angle = coded_data[2] + coded_data[3]*256
                    R_leg_IMU_angle = R_leg_IMU_angle - 65536 if R_leg_IMU_angle > 32767 else R_leg_IMU_angle
                    R_leg_IMU_angle = R_leg_IMU_angle / 100.0

                    L_motor_torque = coded_data[4] + coded_data[5]*256
                    L_motor_torque = L_motor_torque - 65536 if L_motor_torque > 32767 else L_motor_torque
                    L_motor_torque = L_motor_torque / 100.0

                    R_motor_torque = coded_data[6] + coded_data[7]*256
                    R_motor_torque = R_motor_torque - 65536 if R_motor_torque > 32767 else R_motor_torque
                    R_motor_torque = R_motor_torque / 100.0

                    L_motor_torque_desired = coded_data[8] + coded_data[9]*256
                    L_motor_torque_desired = L_motor_torque_desired - 65536 if L_motor_torque_desired > 32767 else L_motor_torque_desired
                    L_motor_torque_desired = L_motor_torque_desired / 100.0

                    R_motor_torque_desired = coded_data[10] + coded_data[11]*256
                    R_motor_torque_desired = R_motor_torque_desired - 65536 if R_motor_torque_desired > 32767 else R_motor_torque_desired
                    R_motor_torque_desired = R_motor_torque_desired / 100.0

                    torque_L = coded_data[12] + coded_data[13]*256
                    torque_L = torque_L - 65536 if torque_L > 32767 else torque_L
                    torque_L = torque_L / 100.0

                    torque_R = coded_data[14] + coded_data[15]*256
                    torque_R = torque_R - 65536 if torque_R > 32767 else torque_R
                    torque_R = torque_R / 100.0

                    trigger_on = coded_data[16]
                    trigger_val = coded_data[17]

                    imu_gait_phase_L = coded_data[18] + coded_data[19]*256
                    imu_gait_phase_L = imu_gait_phase_L - 65536 if imu_gait_phase_L > 32767 else imu_gait_phase_L
                    imu_gait_phase_L = imu_gait_phase_L / 100.0

                    imu_gait_phase_R = coded_data[20] + coded_data[21]*256
                    imu_gait_phase_R = imu_gait_phase_R - 65536 if imu_gait_phase_R > 32767 else imu_gait_phase_R
                    imu_gait_phase_R = imu_gait_phase_R / 100.0

                    l_motor_speed = coded_data[22] + coded_data[23]*256
                    l_motor_speed = l_motor_speed - 65536 if l_motor_speed > 32767 else l_motor_speed
                    l_motor_speed = l_motor_speed / 100.0

                    r_motor_speed = coded_data[24] + coded_data[25]*256
                    r_motor_speed = r_motor_speed - 65536 if r_motor_speed > 32767 else r_motor_speed
                    r_motor_speed = r_motor_speed / 100.0'''

                    print(L_leg_IMU_angle, R_leg_IMU_angle, L_motor_torque, R_motor_torque)
    else:
            print('no data recieved')