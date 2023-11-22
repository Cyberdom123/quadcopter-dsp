from inputs import get_gamepad
import serial
import struct
import time
import numpy as np

from filters import ema_filter

baudRate = 115200
ser = serial.Serial('/dev/ttyUSB0', baudRate)


def get_imu_data():
    time.sleep(0.0001)

    ser.write(bytearray([0, 0, 0, 0, 0, 0]))
    
    while True:
        if(ser.in_waiting > 0):
            msg = ser.read(24)
            try:
                [acc0, acc1, acc2, gyro0, gyro1, gyro2] = struct.unpack('6f', msg)
                #print(f"{acc0} {acc1} {acc2} {gyro0} {gyro1} {gyro2}")
                return [acc0, acc1, acc2, gyro0, gyro1, gyro2]
            except:
                return [0,0,0,0,0,0]
            break
""" void Get_Roll_Pitch(float acc_buf[3], float angles[2]){
  angles[0] = atan2(acc_buf[1], acc_buf[2]);
  angles[1] = asin(acc_buf[0]/g);
} """
axf = 0; ayf = 0; azf = 0
def get_pitch_roll(ax, ay, az):
    global axf, ayf, azf

    axOffset = 0.064
    ax = ax - axOffset
    
    axf = ema_filter(ax, 0.3, axf)
    ayf = ema_filter(ay, 0.3, ayf)
    azf = ema_filter(az, 0.3, azf)

    # axf = ax
    # ayf = ay
    # azf = az
    
    if(azf == 0):
        roll = 1
    else: 
        roll = round(np.rad2deg(np.arctan(ayf/azf)), 2)

    if(axf > 1):
        axf = 1
    if(axf < -1):
        axf = -1

    pitch = round(np.rad2deg(np.arcsin(axf)), 2)

    return pitch, roll

if __name__ == '__main__':
    ser.timeout = 0.005
    while True:
        start = time.time()
        [acc0, acc1, acc2, gyro0, gyro1, gyro2] = get_imu_data()
        stop = time.time()

        [pitch, roll] = get_pitch_roll(acc0, acc1, acc2)
        print(f"pitch = {pitch}, roll = {roll}")