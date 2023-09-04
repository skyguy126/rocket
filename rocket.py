# LSM303DLHC accel/mag
# L3GD20H gyro

import RPi.GPIO as GPIO
import numpy as np
import smbus, time, sys

ACC_ADDR = 0x19
MAG_ADDR = 0x1e
GYR_ADDR = 0x6b

CTRL_REG1_A = 0x20
STATUS_REG_A = 0x27 
OUT_X_L_A = 0x28
OUT_X_H_A = 0x29
OUT_Y_L_A = 0x2a
OUT_Y_H_A = 0x2b
OUT_Z_L_A = 0x2c
OUT_Z_H_A = 0x2d

WHO_REG_G = 0x0f
CTRL_REG1_G = 0x20
CTRL_REG4_G = 0x23 
STATUS_REG_G = 0x27
OUT_X_L_G = 0x28
OUT_X_H_G = 0x29
OUT_Y_L_G = 0x2a
OUT_Y_H_G = 0x2b
OUT_Z_L_G = 0x2c
OUT_Z_H_G = 0x2d

LED_PIN = 18

# this is where we store [time, ax, ay, az, gx, gy, gz]
timeseries = []

# TODO: create a raw array for time series?

def led_on():
    GPIO.output(LED_PIN, GPIO.HIGH)

def led_off():
    GPIO.output(LED_PIN, GPIO.LOW)

# https://stackoverflow.com/questions/1604464/twos-complement-in-python
def twos_complement(val, num_bits):
    if (val & (1 << (num_bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << num_bits)        # compute negative value
    return val 

def build_integer(lo, hi):
    # adafruit bitshifts >> 4 no clue why, but I omitted.
    return twos_complement(((lo & 0xff) | ((hi & 0xff) << 8)), 16)

def fetch_acc_measurement(bus):

    # fetch status of acc measurement registers
    status = bus.read_byte_data(ACC_ADDR, STATUS_REG_A)

    if not (status & 0x8):
        # no new acc data available, try again later 
        print("no acc new data")
        return np.ones(3) / np.linalg.norm(np.ones(3))

    # read data (0x80 for multiple byte read)
    data = bus.read_i2c_block_data(ACC_ADDR, OUT_X_L_A | 0x80, 6)

    v = np.array([build_integer(data[0], data[1]), 
                  build_integer(data[2], data[3]), 
                  build_integer(data[4], data[5])])

    # normalize accelerometer data (unit vec)
    v_hat = v / np.linalg.norm(v)

    return v_hat

def fetch_gyr_measurement(bus):
    
    # fetch status of gyr measurement registers
    status = bus.read_byte_data(GYR_ADDR, STATUS_REG_G)

    if not (status & 0x8):
        # no new gyr data available, try again later 
        print("no gyr new data")
        return np.ones(3) / np.linalg.norm(np.ones(3))
    
    # read data (0x80 for multiple byte read)
    data = bus.read_i2c_block_data(GYR_ADDR, OUT_X_L_G | 0x80, 6)

    v = np.array([build_integer(data[0], data[1]), 
                  build_integer(data[2], data[3]), 
                  build_integer(data[4], data[5])], dtype=np.float64)

    # apply gyro sensitivity range for 2000 DPS
    v *= 0.070

    return v

if __name__ == "__main__":

    # init status LED gpio
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_PIN, GPIO.OUT)

    # fetch the main i2c bus
    bus = smbus.SMBus(1)

    # intialize accelerometer to 400 HZ
    bus.write_byte_data(ACC_ADDR, CTRL_REG1_A, 0x77)

    readback = bus.read_byte_data(ACC_ADDR, CTRL_REG1_A)
    if readback == 0x77:
        print("acc setup ok")
    else:
        print("acc setup failed")
        sys.exit(-1)

    # reset gyro
    bus.write_byte_data(GYR_ADDR, CTRL_REG1_G, 0x00)

    # init gyro and enable all three axis
    bus.write_byte_data(GYR_ADDR, CTRL_REG1_G, 0x0F)

    # set gyro resolution (250 degrees per second)
    bus.write_byte_data(GYR_ADDR, CTRL_REG4_G, 0x00)

    # readback who register to make sure we're still working
    readback = bus.read_byte_data(GYR_ADDR, WHO_REG_G)
    if readback == 0b11010111:
        print("gyr setup ok")
    else:
        print("gyr setup failed")
        sys.exit(-1)
    
    # give the IMU some time to settle before pulling data
    time.sleep(5)
    print("starting...")

    start_time = time.time()

    while True:

        # loop for 5 minutes then exit
        if time.time() - start_time > 300:
            break
        
        acc_measurement = fetch_acc_measurement(bus)
        # print(acc_measurement)

        gyr_measurement = fetch_gyr_measurement(bus)
        # print(debug, gyr_measurement)

        # add measured data to our timeseries
        timeseries.append(np.concatenate([[time.time()], acc_measurement, gyr_measurement], axis=0))

        time.sleep(0.02)
    
    print("saving timeseries...")
    np.save('imu.npy', np.asarray(timeseries))
