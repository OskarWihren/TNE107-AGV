# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

#
"""
import time
import multiprocessing
import os

import board
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
from adafruit_lsm6ds import Rate as Rate_acc

from adafruit_lis3mdl import LIS3MDL
from adafruit_lis3mdl import Rate as Rate_mag


i2c = board.I2C()  # uses board.SCL and board.SDA
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)
accel_gyro.accelerometer_data_rate = Rate_acc.RATE_833_HZ
mag.data_rate = Rate_mag.RATE_80_HZ #Sätter även PerformanceMode till MODE_ULTRA

result = [
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0]
]



def get_imu():
    acc_x, acc_y, acc_z = accel_gyro.acceleration
    gyro_x, gyro_y, gyro_z = accel_gyro.gyro
    mag_x, mag_y, mag_z = mag.magnetic
    
    print("")
    print("Acc: ", acc_x, acc_y, acc_z)
    print("Gyro: ", gyro_x, gyro_y, gyro_z)
    print("Mag: ", mag_x, mag_y, mag_z)

    
    
    result_acc = [acc_x, acc_y, acc_z]
    result_gyro = [gyro_x, gyro_y, gyro_z]
    result_mag = [mag_x, mag_y, mag_z]
    result = [result_acc, result_gyro, result_mag]
    return result


    # print(
    #     "Acceleration: X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} m/s^2".format(*acceleration)
    # )
    # print("Gyro          X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} rad/s".format(*gyro))
    # print("Magnetic      X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} uT".format(*magnetic))
    # print("")
    
    
    
if __name__ == "__main__":
    try:
        while True:
            start = time.time()
            result = get_imu()
            
            end = time.time()
            print(result)
            print(end-start)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("bye")




"""















#NEDAN ÄR KOD MED MULTIPROCESSING, DEN ÄR INTE HELT KLAR.
#MÅSTE SKAPA EN LOOP INTERNT I FUNKTIONEN SOM LÄSER DATAN HELA TIDEN, OCH SEN LÄSER VI AV QUEUE MED ENBART SENASTE VÄRDET PÅ NÅGOT SÄTT


# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import multiprocessing
import os
# from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS

# To use LSM6DS33, comment out the LSM6DSOX import line
# and uncomment the next line
# from adafruit_lsm6ds.lsm6ds33 import LSM6DS33 as LSM6DS

# To use ISM330DHCX, comment out the LSM6DSOX import line
# and uncomment the next line
# from adafruit_lsm6ds.lsm330dhcx import ISM330DHCX as LSM6DS

# To use LSM6DS3TR-C, comment out the LSM6DSOX import line
# and uncomment the next line




def imu_setup():
    
    import board
    from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
    from adafruit_lsm6ds import Rate as Rate_acc

    from adafruit_lis3mdl import LIS3MDL
    from adafruit_lis3mdl import Rate as Rate_mag
    i2c = board.I2C()  # uses board.SCL and board.SDA
    # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
    accel_gyro = LSM6DS(i2c)
    mag = LIS3MDL(i2c)

    accel_gyro.accelerometer_data_rate = Rate_acc.RATE_833_HZ
    mag.data_rate = Rate_mag.RATE_80_HZ #Sätter även PerformanceMode till MODE_ULTRA
    #lägg till datarate för mag


def get_imu(queue):
    import board
    from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
    from adafruit_lsm6ds import Rate as Rate_acc

    from adafruit_lis3mdl import LIS3MDL
    from adafruit_lis3mdl import Rate as Rate_mag

    os.sched_setaffinity(0, {2})

    i2c = board.I2C()  # uses board.SCL and board.SDA
    # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
    accel_gyro = LSM6DS(i2c)
    mag = LIS3MDL(i2c)

    while True: 
        acc_x, acc_y, acc_z = accel_gyro.acceleration
        gyro_x, gyro_y, gyro_z = accel_gyro.gyro
        mag_x, mag_y, mag_z = mag.magnetic
        
        # print("")
        # print("Acc: ", acc_x, acc_y, acc_z)
        # print("Gyro: ", gyro_x, gyro_y, gyro_z)
        # print("Mag: ", mag_x, mag_y, mag_z)
        
        
        
        result_acc = [acc_x, acc_y, acc_z]
        result_gyro = [gyro_x, gyro_y, gyro_z]
        result_mag = [mag_x, mag_y, mag_z]
        result = [result_acc, result_gyro, result_mag]
        
        
        
        #Sen lägger vi in den nya datan i queue

        queue.put(result)
        print("Queue size:", q_imu.qsize())
        
        

    
    # print(
    #     "Acceleration: X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} m/s^2".format(*acceleration)
    # )
    # print("Gyro          X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} rad/s".format(*gyro))
    # print("Magnetic      X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} uT".format(*magnetic))
    # print("")
    
    
    
if __name__ == "__main__":
    i=0
    
    imu_setup()
    time.sleep(1)
    q_imu = multiprocessing.Queue(maxsize=1)
    p_imu = multiprocessing.Process(target=get_imu, args=(q_imu,))
    
    p_imu.start()
    time.sleep(1)
    try:
        start = 0
        end = 0
        time_begin_main = time.time()
        while True:
            
            if not q_imu.empty():
                start = time.time()
                data = q_imu.get()
                
                
                acceleration = data[0]
                gyro = data[1]
                magnetic = data[2]
                
                print(
                "Acceleration: X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} m/s^2".format(*acceleration)
                )
                print("Gyro          X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} rad/s".format(*gyro))
                print("Magnetic      X:{0:7.2f}, Y:{1:7.2f}, Z:{2:7.2f} uT".format(*magnetic))
                print("")
                

                
                # print("IMU Data:", data)
            time.sleep(0.1)
            i = i+1
            print(i)
            print((time.time()-time_begin_main)/i)
            end = time.time()
            print(end-start)
    except KeyboardInterrupt:
        p_imu.terminate()
        p_imu.join()
        print("   \nIMU Shut downn")

