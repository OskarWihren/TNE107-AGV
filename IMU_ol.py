
import time
import multiprocessing
import os
import numpy
import serial

import numpy as np



#from mag_cal.calibration import Calibration
#from mag_cal.utils import read_fixture

import sys
import os
import math

'''
  If you want to use I2C to drive this module, uncomment the codes below, and connect the module with Raspberry Pi via I2C port
  Connect to VCC，GND，SCL，SDA pin
'''
I2C_BUS         = 0x01   #default use I2C1
bmm350 = DFRobot_bmm350_I2C(I2C_BUS, 0x14)

def setup():
  while BMM350_CHIP_ID_ERROR == bmm350.sensor_init():
    print("sensor init error, please check connect") 
    time.sleep(1)

  '''
    Set sensor operation mode
      opMode:
        BMM350_SUSPEND_MODE      suspend mode: Suspend mode is the default power mode of BMM350 after the chip is powered, Current consumption in suspend mode is minimal, 
                                 so, this mode is useful for periods when data conversion is not needed. Read and write of all registers is possible.
        BMM350_NORMAL_MODE       normal mode: Get geomagnetic data normally.      
        BMM350_FORCED_MODE       forced mode: Single measurement, the sensor restores to suspend mode when the measurement is done.
        BMM350_FORCED_MODE_FAST  To reach ODR = 200Hz is only possible by using FM_ FAST.
  '''
  bmm350.set_operation_mode(BMM350_NORMAL_MODE)

  '''
    Set preset mode, make it easier for users to configure sensor to get geomagnetic data (The default rate for obtaining geomagnetic data is 12.5Hz)
      presetMode:
        BMM350_PRESETMODE_LOWPOWER       Low power mode, get a small number of data and draw the mean value.
        BMM350_PRESETMODE_REGULAR        Regular mode, get a number of data and draw the mean value.
        BMM350_PRESETMODE_ENHANCED       Enhanced mode, get a large number of data and draw the mean value.
        BMM350_PRESETMODE_HIGHACCURACY   High accuracy mode, get a huge number of data and draw the mean value.
  '''
  bmm350.set_preset_mode(BMM350_PRESETMODE_HIGHACCURACY)

  '''
    Set the rate of obtaining geomagnetic data, the higher, the faster (without delay function)
      rate:
        BMM350_DATA_RATE_1_5625HZ
        BMM350_DATA_RATE_3_125HZ
        BMM350_DATA_RATE_6_25HZ
        BMM350_DATA_RATE_12_5HZ  (default rate)
        BMM350_DATA_RATE_25HZ
        BMM350_DATA_RATE_50HZ
        BMM350_DATA_RATE_100HZ
        BMM350_DATA_RATE_200HZ
        BMM350_DATA_RATE_400HZ
  '''
  bmm350.set_rate(BMM350_DATA_RATE_12_5HZ)
  
  
  '''
    Enable the measurement at x-axis, y-axis and z-axis, default to be enabled, no config required. When disabled, the geomagnetic data at x, y, and z will be inaccurate.
    Refer to readme file if you want to configure more parameters.
  '''
  bmm350.set_measurement_XYZ()



def compute_alpha(gyro_z):
    ALPHA_MIN = 0.95   # When stationary
    ALPHA_MAX = 0.99  # When rotating fast
    GYRO_THRESHOLD_LOW = 2.0    # deg/s
    GYRO_THRESHOLD_HIGH = 50.0  # deg/s
    abs_gyro = abs(gyro_z)

    if abs_gyro <= GYRO_THRESHOLD_LOW:
        return ALPHA_MIN
    elif abs_gyro >= GYRO_THRESHOLD_HIGH:
        return ALPHA_MAX
    else:
        # Interpolate between min and max
        scale = (abs_gyro - GYRO_THRESHOLD_LOW) / (GYRO_THRESHOLD_HIGH - GYRO_THRESHOLD_LOW)
        return ALPHA_MIN + scale * (ALPHA_MAX - ALPHA_MIN)

def data_retriever(boards):
    accel_gyro = boards[0]
    mag = boards[1]

    acc_x, acc_y, acc_z = accel_gyro.acceleration
    gyro_x, gyro_y, gyro_z = accel_gyro.gyro
    mag_x, mag_y, mag_z = mag.magnetic
    
    result_acc = [acc_x, acc_y, acc_z]
    result_gyro = [gyro_x, gyro_y, gyro_z]
    result_mag = [mag_x, mag_y, mag_z]
    result = [result_acc, result_gyro, result_mag]
    return result

def fit_2d_ellipse(data):
    x = data[:, 0]
    y = data[:, 1]
    D = np.column_stack([x**2, x*y, y**2, x, y, np.ones_like(x)])
    _, _, V = np.linalg.svd(D)
    coef = V[-1, :]
    return coef


def ellipse_center(coef):
    A, B, C, D, E, _ = coef
    den = B**2 - 4*A*C
    x0 = (2*C*D - B*E) / den
    y0 = (2*A*E - B*D) / den
    return np.array([x0, y0])



from numpy.linalg import eig, inv

def ellipse_transformation_matrix(coef):
    A, B, C, _, _, _ = coef
    Q = np.array([[A, B/2],
                  [B/2, C]])

    # Diagonalize the matrix
    evals, evecs = eig(Q)

    if np.any(evals <= 0):
        raise ValueError("Ellipse has non-positive eigenvalues, invalid fit.")

    # Inverse sqrt of eigenvalues
    D_inv_sqrt = np.diag(1 / np.sqrt(evals))
    transform = evecs @ D_inv_sqrt @ evecs.T

    return transform


def correct_mag(raw, offset, transform):
    return (raw - offset) @ transform


def calibrate_gyro(samples, delay, boards):
    print("Kalibrerar gyro...")
    sumX, sumY, sumZ = 0, 0, 0
    for _ in range(samples):
        data = data_retriever(boards)
        gyro_x, gyro_y, gyro_z = data[1]
        sumX += gyro_x
        sumY += gyro_y
        sumZ += gyro_z
        time.sleep(delay)
        
    biasX = sumX / samples
    biasY = sumY / samples
    biasZ = sumZ / samples
    print("gyroscope bias: ", biasX, biasY, biasZ)
    return [biasX, biasY, biasZ]

def calibrate_acc(samples, delay, boards):
    print("Kalibrerar acc...")
    acc_sumX, acc_sumY, acc_sumZ = 0, 0, 0
    for _ in range(samples):
        data = data_retriever(boards)
        acc_x, acc_y, acc_z = data[0]
        acc_sumX += acc_x
        acc_sumY += acc_y
        acc_sumZ += acc_z
        time.sleep(delay)

    acc_biasX = acc_sumX / samples
    acc_biasY = acc_sumY / samples
    acc_biasZ = acc_sumZ / samples
    ser.write("-100,100\n".encode('UTF-8'))
    time.sleep(1)

    print("Accelerometer bias: ", acc_biasX, acc_biasY, acc_biasZ)
    return [acc_biasX, acc_biasY, acc_biasZ]

def calibrate_mag(samples, delay, boards):
    print("Kalibrerar magnetometer. Snurra runt IMU/AGV några varv") #Här kanske vi kör något att den själv ska snurra?
    mag_data = [[0]*samples, [0]*samples, [0]*samples]
    ser.write("-100,100\n".encode('UTF-8'))
    time.sleep(0.5)

    acc_x = [0] * samples #Skapa en vektor med nollor för accelerometer
    acc_y = [0] * samples
    acc_z = [0] * samples


    max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')
    min_x, min_y, min_z = float('inf'), float('inf'), float('inf')

    for j in range(samples): #Läs ut data under x antal samples och spara resultat i en vektor 
        #data = data_retriever(boards)
        #mag_x[j], mag_y[j], mag_z[j] = data[2]
        #acc_x[j], acc_y[j], acc_z[j] = data[0]
        
        mag_data[0][j], mag_data[1][j], mag_data[2][j] = bmm350.get_geomagnetic_data()
        
        #print("Läst mag data under kal: ", mag_x[j], mag_y[j])
        degree = bmm350.get_compass_degree()
        print("Läst mag data under kal: ", mag_data[0][j], mag_data[1][j])
        print("Läst vinkel under kal: ", degree)
        time.sleep(delay)

    #Omvandla datan till korrekt format, för funktionen
    
    #raw_data = np.column_stack((mag_x, mag_y))
    raw_data = np.column_stack((mag_data[0], mag_data[1]))

    coef = fit_2d_ellipse(raw_data)

    # Step 2: Get hard iron offset
    offset = ellipse_center(coef)

    # Step 3: Get transformation matrix
    transform = ellipse_transformation_matrix(coef)
    
    print("Magnetometer calibration finished...")
    print("Offset: ", offset, "Transform: ", transform)

    ser.write("0,0\n".encode('UTF-8'))
    return [offset, transform]


def get_imu(queue):


    ######SETUP SEQUENCE START
    import board
    from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
    from adafruit_lsm6ds import Rate as Rate_acc

    from adafruit_lis3mdl import LIS3MDL
    from adafruit_lis3mdl import Rate as Rate_mag
    
    os.sched_setaffinity(0, {2})

    #i2c = board.I2C() 
    #accel_gyro = LSM6DS(i2c)
    #mag = LIS3MDL(i2c)
    accel_gyro = 0
    mag = 0
    
    ###SETUP BMM350
    I2C_BUS         = 0x01   #default use I2C1
    bmm350 = DFRobot_bmm350_I2C(I2C_BUS, 0x14)


    while BMM350_CHIP_ID_ERROR == bmm350.sensor_init():
        print("sensor init error, please check connect") 
        time.sleep(1)

 
    bmm350.set_operation_mode(BMM350_NORMAL_MODE)

    bmm350.set_preset_mode(BMM350_PRESETMODE_HIGHACCURACY)

    '''
    Set the rate of obtaining geomagnetic data, the higher, the faster (without delay function)
        rate:
        BMM350_DATA_RATE_1_5625HZ
        BMM350_DATA_RATE_3_125HZ
        BMM350_DATA_RATE_6_25HZ
        BMM350_DATA_RATE_12_5HZ  (default rate)
        BMM350_DATA_RATE_25HZ
        BMM350_DATA_RATE_50HZ
        BMM350_DATA_RATE_100HZ
        BMM350_DATA_RATE_200HZ
        BMM350_DATA_RATE_400HZ
    '''
    bmm350.set_rate(BMM350_DATA_RATE_12_5HZ)
  
  

    bmm350.set_measurement_XYZ()



    #####SLUT PÅ SETUP FÖR BMM350

    boards = [accel_gyro, mag, bmm350] #Sparar adresserna för att skickas med i funktionerna separat
    

    #Ställer in datarate för de olika sensorerna
    
    # accel_gyro.accelerometer_data_rate = Rate_acc.RATE_833_HZ
    # mag.data_rate = Rate_mag.RATE_80_HZ #Sätter även PerformanceMode till MODE_ULTRA
    # time.sleep(3)

    ######SETUP SEQUENCE END
    

    #START CALIBRATION
    samples = 50
    delay = 0.05
    #bias_gyro = calibrate_gyro(samples, delay, boards)
    #print(bias_gyro)
    #bias_acc = calibrate_acc(samples, delay, boards)
    #print(bias_acc)
    offset, transform = calibrate_mag(samples*10, delay, boards)
    
    time.sleep(3)

    while True: 
        '''
        acc_x, acc_y, acc_z = accel_gyro.acceleration
        gyro_x, gyro_y, gyro_z = accel_gyro.gyro
        mag_x, mag_y, mag_z = mag.magnetic
        
        result_acc = [acc_x, acc_y, acc_z]
        result_gyro = [gyro_x, gyro_y, gyro_z]
        result_mag = [mag_x, mag_y, mag_z]
        result = [result_acc, result_gyro, result_mag]
        
        raw_reading = np.array([mag_x, mag_y])
        corrected = correct_mag(raw_reading, offset, transform)
        angle_rad = np.arctan2(-corrected[1], corrected[0])  # atan2(y, x)
        angle_deg = np.degrees(angle_rad)
        print("Heading (degrees):", angle_deg)
        '''
        mag_data_x, mag_data_y, mag_data_z = bmm350.get_geomagnetic_data()

        degree = bmm350.get_compass_degree()

        raw_reading = np.array([mag_data_x, mag_data_y])

        print("Heading (degrees):", degree)

        corrected = correct_mag(raw_reading,offset,transform)
        angle_rad = np.arctan2(-corrected[1], corrected[0])  # atan2(y, x)
        angle_deg = np.degrees(angle_rad)
        print("Heading (degrees):", angle_deg)
        

        time.sleep(0.2)
        #Sen lägger vi in den nya datan i queue
        #queue.put(result)
        #print("Queue size:", q_imu.qsize())
        


if __name__ == "__main__":
    i=0
    q_imu = multiprocessing.Queue(maxsize=1)
    p_imu = multiprocessing.Process(target=get_imu, args=(q_imu,))
    p_imu.start()
    time.sleep(2)

   
