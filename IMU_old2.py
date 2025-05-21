
import time
import multiprocessing
import os
import numpy
import serial
import matplotlib.pyplot as plt

import numpy as np


ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1)


PATH = "../tests/fixtures/cal_data/hj2.json"



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
    print("Accelerometer bias: ", acc_biasX, acc_biasY, acc_biasZ)
    
    return [acc_biasX, acc_biasY, acc_biasZ]

def calibrate_mag(samples, delay, boards):
    print("Kalibrerar magnetometer. Snurra runt IMU/AGV några varv") #Här kanske vi kör något att den själv ska snurra?
    ser.write("-130,130\n".encode('UTF-8'))
    time.sleep(1)


    mag_x = [0] * samples #Skapa en vektor med nollor för 
    mag_y = [0] * samples
    mag_z = [0] * samples

    acc_x = [0] * samples #Skapa en vektor med nollor för accelerometer
    acc_y = [0] * samples
    acc_z = [0] * samples


    max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')
    min_x, min_y, min_z = float('inf'), float('inf'), float('inf')

    for j in range(samples): #Läs ut data under x antal samples och spara resultat i en vektor 
        data = data_retriever(boards)
        mag_x[j], mag_y[j], mag_z[j] = data[2]
        acc_x[j], acc_y[j], acc_z[j] = data[0]
        #print("Läst mag data under kal: ", mag_x[j], mag_y[j])
        time.sleep(delay)

    plt.figure()
    plt.plot(mag_x, mag_y, 'o-')
    plt.title("Magnetometer Data (X vs Y)")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis('equal')
    plt.savefig("magnetometer_plot.png")  # Save to file
    plt.close()

    #Omvandla datan till korrekt format, för funktionen
    raw_data = np.column_stack((mag_x, mag_y))

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

    i2c = board.I2C() 
    accel_gyro = LSM6DS(i2c)
    mag = LIS3MDL(i2c)

    boards = [accel_gyro, mag] #Sparar adresserna för att skickas med i funktionerna separat
    

    #Ställer in datarate för de olika sensorerna
    accel_gyro.accelerometer_data_rate = Rate_acc.RATE_833_HZ
    mag.data_rate = Rate_mag.RATE_80_HZ #Sätter även PerformanceMode till MODE_ULTRA
    time.sleep(3)
    ######SETUP SEQUENCE END
    

    #START CALIBRATION
    samples = 50
    delay = 0.1
    bias_gyro = calibrate_gyro(samples, delay, boards)
    print(bias_gyro)
    bias_acc = calibrate_acc(samples, delay, boards)
    print(bias_acc)
    offset, transform = calibrate_mag(samples*4, delay, boards)
    
    time.sleep(1)

    samples = samples*4
    while True: 
        

        
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

        time.sleep(1)


        #TA BORT DETTA
        raw_reading = np.array([mag_x, mag_y])
        corrected = correct_mag(raw_reading, offset, transform)
        angle_rad = np.arctan2(-corrected[1], corrected[0])  # atan2(y, x)
        angle_deg = np.degrees(angle_rad)
        print("Heading (degrees):", angle_deg)
        
        mag_x_new = [0] * samples #Skapa en vektor med nollor för 
        mag_y_new = [0] * samples
        mag_z_new = [0] * samples

        acc_x_new = [0] * samples #Skapa en vektor med nollor för accelerometer
        acc_y_new = [0] * samples
        acc_z_new = [0] * samples

        corrected_x = [0] * samples #Skapa en vektor med nollor för 
        corrected_y = [0] * samples
        corrected_z = [0] * samples
        ser.write("-130,130\n".encode('UTF-8'))
        for j in range(samples): #Läs ut data under x antal samples och spara resultat i en vektor 
            data = data_retriever(boards)
            mag_x_new[j], mag_y_new[j], mag_z_new[j] = data[2]
            acc_x_new[j], acc_y_new[j], acc_z_new[j] = data[0]
            raw_reading = np.array([mag_x_new[j], mag_y_new[j]])
            corrected_x[j],corrected_y[j] = correct_mag(raw_reading, offset, transform)

            time.sleep(0.1)
        plt.figure()
        plt.plot(corrected_x, corrected_y, 'o-')
        plt.title("Magnetometer Data (X vs Y)")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.axis('equal')
        plt.savefig("magnetometer_plot_after.png")  # Save to file
        plt.close()

        


        time.sleep(0.1)
        #Sen lägger vi in den nya datan i queue
        #queue.put(result)
        #print("Queue size:", q_imu.qsize())
        


if __name__ == "__main__":
    i=0
    q_imu = multiprocessing.Queue(maxsize=1)
    p_imu = multiprocessing.Process(target=get_imu, args=(q_imu,))
    p_imu.start()
    time.sleep(2)

   
