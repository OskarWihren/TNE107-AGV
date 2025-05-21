
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

def calibrate_mag_simple(mag_x, mag_y):
    raw_data = np.column_stack((mag_x, mag_y))

    # 1. Hard iron offset
    offset = np.mean(raw_data, axis=0)

    # 2. Soft iron correction (whitening transform)
    centered = raw_data - offset
    cov = np.cov(centered.T)
    evals, evecs = np.linalg.eigh(cov)

    if np.any(evals <= 0):
        raise ValueError("Non-positive eigenvalues in covariance matrix.")

    # Inverse sqrt of covariance
    D_inv_sqrt = np.diag(1.0 / np.sqrt(evals))
    transform = evecs @ D_inv_sqrt @ evecs.T

    return offset, transform

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

def calibrate_mag(samples, delay, boards, bias_gyro):
    print("Kalibrerar magnetometer. Snurrar runt IMU/AGV tre varv") #Här kanske vi kör något att den själv ska snurra?
    ser.write("-75,75\n".encode('UTF-8'))
    

    mag_x = [0] * samples #Skapa en vektor med nollor för 
    mag_y = [0] * samples
    mag_z = [0] * samples

    gyro_x = 0
    gyro_y = 0
    gyro_z = 0
    gyro_angle = 0

    
    last_time = time.monotonic()
    calibration_rotations = 2
    rotation_finished = False
    samples_acquired = 0
    final_angle_cal = 0
    for j in range(samples): #Läs ut data under x antal samples och spara resultat i en vektor 
        data = data_retriever(boards)
        gyro_x, gyro_y, gyro_z = data[1]
        gyro_z = gyro_z-bias_gyro[2]

        dt = last_time - time.monotonic()
        print(dt)
        gyro_angle += (gyro_z * dt)*180/3.14
        print("Gyro angle: ", gyro_angle)
        
        mag_x[j], mag_y[j], mag_z[j] = data[2]
        if gyro_angle >= calibration_rotations*360:
            if rotation_finished == False:
                rotation_finished = True
                samples_acquired = j
                final_angle_cal = gyro_angle
                print("Rotation finished after sample", j)
                ser.write("0,0\n".encode('UTF-8'))
                break
        #print("Läst mag data under kal: ", mag_x[j], mag_y[j])
        
        last_time = time.monotonic()
        time.sleep(delay)

    mag_x = mag_x[:samples_acquired] #Rensa alla mätvärden som kommer efter rotationen egentligen har slutförts
    mag_y = mag_y[:samples_acquired]
    angle_offset = final_angle_cal - calibration_rotations*360 #Kompensera för att vi har roterat för långt

    plt.figure()
    plt.plot(mag_x, mag_y, 'o-')
    plt.title("Magnetometer Data (X vs Y)")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axis('equal')
    plt.savefig("magnetometer_plot.png")  # Save to file
    plt.close()
    
    offset,transform = calibrate_mag_simple(mag_x,mag_y)
    #Omvandla datan till korrekt format, för funktionen
    
    print("Magnetometer calibration finished...")
    print("Offset: ", offset, "Transform: ", transform)
    ser.write("0,0\n".encode('UTF-8'))
    
    
    return [offset, transform, angle_offset]


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
    time.sleep(1)
    ######SETUP SEQUENCE END
    

    #START CALIBRATION
    ser.write("0,0\n".encode('UTF-8'))
    samples = 50
    delay = 0.05
    bias_gyro = calibrate_gyro(samples, delay, boards)
    print(bias_gyro)
    bias_acc = calibrate_acc(samples, delay, boards)
    print(bias_acc)
    offset, transform, angle_offset = calibrate_mag(samples*16, delay, boards, bias_gyro)
    mag_x_sum, mag_y_sum = 0, 0
    for j in range(samples): #Kolla vinkeln vid slutet av kalibrering med kompass
        mag_x, mag_y, mag_z = mag.magnetic
        mag_x_sum += mag_x
        mag_y_sum += mag_y
        time.sleep(delay)
    
    avg_mag_x = mag_x_sum/(samples)
    avg_mag_y = mag_y_sum/(samples)

    raw_reading = np.array([avg_mag_x, avg_mag_y])
    corrected = correct_mag(raw_reading, offset, transform)
    angle_rad = np.arctan2(-corrected[1], corrected[0])  # atan2(y, x)
    angle_average = np.degrees(angle_rad) + 180 #Konvertera till grader
    print("Average angle during standstill: ", angle_average)

    while True: 
        
        acc_x, acc_y, acc_z = accel_gyro.acceleration
        gyro_x, gyro_y, gyro_z = accel_gyro.gyro
        mag_x, mag_y, mag_z = mag.magnetic
       
        result_acc = [acc_x, acc_y, acc_z]
        result_gyro = [gyro_x-bias_gyro[0], gyro_y-bias_gyro[1], gyro_z-bias_gyro[2]]
        result_mag = [mag_x, mag_y, mag_z]
        result = [result_acc, result_gyro, result_mag]
        
        raw_reading = np.array([mag_x, mag_y])
        corrected = correct_mag(raw_reading, offset, transform)
        
        angle_rad = np.arctan2(-corrected[1], corrected[0])  # atan2(y, x)
        angle_deg = np.degrees(angle_rad) + 180
        
        angle_deg = angle_deg - angle_average + angle_offset
        
        
        #Sen lägger vi in den nya datan i queue
        
        if not queue.empty():
            queue.get_nowait() #Rensa kön

        queue.put(angle_deg) #Sätt in i kön
        
        time.sleep(0.1)

        

        #TA BORT DETTA
        # raw_reading = np.array([mag_x, mag_y])
        # corrected = correct_mag(raw_reading, offset, transform)
        # angle_rad = np.arctan2(-corrected[1], corrected[0])  # atan2(y, x)
        # angle_deg = np.degrees(angle_rad)
        # print("Heading (degrees):", angle_deg)  
        
        # mag_x_new = [0] * samples #Skapa en vektor med nollor för 
        # mag_y_new = [0] * samples
        # mag_z_new = [0] * samples

        # acc_x_new = [0] * samples #Skapa en vektor med nollor för accelerometer
        # acc_y_new = [0] * samples
        # acc_z_new = [0] * samples

        # corrected_x = [0] * samples #Skapa en vektor med nollor för 
        # corrected_y = [0] * samples
        # corrected_z = [0] * samples
        # ser.write("-130,130\n".encode('UTF-8'))
        # for j in range(samples): #Läs ut data under x antal samples och spara resultat i en vektor 
        #     data = data_retriever(boards)
        #     mag_x_new[j], mag_y_new[j], mag_z_new[j] = data[2]
        #     acc_x_new[j], acc_y_new[j], acc_z_new[j] = data[0]
        #     raw_reading = np.array([mag_x_new[j], mag_y_new[j]])
        #     corrected_x[j],corrected_y[j] = correct_mag(raw_reading, offset, transform)

        #     time.sleep(0.1)
        # plt.figure()
        # plt.plot(corrected_x, corrected_y, 'o-')
        # plt.title("Magnetometer Data (X vs Y)")
        # plt.xlabel("X")
        # plt.ylabel("Y")
        # plt.grid(True)
        # plt.axis('equal')
        # plt.savefig("magnetometer_plot_after.png")  # Save to file
        # plt.close()

        # data = data_retriever(boards)
        # mag_x, mag_y, mag_z = data[2]
        # acc_x, acc_y, acc_z = data[0]
        # raw_reading = np.array([mag_x, mag_y])
        # corrected_x,corrected_y = correct_mag(raw_reading, offset, transform)



        
        
        #print("Queue size:", q_imu.qsize())
        


if __name__ == "__main__":
    i=0
    q_imu = multiprocessing.Queue(maxsize=1)
    p_imu = multiprocessing.Process(target=get_imu, args=(q_imu,))
    p_imu.start()
    time.sleep(1)

   
