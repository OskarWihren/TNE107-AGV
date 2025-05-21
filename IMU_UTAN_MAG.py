
import time
import multiprocessing
import os
import numpy
import serial
import matplotlib.pyplot as plt
from queue import Empty

import numpy as np

#Inställningar som funkade ibland: 1.66khz och 0 i threshold FAST FUNKADE SEN INTE

ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1)


def normalize_heading(angle):
    angle = angle % 360
    if angle < 0:
        angle += 360
    return angle

def unwrap_heading(current_heading, previous_heading, previous_unwrapped):
    """
    Unwraps compass heading to provide a continuous angle.

    :param current_heading: The current raw heading (between -90 and 270)
    :param previous_heading: The previous raw heading
    :param previous_unwrapped: The previous unwrapped angle
    :return: The new unwrapped angle
    """
    delta = current_heading - previous_heading

    # Detect wraparound
    if delta < -180:
        delta += 360
    elif delta > 180:
        delta -= 360

    return previous_unwrapped + delta




def compute_alpha(gyro_z):
    ALPHA_MIN = 1  # When stationary
    ALPHA_MAX = 1  # When rotating fast
    GYRO_THRESHOLD_LOW = 2.0    # deg/s
    GYRO_THRESHOLD_HIGH = 30  # deg/s
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

def get_imu(queue, calibration_angle):
    ######SETUP SEQUENCE START
    import board
    from adafruit_lsm6ds.lsm6ds3 import LSM6DS3 as LSM6DS
    from adafruit_lsm6ds import Rate
    from adafruit_lsm6ds import GyroRange
    from adafruit_lsm6ds import AccelRange

    from adafruit_lis3mdl import LIS3MDL
    # from adafruit_lis3mdl import Rate_mag
    # from adafruit_lis3mdl import Range as Mag_range

    os.sched_setaffinity(0, {2})

    i2c = board.I2C() 
    accel_gyro = LSM6DS(i2c)
    mag = LIS3MDL(i2c)

    boards = [accel_gyro, mag] #Sparar adresserna för att skickas med i funktionerna separat
    ser.write("0,0\n".encode('UTF-8'))

    #Ställer in datarate för de olika sensorerna
    accel_gyro.accelerometer_data_rate = Rate.RATE_1_66K_HZ
    accel_gyro.accelerometer_range = AccelRange.RANGE_8G
    accel_gyro.gyro_data_rate = Rate.RATE_12_5_HZ
    accel_gyro.gyro_range = GyroRange.RANGE_250_DPS
    time.sleep(0.5)
   
    ######SETUP SEQUENCE END
    

    #START CALIBRATION
    
    samples = 1
    delay = 0.05
    alpha_mag = 0.01
    alpha = 1

    bias_gyro = calibrate_gyro(samples*75, delay, boards)
    print(bias_gyro)
    bias_acc = calibrate_acc(samples, delay, boards)
    print(bias_acc)
    
    fused_yaw = calibration_angle
    previous_heading = 0
    last_time = time.monotonic()
    filtered_unwrapped_mag = 0 #Temporär
    i=0
    vx = 0
    x = 40
    while True: 
        
        acc_x, acc_y, acc_z = accel_gyro.acceleration
        gyro_x, gyro_y, gyro_z = accel_gyro.gyro
      
        result_acc = [acc_x-bias_acc[0], acc_y-bias_acc[1], acc_z-bias_acc[2]]
        result_gyro = [gyro_x-bias_gyro[0], gyro_y-bias_gyro[1], gyro_z-bias_gyro[2]]
        #print("ACCELEROMETER:", result_acc[0])
        #Complementary filter
        now = time.monotonic()
        dt = now - last_time
        #print("Gyro:", result_gyro[2])
        
        if abs(result_gyro[2]) > 0.0: 
            gyro_yaw = fused_yaw - np.degrees(result_gyro[2]) * dt
            vx += result_acc[0] * dt
            x  += vx * dt
            last_time = now
        else:
            gyro_yaw = fused_yaw
            last_time = now
        
        fused_yaw = alpha * gyro_yaw + (1-alpha) * filtered_unwrapped_mag   
        #print("Position from acc:",x)
        #print("Gyro:",gyro_yaw)
        #print("Mag:", unwrapped_mag)   
        #print("Mag filtered: ", filtered_unwrapped_mag) 
        #print("Fused:", fused_yaw)
        #print("\n")


        #Sen lägger vi in den nya datan i queue
        
        try:
            queue.get_nowait() #Rensa kön
        except Empty:
            pass
            
      

    


        actual_heading = normalize_heading(fused_yaw)
        queue.put(actual_heading) #Sätt in i kön
        #print("Actual heading: ", actual_heading)
        
        time.sleep(0.01)
        #print("Max: ",max(angles))
        

        
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
    # q_imu = multiprocessing.Queue(maxsize=1)
    # p_imu = multiprocessing.Process(target=get_imu, args=(q_imu,))
    # p_imu.start()
    # time.sleep(1)

   
