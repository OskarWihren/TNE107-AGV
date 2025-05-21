#Ny verision men samma biblotek som ovan (pyserial)
import serial
import struct
import time
from datetime import datetime
import os
import multiprocessing


def update_lidar(queue):
#def update_lidar():
    os.sched_setaffinity(0,{0})	#kör på kärna 0
        
    PORT = "/dev/ttyUSB0"
    BAUDRATE = 460800
    count = 0
    current_array = [0]*72
    queue.put(current_array)
    check = list(range(0,360,5)) #vektor från 0 till 355 med 5 graders intervall
    
    with serial.Serial(PORT, BAUDRATE, timeout = 1) as ser:
        #ser.write(b'\xA5\x25') #x25 stäng av motorn
        #time.sleep(5)
        ser.write(b'\xA5\x20')	# xA5 start flag, x20 start scanning

        
        
        
        
        #global old_array
        angle = 0
        dist = 0
        old_array = [0]*72
        while True:
            raw_data = ser.read(5)	#Läs 5 bytes [quality, angle_1half, angle_2half, dist_1half, dist_2half]

            if len(raw_data) != 5:
                ser.reset_input_buffer()	#rensa buffer om data ej är fullständig
               
                continue #
        
            try:
                #okomplett = False
            
                values = struct.unpack("<BHH", raw_data)
            
                if len(values) != 3:
                    continue
            
                quality, angle, dist = values
                angle = round((angle >> 1) / 64)
                
                
                if (angle % 5 == 0) & (angle == check[count]):
                    #print(f"Angle: {angle:.0f}, Distance: {dist:.0f} cm")
                    if count < 38 and count > 34:
                            dist = 0
                    else:
                        dist = round(dist/40)
                    
                    current_array[count] = dist
                    count = count + 1
                    
                    if count > len(check)-1:
                        count = 0
                        old_array = current_array
                        while not queue.empty():	#Resna kön innan lägger in nytt värde
                            
                            queue.get()
                            
                        queue.put(old_array)	#Lägg till listan i kön
                        
                        
                        
                #print(f"{angle}, {dist}")
                
            except struct.error:
                continue
        

def stop_lidar(queue):
    os.sched_setaffinity(0,{0})	#kör på kärna 0
        
    PORT = "/dev/ttyUSB0"
    BAUDRATE = 460800
    
    #print("Motorn kanske stannad")
    with serial.Serial(PORT, BAUDRATE, timeout = 1) as ser:
        ser.write(b'\xA5\x25') #x25 stäng av motorn
        print("Motorn stannad")

"""
while True:
    update_lidar()
"""