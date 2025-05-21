import serial
import time
import os
import multiprocessing

#Använd dwm_setup() i början av hela AGV programmet för att starta igång DWM
#Använd dwm_read() när position ska läsas ut. 
ser_dwm = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)

def dwm_setup(): #funktion för att starta igång DWM
    enter_shell_mode()
    dwm_wake_up()
    print("shell mode ok")
    ser_dwm.write(b'lep\r')	#skicka kommando ge mig position

def enter_shell_mode():
    ser_dwm.write(b'\r\r')
    time.sleep(1)
    ser_dwm.reset_input_buffer()
    
def dwm_wake_up():
    ser_dwm.write(b'\r')
    time.sleep(0.1)
    ser_dwm.reset_input_buffer()

def dwm_read(): #funktion för att läsa position
    #os.sched_setaffinity(0, {3})
    
    
        
    try:
        ser_dwm.flushInput()
       
        #print("vaknade")
        #ser.write(b'la\r')	#Skriv vilka anchors vi har kontakt med
        
       
        
        #response=ser_dwm.readline().decode('utf-8', errors='ignore')
        #print(response)
        response = ""
        while "POS" not in response:
            response=ser_dwm.readline().decode('utf-8', errors='ignore')
            if "POS" in response:
                print(response)
                parts = response.split(',')
                
                x_pos = float(parts[1])
                y_pos = float(parts[2])
                pos_quality = int(parts[4])

                # print(x_pos)
                # print(y_pos)
                # print(pos_quality)
                result = [round(x_pos*100), round(y_pos*100), pos_quality]
                
                #print(result)
                return result

	
    
    except KeyboardInterrupt:
        print("Avslutar")


# dwm_setup()
# time.sleep(1)

# q_DWM = multiprocessing.Queue()
# p_DWM = multiprocessing.Process(target=dwm_read, args=(q_DWM,)) 
# p_DWM.start()






