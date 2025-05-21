#LateNightGrind

import os
import serial
import time
import multiprocessing
import math
from multiprocessing import Value
from Test_of_Mouse import update_mouse	#From "filnamn" import "funktion namn"
from multiprocessing import Process, Queue, Event
from Test_of_lidar import update_lidar
from Test_of_lidar import stop_lidar
from sleep_core import sleep_core_2
from IMU_UTAN_MAG import get_imu #=================================
from DWM_positionering_async import dwm_read
from DWM_positionering_async import dwm_setup
from mouse_library import open_mouse, read_motion_delta



# Global riktnings variabel
current_dir = 0	#AGV startar i NORR 
   
def AGV_FRAM():
    ser.write("150,0\n".encode('UTF-8'))

def AGV_STOP():
    ser.write("0,0\n".encode('UTF-8'))
    
def AGV_BAK():
    ser.write("-150,-150\n".encode('UTF-8'))

def AGV_Right():
    ser.write("-150,150\n".encode('UTF-8'))

def AGV_Left():
    ser.write("150,-150\n".encode('UTF-8'))

def AGV_SLOW_RIGHT():
    ser.write("-50,50\n".encode('UTF-8'))

def AGV_SLOW_LEFT():
    ser.write("50,-50\n".encode('UTF-8'))


def AGV_CALIBRATE_ROT():
    ser.write("-70,70\n".encode('UTF-8'))

# ---------------- parameters you may tune -----------------
ALPHA             = 0.5      # weight for mouse/gyro when blending
POS_GATE_X        = 100      # [cm] x‑threshold before DWM is blended
POS_GATE_Y        = 100      # [cm] y‑threshold before DWM is blended
DWM_JUMP_LIMIT    = 10.0     # [cm] max allowed step from successive DWM readings
# -----------------------------------------------------------

# persistent state (module level)
_x_est, _y_est          = None, None      # fused estimate
_prev_dwm_x, _prev_dwm_y = None, None     # last DWM reading


def fuse_position(m_data, dwm_data):
    """
    Fuse absolute mouse/gyro (m_data) with DWM1001 (dwm_data)
    while rejecting large *DWM* jumps (> DWM_JUMP_LIMIT).

    Parameters
    ----------
    m_data   : (x_cm, y_cm)              absolute mouse/gyro position
    dwm_data : (x_m, y_m, quality)       DWM1001 position in metres

    Returns
    -------
    (x_fused, y_fused)  – centimetres
    """
    global _x_est, _y_est, _prev_dwm_x, _prev_dwm_y

    # unpack ---------------------------------------------------------------
    x_m,  y_m               = m_data
    x_dwm_m, y_dwm_m, _qual = dwm_data         # DWM is delivered in metres
    x_dwm,  y_dwm           = 100*x_dwm_m, 100*y_dwm_m   # convert to cm

    # ---------- first run: initialise ------------------------------------
    if _x_est is None:
        _x_est, _y_est     = x_m, y_m
        _prev_dwm_x, _prev_dwm_y = x_dwm, y_dwm
        return _x_est, _y_est

    # ---------- DWM‑jump gate --------------------------------------------
    if _prev_dwm_x is not None:
        dx_dwm = x_dwm - _prev_dwm_x
        dy_dwm = y_dwm - _prev_dwm_y
        dwm_jump = (dx_dwm**2 + dy_dwm**2) ** 0.5   # Euclidean jump (cm)
    else:
        dwm_jump = 0.0                              # first comparison

    use_dwm = dwm_jump <= DWM_JUMP_LIMIT

    # store current DWM for next cycle
    _prev_dwm_x, _prev_dwm_y = x_dwm, y_dwm

    # ---------- decide how to update estimate ----------------------------
    if (_x_est > POS_GATE_X and _y_est > POS_GATE_Y) and use_dwm:
        # trusted area and DWM is “sane” → blend
        _x_est = ALPHA * x_m + (1 - ALPHA) * x_dwm
        _y_est = ALPHA * y_m + (1 - ALPHA) * y_dwm
    else:
        # outside gate or DWM jump too large → rely solely on mouse/gyro
        _x_est, _y_est = x_m, y_m

    return _x_est, _y_est

mouse = open_mouse()           # defaults to /dev/input/event2
global mouse_x
global mouse_y
mouse_x = 40
mouse_y = 40
scale_factor = 0.002395209581

def mouse_data():
    global mouse_x
    global mouse_y
    global data
    delta = read_motion_delta(mouse)
    if delta.dx != 0 or delta.dy != 0:
        print(f"Mouse moved: Δx={delta.dx}, Δy={delta.dy}")
    delta_x = delta.dx
    delta_y = -delta.dy
    delta_x = delta_x*scale_factor
    delta_y = delta_y*scale_factor
    mouse_x += delta_x
    mouse_y += delta_y

    data = mouse_x,mouse_y
    return data
    
def rotate_to(target_dir):
    current_dir = q_IMU.get()
    diff = target_dir - current_dir
    with riktning.get_lock():      # Skickar vilken riktining AGV har till datormusen
        riktning.value = target_dir
    if target_dir == 0:
        diff_sample = 0
        
        if current_dir > 180:
            #HÖGER
            AGV_Right()
            while diff_sample < 180:
                previous_dir = current_dir
                current_dir = q_IMU.get()
                diff_sample = abs(previous_dir - current_dir)
            AGV_STOP()
                
            
            
        elif current_dir < 180:
            #VÄNSTER
            AGV_Left()
            
            while diff_sample < 180:
                previous_dir = current_dir
                current_dir = q_IMU.get()
                diff_sample = abs(previous_dir - current_dir)
            AGV_STOP()
            
        else:
            print("Redan rätt")
                
            
        
    else:
        print("ELSE")
        if diff > 0:
            #HÖGER
            print("HÖGER")
            slow = True
            if abs(target_dir - current_dir) < 15:
                AGV_SLOW_RIGHT()
            else:
                AGV_Right()
            while target_dir > current_dir:
                current_dir = q_IMU.get()
                if abs(target_dir - current_dir) < 15 and slow:
                    AGV_SLOW_RIGHT()
                    slow = False
            AGV_STOP()

            
        elif diff < 0:
            #VÄSNTER
            print("VÄNSTER")
            slow = True
            if abs(target_dir - current_dir) < 15:
                AGV_SLOW_LEFT()
            else:
                AGV_Left()

            while target_dir < current_dir:
                current_dir = q_IMU.get()
                if abs(target_dir - current_dir) < 15 and slow:
                    AGV_SLOW_LEFT()
                    slow = False
            AGV_STOP()


        
    
    
    
def calc_lidar_cord(lidar_vector, AGV_kord):#Lägg till en input för kompass
    x_vector = [0]*len(lidar_vector)	#skapar en x_vector fylld med 0
    y_vector = [0]*len(lidar_vector)	#skapar en y_vector fylld med 0
    
    for i in range(len(lidar_vector)):
#============================Om lidar data är felaktig=========================
        x = round(math.sin(math.radians(5*i+0))*lidar_vector[i])
        y = round(math.cos(math.radians(5*i+0))*lidar_vector[i])
        if x == 0: #om x är 0 är också y 0 (ogiltig avläsning)
            x_vector[i] = 10000
            y_vector[i] = 10000
        else:
            x_vector[i] = x
            y_vector[i] = y
            
#===============================================================================
        
    x_current_pos, y_current_pos = AGV_kord
    x_hinder_pos = [x_current_pos + x for x in x_vector]	#Beräknar vilka koordinater hindren har mha datormus
    y_hinder_pos = [y_current_pos + y for y in y_vector]
    return x_hinder_pos, y_hinder_pos
    







ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

dwm_setup()
time.sleep(1)

riktning = Value('i', 0)  # Startar som NORR (0)	#[0=Norr, 1=Ã–ST, 2=SÃ–DER, 3=VÃ„ST]


pause_event = Event()
pause_event.clear()  # Start i "inte pausad"

q_mouse = multiprocessing.Queue()
q_IMU = multiprocessing.Queue() #==================================#==================================

#p_mouse = multiprocessing.Process(target=update_mouse, args=(q_mouse,riktning,pause_event))
p_IMU = multiprocessing.Process(target=get_imu, args=(q_IMU,0)) #=====================================#==================================


p_IMU.start()   #==========================================#==================================
while q_IMU.empty():
    pass
#p_mouse.start()

try:
    
    while True:
        message = input("Skriv ditt meddelande: ") #skriv in meddelande till esp
        #ser.write(f"{message}\n".encode('UTF-8')) #skickar meddelande till esp   
        
        
        #if ser.in_waiting > 0:
        #data = ser.readline().decode('UTF-8').strip() #tar emot meddelande frÃ¥n esp
        #print(f"Mottaget: {data}") #skriver ut mottaget meddelande frÃ¥n esp
        
        match message:
        
            case "START":
                target = [0,5,0,10,0,15,0,20,0,25,5,25,10,25,15,25,20,25,25,25]
                #target = [0,40,40,40]
                current_mouse = mouse_data()
                for i in range(0,len(target),2):
                    print(current_mouse)
                    
                    if current_mouse[0] < target[i]+2 and current_mouse[0] > target[i]-2: #Kollar om vi redan står på x-axeln
                        
                        if current_mouse[1] < target[i+1]-1:
                            #rotera till norr
                            
                            rotate_to(0)
                            current_mouse = mouse_data()
                            AGV_FRAM()
                            while current_mouse[1] < target[i+1]:
                                current_mouse = mouse_data()
                                #print(current_mouse)

                            AGV_STOP()
                            current_mouse = mouse_data()
                            print(f"EFTER WHILE: x={current_mouse[0]} | y={current_mouse[1]}")
                            ser.write("0,0\n".encode('UTF-8'))

                            current_mouse = mouse_data()
                            print(f"FRAMME! Nuvarande koordinater: x={current_mouse[0]}, y={current_mouse[1]}")
                            sleep_core_2(0.5)

                        else:
                            #rotera till söder
                            rotate_to(2)
                            current_mouse = mouse_data()
                            AGV_FRAM()
                            
                            while current_mouse[1] > target[i+1]:
                                current_mouse = mouse_data()
                            
                            AGV_STOP()
                            current_mouse = mouse_data()
                            print(f"FRAMME! Nuvarande koordinater: x={current_mouse[0]}, y={current_mouse[1]}")
                            sleep_core_2(0.5)


                    else:   #om vi redan står på rätt y-kord
                        print("ELSE")
                        if current_mouse[0] < target[i]:
                            #rotera till öst
                            rotate_to(1)
                            current_mouse = mouse_data()
                            AGV_FRAM()
                            
                            while current_mouse[0] < target[i]:
                                current_mouse = mouse_data()
                                
                            AGV_STOP()
                            current_mouse = mouse_data()
                            print(f"FRAMME! Nuvarande koordinater: x={current_mouse[0]}, y={current_mouse[1]}")
                            sleep_core_2(0.5)

                        else:
                            #rotera till väst
                            rotate_to(3)
                            AGV_FRAM()
                            current_mouse = mouse_data()
                            
                            while current_mouse[0] > target[i]:
                                current_mouse = mouse_data()
                                
                            AGV_STOP()
                            current_mouse = mouse_data()
                            print(f"FRAMME! Nuvarande koordinater: x={current_mouse[0]}, y={current_mouse[1]}")
                            sleep_core_2(0.5)


            case "STOPP":
                print("STOPP")
                ser.write("0,0\n".encode('UTF-8'))
            
            case "FRAM":
                print("FRAM")
                current_mouse = mouse_data()
                print("Före mus:",current_mouse)
                #while True:
                ser.write("120,120\n".encode('UTF-8'))
                sleep_core_2(0.1)
                current_mouse = mouse_data()
                ser.write("0,0\n".encode('UTF-8'))
                print("Innan stopp",current_mouse)
                current_mouse = mouse_data()
                print("Efter stopp:",current_mouse)
                sleep_core_2(0.5)
            
            case "BAK":
                print("BAK")
                ser.write("-100,-100\n".encode('UTF-8'))
                sleep_core_2(5)
                ser.write("0,0\n".encode('UTF-8'))
                
            case "NORR":
                rotate_to(0)
                
                

            case "ÖST":
                rotate_to(90)
                
                    
            case "VÄST":
                rotate_to(270)
                
                    
            case "SÖDER":
                rotate_to(180)
                
                
            case "CHECK":
                m_data = mouse_data()
                IMU_data = q_IMU.get()  #==================================#==================================
                dwm_data = dwm_read()
                print("DWM:", dwm_data)

                print(f"MOUSE: {m_data} | IMU: {IMU_data}")
                x_fused, y_fused = fuse_position(m_data,dwm_data)
                print("X fused:", x_fused, "Y fused", y_fused)

                


                
            case _:
                #Om ett okÃ¤nt kommando skrivs
                print("OkÃ¤nt kommando")
                
        sleep_core_2(0.2)
                
                
except KeyboardInterrupt:
    print("Avslyutar")
    #p_mouse.terminate()
    #p_mouse.join()
    p_IMU.terminate()   #==================================#==================================
    p_IMU.join()    #==================================#==================================