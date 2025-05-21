#MANHATTAN
import os
import serial
import time
import multiprocessing
import math
import RPi.GPIO as GPIO #GPIO biblioteket
from multiprocessing import Value
from Test_of_Mouse import update_mouse	#From "filnamn" import "funktion namn"
from multiprocessing import Process, Queue, Event
from Test_of_lidar import update_lidar
from Test_of_lidar import stop_lidar
from BT_rec import BT_reciever
from BT_tran import BT_transmitter
from datetime import datetime
from sleep_core import sleep_core_2
from IMU_UTAN_MAG import get_imu
from DWM_positionering_async import dwm_setup
from DWM_positionering_async import dwm_read
from mouse_library import open_mouse, read_motion_delta
from Linjefoljare import start_linjefoljare
#from Ultraljud import update_ultraljud

bluetooth_pin = 26 #Blå kabel och lampa
gul_led = 19
grön_led1 = 13
grön_led2 = 6

GPIO.setup(gul_led, GPIO.OUT)
GPIO.setup(grön_led1, GPIO.OUT)
GPIO.setup(grön_led2, GPIO.OUT)

def short_delay(t_us):
    start = time.time_ns()
    while (time.time_ns() - start) < (t_us*1000):
        pass

def setup_ultra():
    global hastighet_ljud
    global trigg_pin
    global echo_pin
    GPIO.setwarnings(False) #för att ta bort varningarna

    #ställ in gpio läge till bcm (broadcom pin number)
    GPIO.setmode(GPIO.BCM)

    trigg_pin = 17
    echo_pin = 27
    hastighet_ljud = 34300

    GPIO.setup(trigg_pin, GPIO.OUT) #konfigurera gpio 17 som input, trigg

    GPIO.setup(echo_pin, GPIO.IN) #konfigurera gpio 27 som output, echo

    GPIO.output(trigg_pin,False)
    
def get_ultraljud():
    
    GPIO.output(trigg_pin,True)
    
    short_delay(10)
    
    GPIO.output(trigg_pin,False)
        
    while GPIO.input(echo_pin) == 0:
        pass

    start_time = time.time()
    
    while GPIO.input(echo_pin) == 1:
        pass

    stop_time = time.time()
    
    res_time = stop_time - start_time
    
    distans = round((res_time*hastighet_ljud)/2,2) #avrunda till 2 decimaler
        
    return distans

def AGV_FRAM():
    ser.write("200,200\n".encode('UTF-8'))

def AGV_STOP():
    ser.write("0,0\n".encode('UTF-8'))
      
def AGV_BAK():
    ser.write("-140,-140\n".encode('UTF-8'))

def AGV_Right():
    ser.write("-140,140\n".encode('UTF-8'))

def AGV_Left():
    ser.write("140,-140\n".encode('UTF-8'))

def AGV_SLOW_RIGHT():
    ser.write("-60,60\n".encode('UTF-8'))

def AGV_SLOW_LEFT():
    ser.write("60,-60\n".encode('UTF-8'))

def AGV_line_left():
    ser.write("100,0\n".encode('UTF-8'))

def AGV_line_right():
    ser.write("0,100\n".encode('UTF-8'))

def STOP_CHECK():
    stop = False
    if not q_BT_reciever.empty():
        mottaget = q_BT_reciever.get()	# [12:06:32:START:X1:Y1:X2:Y2...]
        #print(f"Mottaget meddelande i stop check från ös: {mottaget}")
        split_mottaget = mottaget.split(":") # [12,06,32,START,X1,Y1,X2,Y2,...]
        if len(split_mottaget)>3:
            meddelande_mottaget = split_mottaget[3].strip() 
            print(f"Det ska stå STANNA här: {meddelande_mottaget}")
            if meddelande_mottaget == "STANNA":
                AGV_STOP()
                stop = True
    return stop


global mouse_x
global mouse_y
mouse_x = 40
mouse_y = 40
scale_factor = 0.002395209581

mouse = open_mouse()           # defaults to /dev/input/event2
AGV_riktning = 0    #AGV börjar i Norr

def mouse_data(angle, rotating = False):
    global data
    global mouse_x
    global mouse_y
    if not rotating:
        delta = read_motion_delta(mouse)          
        delta_x = delta.dx
        delta_y = -delta.dy
        delta_x = delta_x*scale_factor
        delta_y = delta_y*scale_factor
    else:
        delta_x = 0
        delta_y = 0
        
    match angle:
        case 0:
            #NORR
            #mouse_x += delta_x
            mouse_y += delta_y

        case 90:
            #ÖST
            #mouse_x += delta_x
            mouse_x += delta_y

        case 180:
            #Söder
            #mouse_x -= delta_x
            mouse_y -= delta_y

        case 270:
            #VÄST
            #mouse_x -= delta_x
            mouse_x -= delta_y


    
    
    data = round(mouse_x),round(mouse_y)
    return data

def rotate_to(target_dir):
    #pause_event.set()
    current_dir = q_IMU.get()
    diff = target_dir - current_dir
    print(current_dir)
    with riktning.get_lock():      # Skickar vilken riktining AGV har till datormusen
        riktning.value = target_dir
    if target_dir == 0: #Om vi ska till NORR
        diff_sample = 0
        slow = True
        
        if current_dir > 180:
            #HÖGER
            if current_dir > 345: #Om vi närmar oss 360 grader sakta ner
                    AGV_SLOW_RIGHT()
            
            else:
                AGV_Right()

            while diff_sample < 180:    #Gå ur om mätdata är stor skillnad från förra mätdatan
                previous_dir = current_dir
                current_dir = q_IMU.get()
                diff_sample = abs(previous_dir - current_dir)
                if current_dir > 345 and slow:   #Om vi närmar oss 360 grader och vi kör snabbt
                    AGV_SLOW_RIGHT()
                    slow = False
    
            AGV_STOP()
                
            
            
        elif current_dir < 180:
            #VÄNSTER
            if current_dir < 15: #Om vi närmar oss 0 grader sakta ner
                    AGV_SLOW_LEFT()

            else:
                AGV_Left()
            
            while diff_sample < 180:
                previous_dir = current_dir
                current_dir = q_IMU.get()
                diff_sample = abs(previous_dir - current_dir)
                if current_dir < 15 and slow:   #Om vi närmar oss 360 grader och vi kör snabbt
                    AGV_SLOW_LEFT()

                    slow = False

            AGV_STOP()
    
                
            
        
    else:   #Om vi inte ska till NORR
        if diff > 0:
            #HÖGER
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
    #pause_event.clear()

def calc_lidar_cord(lidar_vector, AGV_kord,angle):#Lägg till en input för kompass
    print("AGV HAR RIKTNING VID BERÄKNING AV LIDAR: ",angle)
    x_vector = [0]*len(lidar_vector)	#skapar en x_vector fylld med 0
    y_vector = [0]*len(lidar_vector)	#skapar en y_vector fylld med 0
    print("LIDAR VECTOR POS 0",lidar_vector[0])
    for i in range(len(lidar_vector)):
#============================Om lidar data är felaktig=========================

        x = round(math.sin(math.radians(5*i+angle))*lidar_vector[i])
        y = round(math.cos(math.radians(5*i+angle))*lidar_vector[i])
        if x == 0 and y==0: #om x är 0 är också y 0 (ogiltig avläsning)
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
    

# ---------------- parameters you may tune -----------------
ALPHA             = 1      # weight for mouse/gyro when blending
POS_GATE_X        = 1000      # [cm] x‑threshold before DWM is blended
POS_GATE_Y        = 1000      # [cm] y‑threshold before DWM is blended
DWM_JUMP_LIMIT    = 1500     # [cm] max allowed step from successive DWM readings
# -----------------------------------------------------------
# persistent state (module level)
_x_est, _y_est          = None, None      # fused estimate
_prev_dwm_x, _prev_dwm_y = None, None     # last DWM reading

def fuse_position():
    global mouse_offset_x
    global mouse_offset_y
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
    sleep_core_2(0.1) #Sleep för att fixa DWM
    x_m,  y_m          = mouse_data()
    x_dwm, y_dwm, _qual = dwm_read()      


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
    if (_x_est > POS_GATE_X or _y_est > POS_GATE_Y) and use_dwm: #Om vi är utanför gaten, och dwm:en är bra
        # trusted area and DWM is “sane” -> blend
        _x_est = round(ALPHA * x_m + (1 - ALPHA) * x_dwm)
        _y_est = round(ALPHA * y_m + (1 - ALPHA) * y_dwm)
        mouse_offset_x += (_x_est - x_m) #om dwm vid 10 och mus vid 5, lägg på 5 på musen. offset = 10-5 = 5
        mouse_offset_y += (_y_est - y_m)
        print("Vi är nu innanför gaten och dwmen är bra, offset är:", mouse_offset_x, mouse_offset_y)
        
        
    elif (_x_est > POS_GATE_X and _y_est > POS_GATE_Y): #Om vi är utanför gaten men dwm_en indikerar att vi har hoppat
        #Använd bara dwm och uppdatera offset hos musen
        mouse_offset_x += (x_dwm - x_m) #om dwm vid 10 och mus vid 5, lägg på 5 på musen. offset = 10-5 = 5
        mouse_offset_y += (y_dwm - y_m) 
        _x_est = x_dwm
        _y_est = y_dwm
       
    else: #Om vi är innanför gaten
        _x_est, _y_est = x_m, y_m

        
        
    print("SENSOR FUSION: ",_x_est, _y_est)
    print("MOUSE OFFSET:",mouse_offset_x, mouse_offset_y)
    return _x_est, _y_est



ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
) 
setup_ultra()
dwm_setup()

riktning = Value('i', 0)  # Startar som NORR (0)	#[0=Norr, 1=Ã–ST, 2=SÃ–DER, 3=VÃ„ST]
# Global riktnings variabel
current_dir = 0	#AGV startar i NORR

pause_event = Event()
pause_event.clear()  # Start i "inte pausad"

q_mouse = multiprocessing.Queue()
q_BT_reciever = multiprocessing.Queue()
q_BT_transmitter = multiprocessing.Queue()
q_lidar = multiprocessing.Queue()
q_lidar_stop = multiprocessing.Queue()
q_IMU = multiprocessing.Queue()
#q_ultraljud = multiprocessing.Queue()

p_BT_reciever = multiprocessing.Process(target=BT_reciever, args=(q_BT_reciever,))
p_BT_transmitter = multiprocessing.Process(target=BT_transmitter, args=(q_BT_transmitter,))
#p_mouse = multiprocessing.Process(target=update_mouse, args=(q_mouse,riktning,pause_event))
p_lidar = multiprocessing.Process(target=update_lidar, args=(q_lidar,))
p_IMU = multiprocessing.Process(target=get_imu, args=(q_IMU,0,))
#p_ultraljud = multiprocessing.Process(target=update_ultraljud, args=(q_ultraljud,))

p_IMU.start()
while q_IMU.empty():    #Gå vidare när p_IMU är färdigkalibrerad och får sitt första mätvärde
    pass
#p_ultraljud.start()
#while q_ultraljud.empty():    #Gå vidare när ultarljud är klar med sin setup
    #pass
p_BT_reciever.start()
p_BT_transmitter.start()
#p_mouse.start()
p_lidar.start()

ultraljud_threash = 3
linjefoljare_klar = False

try:
    
    while True:
        #AGV_STOP()
        mottaget = q_BT_reciever.get()	# [12:06:32:START:X1:Y1:X2:Y2...]
        split_mottaget = mottaget.split(":") # [12,06,32,START,X1,Y1,X2,Y2,...]
        #kts_tid = [int(s) for s in split_mottaget[0:3]]
        ed_tid = time.strftime("%H:%M:%S:", time.localtime())
        meddelande_mottaget = split_mottaget[3].strip()
        ack_meddelande = split_mottaget[3:] 
        #q_BT_transmitter.put((f"ACK:{kts_tid[0]}:{kts_tid[1]}:{kts_tid[2]}:{ed_tid}:{ack_meddelande}"))	#Svara ÖS med ACK samt meddelandet som togs emot

        
        match meddelande_mottaget:
            case "KART":
                target_list = [int(s) for s in split_mottaget[4:]]
                nodstopp = False
                #fuse_position()
                current_mouse = mouse_data(AGV_riktning)
                #current_ultraljud = get_ultraljud()
                print(target_list)
                

                for i in range(0,len(target_list),2):
                    #fuse_position()
                    current_mouse = mouse_data(AGV_riktning)
                    print("Mouse now i KART:", current_mouse)
                    
                    current_dir = q_IMU.get()
                    print("Med riktning: ",current_dir)
                    x_hinder_pos, y_hinder_pos = calc_lidar_cord(q_lidar.get(),current_mouse,current_dir)
                    x_current_pos, y_current_pos = current_mouse
                    ed_tid = time.strftime("%H:%M:%S:", time.localtime())
                    q_BT_transmitter.put((f"{ed_tid}HINDER:{x_hinder_pos}:{y_hinder_pos}:{x_current_pos}:{y_current_pos}:"))
                    current_ultraljud = get_ultraljud()
                    if current_ultraljud < ultraljud_threash:
                        rotate_to((riktning.value+180)%360)
                        AGV_riktning = riktning.value + 180
                        if AGV_riktning > 270:
                            AGV_riktning = 0
                        nodstopp = True
                        break
                    print(f"{ed_tid}:POSITION:{current_mouse[0]}:{current_mouse[1]}")
                    time.sleep(0.1)
                    print(dwm_read())
                    stopp = STOP_CHECK()
                    if stopp == True:
                        break
                    
                    if current_mouse[0] < target_list[i]+1 and current_mouse[0] > target_list[i]-1: #Kollar om vi redan står på x-axeln
                        
                        if current_mouse[1] < target_list[i+1]:
                            #rotera till norr
                            rotate_to(0)
                            AGV_riktning = 0
                            current_mouse = mouse_data(AGV_riktning,True)
                            AGV_FRAM()
                            
                            while current_mouse[1] < target_list[i+1]:
                            #while current_mouse[1] < target_list[i+1]:
                                current_mouse = mouse_data(AGV_riktning)
                                #current_ultraljud = get_ultraljud() 
                            
                            AGV_STOP()
                            

                            
                          
                           

                        else:
                            #rotera till söder
                            rotate_to(180)
                            AGV_riktning = 180
                            current_mouse = mouse_data(AGV_riktning,True)
                            AGV_FRAM()
                            
                            while current_mouse[1] > target_list[i+1]:
                            #while current_mouse[1] > target_list[i+1]:
                                
                                current_mouse = mouse_data(AGV_riktning)
                                #current_ultraljud = get_ultraljud()

                            
                            AGV_STOP()

                            
                 
                            


                    else:   #om vi redan står på rätt y-kord
                        print("ELSE")
                        if current_mouse[0] < target_list[i]:
                            #rotera till öst
                            rotate_to(90)  
                            AGV_riktning = 90
                            current_mouse = mouse_data(AGV_riktning,True)
                            AGV_FRAM()
                            
                            while current_mouse[0] < target_list[i]:
                            #while current_mouse[0] < target_list[i]:
                                
                                current_mouse = mouse_data(AGV_riktning)
                                #current_ultraljud = get_ultraljud()
                                
                            AGV_STOP()

                            
                            
                           
                        else:
                            #rotera till väst
                            rotate_to(270)
                            AGV_riktning = 270
                            current_mouse = mouse_data(AGV_riktning,True)
                            AGV_FRAM()
                            
                            while current_mouse[0] > target_list[i]:
                            #while current_mouse[0] > target_list[i]:
                               
                                current_mouse = mouse_data(AGV_riktning)
                                
                                #current_ultraljud = get_ultraljud()
                            
                            AGV_STOP()          
                            
                       
                            
                if nodstopp == False:
                    #ed_tid = time.strftime("%H:%M:%S:", time.localtime())
                    #q_BT_transmitter.put((f"{ed_tid}FRAMME"))	#Svara ÖS med ACK samt meddelandet som togs emot
                    print("FRAMME!")
                    GPIO.output(grön_led1,True)
                    calibration_angle = q_IMU.get()
                    p_IMU.terminate()
                    p_IMU.join()
                    p_IMU = multiprocessing.Process(target=get_imu, args=(q_IMU, calibration_angle))
                    p_IMU.start()
                    sleep_core_2(4)
                    GPIO.output(grön_led1,False)

                    
                else:
                    q_BT_transmitter.put((f"{ed_tid}FAST:{current_mouse[0]}:{current_mouse[1]}"))
                    print("NODSTOPP")
                """
                if current_ultraljud < 20:
                    nodstopp = True
                    q_BT_transmitter.put((f"{ed_tid}FAST:{current_mouse[0]}:{current_mouse[1]}"))
                    print("FAST!!!!!!")
                    break

                else:
                    print(f"FRAMME! Nuvarande koordinater: x={current_mouse[0]}, y={current_mouse[1]}")
                sleep_core_2(0.5)
                """
                current_mouse = mouse_data(AGV_riktning)
                current_dir = q_IMU.get()
                x_hinder_pos, y_hinder_pos = calc_lidar_cord(q_lidar.get(),current_mouse,current_dir)
                # fuse_position()
                x_current_pos, y_current_pos = current_mouse
                ed_tid = time.strftime("%H:%M:%S:", time.localtime())
                q_BT_transmitter.put((f"{ed_tid}HINDER:{x_hinder_pos}:{y_hinder_pos}:{x_current_pos}:{y_current_pos}:"))
                
                print(f"{ed_tid}:SISTA POSITION:{current_mouse[0]}:{current_mouse[1]}")      
                



            case "TRANS":
                print(split_mottaget)
                target_list = [int(s) for s in split_mottaget[4:]]
                nodstopp = False
                #fuse_position()
                current_mouse = mouse_data(AGV_riktning)
                #current_ultraljud = get_ultraljud()
                print(target_list)
                

                for i in range(0,len(target_list),2):
                    #fuse_position()
                    current_mouse = mouse_data(AGV_riktning)
                    print("Mouse now i TRANS:", current_mouse)
                    
                    current_dir = q_IMU.get()
                    print("Med riktning: ",current_dir)
                    x_current_pos, y_current_pos = current_mouse
                    ed_tid = time.strftime("%H:%M:%S:", time.localtime())
                    q_BT_transmitter.put((f"{ed_tid}POSITION:{x_current_pos}:{y_current_pos}"))
                    current_ultraljud = get_ultraljud()
                    # if current_ultraljud < ultraljud_threash:
                    #     rotate_to((riktning.value+180)%360)
                    #     AGV_riktning = riktning.value + 180
                    #     if AGV_riktning > 270:
                    #         AGV_riktning = 0
                    #     nodstopp = True
                    #     break
                    print(f"{ed_tid}:POSITION:{current_mouse[0]}:{current_mouse[1]}")
                    time.sleep(0.1)
                    print(dwm_read())
                    stopp = STOP_CHECK()
                    if stopp == True:
                        break
                    
                    if current_mouse[0] < target_list[i]+1 and current_mouse[0] > target_list[i]-1: #Kollar om vi redan står på x-axeln
                        
                        if current_mouse[1] < target_list[i+1]:
                            #rotera till norr
                            rotate_to(0)
                            AGV_riktning = 0
                            current_mouse = mouse_data(AGV_riktning,True)
                            AGV_FRAM()
                            
                            while current_mouse[1] < target_list[i+1]:
                                current_mouse = mouse_data(AGV_riktning) 
                            
                            AGV_STOP()
                            

                            
                          
                           

                        else:
                            #rotera till söder
                            rotate_to(180)
                            AGV_riktning = 180
                            current_mouse = mouse_data(AGV_riktning,True)
                            AGV_FRAM()
                            
                            while current_mouse[1] > target_list[i+1]:
                            #while current_mouse[1] > target_list[i+1]:
                                
                                current_mouse = mouse_data(AGV_riktning)
                                #current_ultraljud = get_ultraljud()

                            
                            AGV_STOP()

                            
                 
                            


                    else:   #om vi redan står på rätt y-kord
                        if current_mouse[0] < target_list[i]:
                            #rotera till öst
                            rotate_to(90)  
                            AGV_riktning = 90
                            current_mouse = mouse_data(AGV_riktning,True)
                            AGV_FRAM()
                            
                            while current_mouse[0] < target_list[i]:
                            #while current_mouse[0] < target_list[i]:
                                
                                current_mouse = mouse_data(AGV_riktning)
                                #current_ultraljud = get_ultraljud()
                                
                            AGV_STOP()

                            
                            
                           
                        else:
                            #rotera till väst
                            rotate_to(270)
                            AGV_riktning = 270
                            current_mouse = mouse_data(AGV_riktning,True)
                            AGV_FRAM()
                            
                            while current_mouse[0] > target_list[i]:
                            #while current_mouse[0] > target_list[i]:
                               
                                current_mouse = mouse_data(AGV_riktning)
                                
                                #current_ultraljud = get_ultraljud()
                            
                            AGV_STOP()          
                            
                       
                            
                if nodstopp == False:
                    error_pos = start_linjefoljare()
                    GPIO.output(grön_led2,True)
                    rotate_to(AGV_riktning)
                    
                    match AGV_riktning:
                        case 0:
                            print("CASE:0")
                            besok_pos_x = target_list[-2] #x-värde av besökplats
                            besok_pos_y = target_list[-1] + 20 #y-värde
                            mouse_x = mouse_x
                            mouse_y = besok_pos_y - error_pos - 12 #Beräknar var vi står efter lyckat besök
                            current_mouse = mouse_data(AGV_riktning,True) #Rensa kön för musen (ignorera all data tills hit)
                            AGV_BAK()
                            print("BAK")
                            while current_mouse[1] > target_list[-1]:
                                current_mouse = mouse_data(AGV_riktning)
                            AGV_STOP()
                        case 90:
                            print("CASE:90")
                            besok_pos_x = target_list[-2] + 20#x-värde av besökplats
                            besok_pos_y = target_list[-1] #y-värde
                            mouse_x = besok_pos_x - error_pos -12
                            mouse_y = mouse_y #Beräknar var vi står efter lyckat besök
                            current_mouse = mouse_data(AGV_riktning,True) #Rensa kön för musen (ignorera all data tills hit)
                            AGV_BAK()
                            print("BAK")
                            while current_mouse[0] > target_list[-2]:
                                current_mouse = mouse_data(AGV_riktning)
                            AGV_STOP()

                        case 180:
                            print("CASE:180")
                            besok_pos_x = target_list[-2] #x-värde av besökplats
                            besok_pos_y = target_list[-1] - 20 #y-värde
                            mouse_x = mouse_x
                            mouse_y = besok_pos_y + error_pos + 12
                            print("POSITION FRÅN BESÖKSPLATS:", mouse_x, mouse_y)
                            #Beräknar var vi står efter lyckat besök
                            current_mouse = mouse_data(AGV_riktning,True) #Rensa kön för musen (ignorera all data tills hit)
                            print("POSITION FRÅN BESÖKSPLATS:", mouse_x, mouse_y)
                            AGV_BAK()
                            print("BAK")
                            print("VI STÅR PÅ: ", current_mouse)
                            print("VI SKA TILL: ", target_list[-1])
                            
                            while current_mouse[1] < target_list[-1]:
                                current_mouse = mouse_data(AGV_riktning)
                                #print("BACKAR NU!!!, pos:", current_mouse)

                            AGV_STOP()
                            print("VI ÄR KLARA MED BACKEN OCH STÅR PÅ:", current_mouse)
                        case 270:
                            print("CASE:270")
                            besok_pos_x = target_list[-2] - 20#x-värde av besökplats
                            besok_pos_y = target_list[-1] #y-värde
                            mouse_x = besok_pos_x + error_pos + 12
                            mouse_y = mouse_y #Beräknar var vi står efter lyckat besök
                            current_mouse = mouse_data(AGV_riktning,True) #Rensa kön för musen (ignorera all data tills hit)
                            AGV_BAK()
                            print("BAK")
                            while current_mouse[1] < target_list[-2]:
                                current_mouse = mouse_data(AGV_riktning)
                            AGV_STOP()
                    
                    print("POSITION EFTER BESÖKSHANTERING ÄR HELT KLAR:",current_mouse)
                    ed_tid = time.strftime("%H:%M:%S:", time.localtime())
                    q_BT_transmitter.put((f"{ed_tid}FARDIG:{current_mouse[0]}:{current_mouse[1]}"))	
                    print("FARDIG!")
                    GPIO.output(grön_led2,False)
                else:
                    
                    q_BT_transmitter.put((f"{ed_tid}FAST:{current_mouse[0]}:{current_mouse[1]}"))

                    print("NODSTOPP")
                

            
            case "POSITION":
                print("POSITION")
                #fuse_position()
                current_mouse = mouse_data(AGV_riktning)
                #ultra = get_ultraljud()
                IMU_data = q_IMU.get()
                #print(f"MOUSE: {data} | IMU: {IMU_data} | Avstånd fram: {ultra}")
                print(f"MOUSE: {current_mouse[0]}:{current_mouse[1]} | IMU: {IMU_data}")
                q_BT_transmitter.put((f"POSITION:{current_mouse[0]}:{current_mouse[1]}"))

            case "HINDER":
                    #Skica endast lidar
                    #fuse_position()
                    mouse_now = mouse_data(AGV_riktning)
                    current_dir = q_IMU.get()
                    print("Mouse now i HINDER:", mouse_now)
                    x_hinder_pos, y_hinder_pos = calc_lidar_cord(q_lidar.get(),mouse_now,current_dir)
                    x_current_pos, y_current_pos = mouse_now
                    ed_tid = time.strftime("%H:%M:%S:", time.localtime())
                    q_BT_transmitter.put((f"{ed_tid}HINDER:{x_hinder_pos}:{y_hinder_pos}:{x_current_pos}:{y_current_pos}"))
                    print("Nu kör vi HINDER")
                    print(f"x hinder: {x_hinder_pos}")
                    print(f"y hinder: {y_hinder_pos}")
                
            case _:
                #Om ett okÃ¤nt kommando skrivs
                print("OkÃ¤nt kommando")
                
        sleep_core_2(0.2)
                
                
except KeyboardInterrupt:
    print("Avslyutar")
    AGV_STOP()
#===================================Stoppa lidar===============================================
    p_lidar_stop = multiprocessing.Process(target=stop_lidar, args=(q_lidar_stop,))	#Starta lidar stop processen
    p_lidar_stop.start()	#starta funktionen
    p_lidar_stop.join()		#Avsluta processen
#===============================================================================================
    #Måste använda terminate() för finns while True i dessa funktioner
    #p_mouse.terminate()
    p_lidar.terminate()
    p_BT_reciever.terminate()
    p_BT_transmitter.terminate()
    p_IMU.terminate()
    #p_ultraljud.terminate()
    p_BT_reciever.join()
    p_BT_transmitter.join()
    #p_mouse.join()
    p_lidar.join()
    p_IMU.join()
    #p_ultraljud.join()
    #p_dwm.join()
    GPIO.output(grön_led1,False)
    GPIO.output(grön_led2,False)
    GPIO.output(gul_led,False)
    
