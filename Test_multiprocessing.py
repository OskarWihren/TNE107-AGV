import os
import time
import multiprocessing
import math
import serial
from multiprocessing import Value
from Test_of_Mouse import update_mouse	#From "filnamn" import "funktion namn"
from Test_of_lidar import update_lidar
from Test_of_lidar import stop_lidar
from BT_rec import BT_reciever
from BT_tran import BT_transmitter
from datetime import datetime
from sleep_core import sleep_core_2
#from Ultraljud import update_ultraljud
#from DWM_positionering_async import dwm_setup, dwm_read

current_dir = 0	#AGV startar i NORR    

def setup_UART():
    global uart
    uart = serial.Serial(
        port='/dev/ttyS0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
    
def send_UART(input):
    uart.write(f"{input}\n".encode('UTF-8')) #skickar meddelande till esp
    
def read_UART():
    data = uart.readline().decode('UTF-8').strip() #tar emot meddelande från esp
    return data

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
    
def calc_AGV_cord(mouse_data):
    #kombinera datormus och DWM 
    x_mouse, y_mouse = mouse_data
    AGV_x = (x_mouse + x_dwm)/2
    AGV_y =	(y_mouse + y_dwm)/2
    
    return AGV_x, AGV_y

        


def main():
    
    #skapa en kö för varje sensor
    q_BT_reciever = multiprocessing.Queue()
    q_BT_transmitter = multiprocessing.Queue()
    q_mouse = multiprocessing.Queue()
    q_lidar = multiprocessing.Queue()
    q_lidar_stop = multiprocessing.Queue()
    #q_ultraljud = multiprocessing.Queue()
    #q_DWM = multiprocessing.Queue()
    #stop_event = multiprocessing.Event()	#skapar ett event för nödstop
    
    riktning = Value('i', 0)
    print("startar prcesser")
    #starta processerna
    p_BT_reciever = multiprocessing.Process(target=BT_reciever, args=(q_BT_reciever,))
    p_BT_transmitter = multiprocessing.Process(target=BT_transmitter, args=(q_BT_transmitter,))
    p_mouse = multiprocessing.Process(target=update_mouse, args=(q_mouse,riktning,))
    p_lidar = multiprocessing.Process(target=update_lidar, args=(q_lidar,))
    #p_ultraljud = multiprocessing.Process(target=update_ultraljud, args=(q_ultraljud,))
    #p_DWM = multiprocessing.Process(target=dwm_read, args=(q_DWM,))
    #p_stop_event = multiprocessing.Process(target=AGV_stop, args=(stop_event,))
    
    #dwm_setup()

    print("start")
    p_BT_reciever.start()
    p_BT_transmitter.start()
    p_mouse.start()
    p_lidar.start()
    #p_ultraljud.start()
    #p_DWM.start()
    #p_stop_event.start()
    
    setup_UART()
    
    
    
    try:
        
        while True:
            q_BT_transmitter.put(("datetime.now(),mottaget"))	#Svara ÖS med ACK samt meddelandet som togs emot
            mottaget = q_BT_reciever.get()
            split_mottaget = mottaget.split(":")
            meddelande_mottaget = split_mottaget[3].strip()
            #print(f"Mottaget meddelande: {meddelande_mottaget}")
            q_BT_transmitter.put(("datetime.now(),mottaget"))	#Svara ÖS med ACK samt meddelandet som togs emot
            
            #===========
            
            #if ultraljudssensor_avstånd < 5 cm {STOP, skicka lidar data samt AGV position?}
            
            
            match meddelande_mottaget:
                case "START":
                    #START:X:Y:X:Y:X...
                    
                    #denna är knaas!!
                    """
                    for i in range(4,len(split_mottaget)-1,2):
                        agv_angle = 0
                        x_goal = split_mottaget[i]
                        y_goal = split_mottaget[i+1]
                        mouse_now = q_mouse.get()#agv calc koordinater sen
                        q_mouse.put(mouse_now)
                        
                        x_diff = x_goal - mouse_now[0]
                        y_diff = y_goal - mouse_now[1]
                        
                        alpha = math.atan(y_diff/x_diff)
                        while alpha not agv_angle:
                            send_UART("150,-150")
                            
                        send_UART("0,0")
                        
                        while mouse_now[0] not x_goal | mouse_now[1] not y_goal:
                            send_UART("255,255")
                            
                        send_UART("0,0")
                        
                        if not q_BT_reciever.empty()
                            mottaget = q_BT_reciever.get()
                            split_mottaget = mottaget.split(":")
                            meddelande_mottaget = split_mottaget[3].strip()
                            
                            match meddelande_mottaget:
                                case "AVBRYT":
                                    break
                                    """
                                
                    
                    x_hinder_pos, y_hinder_pos = calc_lidar_cord(q_lidar.get(),q_mouse.get())
                    q_BT_transmitter.put((f"{x_hinder_pos}:{y_hinder_pos}"))                    
                    
                case "KOR":
                    #KOR:X:Y:X:Y:X:...
                    #UART
                    #send_UART(255,255)
                    print("Nu kör vi KOR!")
                    mouse_now = q_mouse.get()
                    q_mouse.put(mouse_now)
                    calc_AGV_cord(mouse_now)	#Beräkna AGV koordinater, lägg till DWM!
                    
                    x_hinder_pos, y_hinder_pos = calc_lidar_cord(q_lidar.get(),mouse_now)
                    q_BT_transmitter.put((f"KOR:{x_hinder_pos}:{y_hinder_pos}"))	#Skicka som en array .put((...))
                    
                case "HINDER":
                    #Skica endast lidar
                    mouse_now = q_mouse.get()
                    q_mouse.put(mouse_now)
                    x_hinder_pos, y_hinder_pos = calc_lidar_cord(q_lidar.get(),mouse_now)
                    x_current_pos, y_current_pos = mouse_now
                    q_BT_transmitter.put((f"HINDER:{x_hinder_pos}:{y_hinder_pos}:{x_current_pos},{y_current_pos}"))
                    print("Nu kör vi LIDAR")
                    
                case "AVBRYT":
                    #Avbryt all aktivitet kopplat till AGV och ÖS
                    print("Nu kör vi AVBRYT!")
                    send_UART("0,0")
                    #Måste använda terminate() för finns while True i dessa funktioner
                    p_mouse.terminate()
                    p_lidar.terminate()
                    p_BT_reciever.terminate()
                    p_BT_transmitter.terminate()
                    #p_ultraljud.terminate()
                    p_BT_reciever.join()
                    p_BT_transmitter.join()
                    p_mouse.join()
                    p_lidar.join()
                    #p_ultraljud.join()
                    #===================================Stoppa lidar===============================================
                    p_lidar_stop = multiprocessing.Process(target=stop_lidar, args=(q_lidar_stop,))	#Starta lidar stop processen
                    p_lidar_stop.start()	#starta funktionen
                    p_lidar_stop.join()		#Avsluta processen
                    #===============================================================================================
                    
                    
                case "STANNA":
                    #Stanna men inte avbryta all aktivitet
                    #UART
                    #send_UART(0,0)
                    print("Nu kör vi STANNA!")
                    
                    
                case "KONTROLL":
                    x_hinder_pos, y_hinder_pos = calc_lidar_cord(q_lidar.get(),q_mouse.get())
                    q_BT_transmitter.put((f"{x_hinder_pos}:{y_hinder_pos}"))	#Skicka som en array .put((...))
                    print("Nu kör vi KONTROLL!")
                    
                    x_current_pos, y_current_pos = q_mouse.get()
                    
                    kts_tid = split_mottaget[0].strip() + ":" + split_mottaget[1].strip() +":"+ split_mottaget[2].strip()
            
                    ed_tid = datetime.now()
                    ed_tid_str = ed_tid.strftime("%H:%M:%S")
            
                    ack = "ACK:"+kts_tid+":"+ed_tid+":"+meddelande_mottaget+":"+x_current_pos+":"+y_current_pos+":"+"A"
            
                    q_BT_transmitter.put(ack)	#Svara ÖS med ACK samt meddelandet som togs emot
                    
                    new_message = q_BT_reciever.get()
                    
                    split_new_message = new_message.split(":")
                    
                    old_ed_tid = split_new_message[0].strip() + ":" + split_new_message[1].strip() +":"+ split_new_message[2].strip()
                    
                    
                    if ed_tid == old_ed_tid:
                        #SKICKA TIL ÖS: HH:MM:SS:KONTROLL:XPOS:YPOS:STATUS 	,där status kan vara två fall: Arbetande(A), Vilande(V)
                        x_current_pos, y_current_pos = q_mouse.get()
                        x_vector, y_vector = calc_lidar_cord(q_lidar.get())
                        x_hinder_pos = [x_current_pos + x for x in x_vector]
                        y_hinder_pos = [y_current_pos + y for y in y_vector]
                        q_BT_transmitter.put((x_hinder_pos, y_hinder_pos))	#skicka till ös som en "tupel" (x,y) hamnar på plats 1 i kö, saknar tid och status!
                        q_BT_transmitter.put("hej från agv :|")
                        
                        tid = datetime.now()
                        tid_str = tid.strftime("%H:%M:%S")
                        q_BT_transmitter.put(f"{tid_str}:POSITION:{x_current_pos},{y_current_pos}")
                    print("Nu kör vi KONTROLL!")
                    
                case "FRAM":
                    #OBS tillfälligt kommmando, kör fram i 5-10 sek
                    #UART
                    send_UART("255,255")
                    print("Nu kör vi FRAM")
                    sleep_core_2(5)
                    send_UART("0,0")
                    print("STANNAD")
                    
                case "POSITION":
                    mouse_now = q_mouse.get()
                    x_current_pos, y_current_pos = calc_AGV_cord(mouse_now)
                    tid = datetime.now()
                    tid_str = tid.strftime("%H:%M:%S")
                    q_BT_transmitter.put(f"{tid_str}:POSITION:{x_current_pos},{y_current_pos}")
                    
                    
                case _:
                    #Om ett okänt kommando skrivs
                    message = "Okänt meddelande!"
                    #q_BT_transmitter.put(message)	#skicka till ös
                    print("Okänt kommando")
             
            sleep_core_2(0.2)
           
    except KeyboardInterrupt:
        print("Avsluytar")
      
#===================================Stoppa lidar===============================================
        p_lidar_stop = multiprocessing.Process(target=stop_lidar, args=(q_lidar_stop,))	#Starta lidar stop processen
        p_lidar_stop.start()	#starta funktionen
        p_lidar_stop.join()		#Avsluta processen
#===============================================================================================
          #Måste använda terminate() för finns while True i dessa funktioner
        p_mouse.terminate()
        p_lidar.terminate()
        p_BT_reciever.terminate()
        p_BT_transmitter.terminate()
        #p_ultraljud.terminate()
        p_BT_reciever.join()
        p_BT_transmitter.join()
        p_mouse.join()
        p_lidar.join()
        #p_ultraljud.join()
        #p_dwm.join()
if __name__ == '__main__':
        main()