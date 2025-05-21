import os
import time
import multiprocessing
import math
import serial
from multiprocessing import Value
from multiprocessing import Process, Queue, Event
from Test_of_Mouse import update_mouse	#From "filnamn" import "funktion namn"
from Test_of_lidar import update_lidar
from Test_of_lidar import stop_lidar
from BT_rec import BT_reciever
from BT_tran import BT_transmitter
from datetime import datetime
from sleep_core import sleep_core_2
#from Ultraljud import update_ultraljud
#from DWM_positionering_async import dwm_setup, dwm_read





def setup_SERIAL():
    global ser
    ser = serial.Serial(
        port='/dev/ttyS0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )


setup_SERIAL()

#Nyvwersoionsds dw






def send_UART(input):
    ser.write(f"{input}\n".encode('UTF-8')) #skickar meddelande till esp
    
def read_UART():
    data = ser.readline().decode('UTF-8').strip() #tar emot meddelande från esp
    return data

def mouse_data():
    global data
    while not q_mouse.empty():
        data = q_mouse.get()
    #q_mouse.put(data)
    return data

def rotate_to(target_dir):
    global current_dir
    pause_event.set()
    while current_dir != target_dir:
        ser.write("128,-128\n".encode('UTF-8'))
        time.sleep(1.076)
        ser.write("0,0\n".encode('UTF-8'))
        time.sleep(2)
        current_dir = (current_dir + 1) % 4
    pause_event.clear()

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

    pause_event = Event()
    pause_event.clear()  # Start i "inte pausad"
    riktning = Value('i', 0)  # Startar i NORR (0)	#[0=Norr, 1=Ã–ST, 2=SÃ–DER, 3=VÃ„ST] till datormusen
    current_dir = 0	#AGV startar i NORR
    #skapa en kö för varje sensor
    q_BT_reciever = multiprocessing.Queue()
    q_BT_transmitter = multiprocessing.Queue()
    q_mouse = multiprocessing.Queue()
    q_lidar = multiprocessing.Queue()
    q_lidar_stop = multiprocessing.Queue()
    #q_ultraljud = multiprocessing.Queue()
    #q_DWM = multiprocessing.Queue()
    #stop_event = multiprocessing.Event()	#skapar ett event för nödstop


     
    print("startar prcesser")
    #starta processerna
    p_BT_reciever = multiprocessing.Process(target=BT_reciever, args=(q_BT_reciever,))
    p_BT_transmitter = multiprocessing.Process(target=BT_transmitter, args=(q_BT_transmitter,))
    p_mouse = multiprocessing.Process(target=update_mouse, args=(q_mouse,riktning,pause_event))
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
    
    
    
    
    
    
    try:
        
        while True:
            q_BT_transmitter.put(("datetime.now(),mottaget"))	#Svara ÖS med ACK samt meddelandet som togs emot
            mottaget = q_BT_reciever.get()
            split_mottaget = mottaget.split(":")
            meddelande_mottaget = split_mottaget[3].strip()
            print(f"Mottaget meddelande: {meddelande_mottaget}")
            q_BT_transmitter.put(("datetime.now(),mottaget"))	#Svara ÖS med ACK samt meddelandet som togs emot
            
            
            match meddelande_mottaget:
                case "KOR":
                    #KOR:X:Y:X:Y:X:...                    
                    x_hinder_pos, y_hinder_pos = calc_lidar_cord(q_lidar.get(),q_mouse.get())
                    q_BT_transmitter.put((f"{x_hinder_pos}:{y_hinder_pos}"))                    
                    
                case "START":
                    #START:X:Y:X:Y:X...                    
                    print("Nu kör vi START!")
                    nodstopp = False
                    int_array = [int(x) for x in split_mottaget[4:]]
                    print(int_array)
                    for i in range(0,len(int_array)-1,2):	#Går igenom alla koordinater skickat från ÖS börjar på i=4
                        current_mouse = mouse_data()
                        
                        #agv_angle = 0
                        print(i)
                        x_goal = int_array[i]
                        y_goal = int_array[i+1]
                        print(x_goal)
                        print(y_goal)
                        print(current_mouse)
                        if current_mouse[0] < x_goal+2 and current_mouse[0] > x_goal-2: #Kollar om vi redan står på x-axeln
                            print("IF")
                            if current_mouse[1] < y_goal:
                                #rotera till norr
                                rotate_to(0)
                                with riktning.get_lock():     #Ändra vilken riktning AGV står i som beräknas i mouse filen
                                    riktning.value = 0

                                lidar_data = q_lidar.get()
                                send_UART("50,50")

                                while current_mouse[1] < y_goal and lidar_data[0] > 20:
                                    current_mouse = mouse_data()
                                    lidar_data = q_lidar.get()
                                    print(f"{current_mouse}: riktning: {riktning}")
                                    
                                    time.sleep(0.1)

                                if lidar_data[0] < 20:
                                    send_UART("0,0")
                                    nodstopp = True
                                    print("KARIN SKRIKER: HINDER FRAMFÖR MIG, STANNA!!!")
                                    q_BT_transmitter.put(("FEL"))
                                
                                else:
                                    send_UART("0,0")
                                    print(f"FRAMME! Nuvarande koordinater: x={current_mouse[0]}, y={current_mouse[1]}")
                                    time.sleep(0.5)   


                            else:
                                #rotera till söder
                                
                                rotate_to(2)
                                with riktning.get_lock():      #Ändra vilken riktning AGV står i som beräknas i mouse filen
                                    riktning.value = 2
                                send_UART("50,50")
                                
                                while current_mouse[1] < y_goal and lidar_data[0] > 20:
                                    current_mouse = mouse_data()
                                    lidar_data = q_lidar.get() 
                                    time.sleep(0.1)
                                    
                                if lidar_data[0] < 20:
                                    send_UART("0,0")
                                    nodstopp = True
                                    print("KARIN SKRIKER: HINDER FRAMFÖR MIG, STANNA!!!")
                                    q_BT_transmitter.put(("FEL"))
                                
                                else:
                                    send_UART("0,0")
                                    print(f"FRAMME! Nuvarande koordinater: x={current_mouse[0]}, y={current_mouse[1]}")
                                    time.sleep(0.5)   

                        else:   #om vi redan står på rätt y-kord
                            print("ELSE")
                            if current_mouse[0] < x_goal:
                                #rotera till öst
                                rotate_to(1)
                                with riktning.get_lock():      #Ändra vilken riktning AGV står i som beräknas i mouse filen
                                    riktning.value = 1

                                lidar_data = q_lidar.get()
                                send_UART("50,50")
                                while current_mouse[0] < x_goal and lidar_data[0] > 20:
                                    current_mouse = mouse_data()
                                    lidar_data = q_lidar.get() 
                                    time.sleep(0.1)
                                    print(f"{current_mouse}: riktning: {riktning}")

                                if lidar_data[0] < 20:
                                    send_UART("0,0")
                                    nodstopp = True
                                    print("KARIN SKRIKER: HINDER FRAMFÖR MIG, STANNA!!!")
                                    q_BT_transmitter.put(("FEL"))
                                
                                else:
                                    send_UART("0,0")
                                    print(f"FRAMME! Nuvarande koordinater: x={current_mouse[0]}, y={current_mouse[1]}")
                                    time.sleep(0.5)  
                                

                            else:
                                #rotera till väst
                                rotate_to(3)
                                with riktning.get_lock():      #Ändra vilken riktning AGV står i som beräknas i mouse filen
                                    riktning.value = 3

                                print("VÄST")
                                send_UART("50,50")
                                lidar_data = q_lidar.get()
                                while current_mouse[0] < x_goal and lidar_data[0] > 20:
                                    current_mouse = mouse_data()
                                    print(f"{current_mouse}: riktning: {riktning}")
                                    
                                    time.sleep(0.1)
                                send_UART("0,0")
                                print(f"FRAMME! Nuvarande koordinater: x={current_mouse[0]}, y={current_mouse[1]}")
                                time.sleep(0.5)

                        if nodstopp == True:    #Gå till while True och invänta instruktioner från ÖS
                            break

                    
                case "HINDER":
                    #Skica endast lidar
                    mouse_now = q_mouse.get()
                    q_mouse.put(mouse_now)
                    x_hinder_pos, y_hinder_pos = calc_lidar_cord(q_lidar.get(),mouse_now)
                    x_current_pos, y_current_pos = mouse_now
                    q_BT_transmitter.put((f"HINDER:{x_hinder_pos}:{y_hinder_pos}:{x_current_pos},{y_current_pos}"))
                    print("Nu kör vi HINDER")
                    
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
        send_UART("0,0")
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
