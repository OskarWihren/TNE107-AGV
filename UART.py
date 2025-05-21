import serial
import time
import multiprocessing
from Test_of_Mouse import update_mouse	#From "filnamn" import "funktion namn"

def update_kord_system(direction):
    if direction == NORR:
        

def mouse_data():
    data = q_mouse.get()
    q_mouse.put(data)
    return data
def check_x_axis(mouse_x,target_x):
    x_axis = False
    
    if mouse_x == target_x:
        x_axis = True
        
    return x_axis
    

def rotate90right():
    ser.write("255,-255\n".encode('UTF-8'))
    time.sleep(1.078)
    ser.write("0,0\n".encode('UTF-8'))
    time.sleep(0.1)
    

ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
riktning = 0	#[0=Norr, 1=ÖST, 2=SÖDER, 3=VÄST]

q_mouse = multiprocessing.Queue()
p_mouse = multiprocessing.Process(target=update_mouse, args=(q_mouse,))
p_mouse.start()

try:
    
    while True:
        message = input("Skriv ditt meddelande: ") #skriv in meddelande till esp
        #ser.write(f"{message}\n".encode('UTF-8')) #skickar meddelande till esp
        target_kord = [0,10]	#[x,y]
        
        #if ser.in_waiting > 0:
        #data = ser.readline().decode('UTF-8').strip() #tar emot meddelande från esp
        #print(f"Mottaget: {data}") #skriver ut mottaget meddelande från esp
        
        match message:
            
            #	     VÄST
            #	Söder	 NORR
            #		 ÖST
            
            case "TEST":
                #Spara alla koordinater i en vector = [x1:y1:x2:y2:x3:y3:....]
                #Gör en for loop som går igenom varje koordinat
                for i in range(len(target_kord)):
                
                    mouse_x, mouse_y = mouse_data()
                    if check_x_axis(mouse_x,target_kord[i]):	#Om vi ligger på rätt y koordinat
                        if mouse_x < target_kord[i]:
                            #Kör fram med while
                            #Rotera så vi pekar NORR, ändra koordinatsystemetså musen uppdaterar rätt?????????
                            direction = NORR
                            
                            
                            
                            while mouse_x != target_kord[i]:
                                ser.write("100,100\n".encode('UTF-8'))
                                mouse_x, mouse_y = mouse_data()
                            ser.write("0,0\n".encode('UTF-8'))
                            
                            
                        else:
                            #rotera så vi pekar SÖDER, ändra koordinatsystemet så musen uppdaterar rätt?????????
                            direction = SODER
                            #kör rakt fram med while
                            while mouse_x != target_kord[i]:
                                ser.write("100,100\n".encode('UTF-8'))
                                mouse_x, mouse_y = mouse_data()
                            ser.write("0,0\n".encode('UTF-8'))
                            
                    else:	#Om vi ligger i rätt y-axel
                        if mouse_y < target_kord[i+1]:
                            #rotera så vi pekar VÄST
                            direction = VAST
                            #Kör rakt fram med while
                            while mouse_y != target_kord[i+1]:
                                ser.write("100,100\n".encode('UTF-8'))
                                mouse_x, mouse_y = mouse_data()
                            ser.write("0,0\n".encode('UTF-8'))
                            
                        else:
                            #rotera så vi pekar ÖST, ändra koordinatsystemet så musen uppdaterar rätt?????????
                            direction = OST
                            
                            #kör rakt fram med while
                            while mouse_y != target_kord[i+1]:
                                ser.write("100,100\n".encode('UTF-8'))
                                mouse_x, mouse_y = mouse_data()
                            ser.write("0,0\n".encode('UTF-8'))
                        
                    


            case "STOPP":
                print("STOPP")
                ser.write("0,0\n".encode('UTF-8'))
            
            case "FRAM":
                print("FRAM")
                ser.write("100,100\n".encode('UTF-8'))
                time.sleep(1)
                ser.write("0,0\n".encode('UTF-8'))
            
            case "BAK":
                print("BAK")
                ser.write("-200,-200\n".encode('UTF-8'))
                time.sleep(1)
                ser.write("0,0\n".encode('UTF-8'))
                
                
            case "NORR":
                while riktning != 0:
                    print("ROTERAR")
                    rotate90right()
                    riktning +=1
                    
                    if riktning > 3:
                        riktning = 0
                        
                    time.sleep(1)
                print("NORR")
                

                
            case "ÖST":
                while riktning != 1:
                    print("ROTERAR")
                    rotate90right()
                    riktning +=1
                    
                    if riktning > 3:
                        riktning = 0
                print("ÖST")
                
                
            case "SÖDER":
                while riktning != 2:
                    
                    print("ROTERAR")
                    rotate90right()
                    riktning +=1
                    
                    if riktning > 3:
                        riktning = 0
                print("SÖDER")
                
                
                
            case "VÄST":
                while riktning != 3:
                    print("ROTERAR")
                    rotate90right()
                    riktning +=1
                    
                    if riktning > 3:
                        riktning = 0
                print("VÄST")
                
                
            case "CHECK":
                data = mouse_data()
                print(f"{data}")
                
            case _:
                #Om ett okänt kommando skrivs
                print("Okänt kommando")
                
        time.sleep(0.2)
                
                
except KeyboardInterrupt:
    print("Avslyutar")
    p_mouse.terminate()
    p_mouse.join()