import struct
from multiprocessing import Process
import os
import multiprocessing
import time
import math
from sleep_core import sleep_core_2

file = open("/dev/input/mice", "rb") 	#Öppnar filen i /dev/input/mice och läser den som binär "rb"


#Desta mindre scale_factor desto längre måste agv:n åka
#scale_factor = 0.0029975 #mystiska musen
#scale_factor = 0.00244648  #Lucas mus
scale_factor = 0.002395209581  #Lucas mus på AGV ---> 40 cm = 16 700	Scale_factor = 1/16700
#scale_factor = 1 #LUCAS OCH OSKAR MÄTNING
#scale_factor = 0.00249588153386 #Gustav L mätning 

#Test 1 lång i banan med Gustav L: 127504 pixlar på 315cm
#Test 2 lång i banan med Gustav L: 124503 pixlar på 312cm
#Test 3 lång i banan med Gustav L: 123276 pixlar på 309,5cm





#def update_mouse():
def update_mouse(queue,riktning,pause_event):
    os.sched_setaffinity(0, {1})
    
    #Alternativ_2
    result = [0, 0] #[x y] start koord
    queue.put(result)
    
    x_cm = 0
    y_cm = 0
    x = 0	#start koordinat
    y = 0	#start koordinat
    first = True
    while True: #En loop som körs för evigt för vi läser datamusens info för evigt
        if not pause_event.is_set():
            #print("Inside loop")
            
            if not first:
                data = file.read(3) #Läser 3 bytes från file 1 = knapptryck, 2 = x_rörelse, 3 = y_rörelse
                buttons, dx, dy = struct.unpack("bbb",data) #Omvandlar binärdata från data till tre int "bbb" = (button, dx, dy)
               
            else:
               
                buttons = 0
                dx = 0
                dy = 0
                first = False
            
            #print("Binary data read done")
        #Om det är negativ rörelse så kommer det ge positvt dx > 127, måste gör den negativ med -256
            if dx > 127: # Det är ett negativt tal
                dx -=256
                
            if dy > 127:
                dy -= 256

            current = riktning.value
            match current:
                case 0:
                    y +=dy

                case 90:
                    x +=dy

                case 180:
                    y -=dy

                case 270:
                    x -=dy
        #=================================lägg till 4 if satser för varje vädersträck====================================
            #Uppdatera koordinater
            #x +=dx
            #y +=dy
        #================================================================================================================
            x_cm = x*scale_factor +40
            y_cm = y*scale_factor +40
            
            #x_cm = f"{x_cm:.10f}"[:5] #skriver endast ut 5 tecken (t.ex 3.xxx) på x_cm utan att avrunda notera att den endast skriver ut 2 värdesiffror för negativa tal!
            #y_cm = f"{y_cm:.10f}"[:5]
            
            x_cm = round(x_cm)
            y_cm = round(y_cm)
            
            result = x_cm, y_cm	#Alternativ_2
            while not queue.empty():	#Resna kön innan lägger in nytt värde
                queue.get()
            
            queue.put(result)	#Lägg till listan i kön
            sleep_core_2(0.01)
            
            #print(result)

        else:
            data = file.read(3) #Läser 3 bytes från file 1 = knapptryck, 2 = x_rörelse, 3 = y_rörelse
            buttons, dx, dy = struct.unpack("bbb",data) #Omvandlar binärdata från data till tre int "bbb" = (button, dx, dy)
            dx = 0
            dy = 0
            sleep_core_2(0.01)  # Sov kort medan pausad

        

"""
except KeyboardInterrupt:	#Avslutar, går ur while True om Ctrl + C
            print("Avslutar...")
            file.close()
"""

"""
while True:
    update_mouse()
"""