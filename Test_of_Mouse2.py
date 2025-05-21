import struct
from multiprocessing import Process
import os
import multiprocessing
from multiprocessing import Value
from multiprocessing import Process, Queue, Event
import time
from sleep_core import sleep_core_2

file = open("/dev/input/mice", "rb") 	#�ppnar filen i /dev/input/mice och l�ser den som bin�r "rb"



#Desta mindre scale_factor desto l�ngre m�ste agv:n �ka
#scale_factor = 0.0029975 #mystiska musen
#scale_factor = 0.00244648  #Lucas mus
#scale_factor = 0.002395209581  #Lucas mus p� AGV ---> 40 cm = 16 700	Scale_factor = 1/16700
scale_factor = 0.00225 #LUCAS OCH OSKAR M�TNING



#def update_mouse():
def update_mouse(queue,riktning,pause_event):
    os.sched_setaffinity(0, {1})

    paused_x = 0
    paused_y = 0
    was_paused = False
    
    #Alternativ_2
    result = [0, 0] #[x y] start koord
    queue.put(result)
    
    x_cm = 0
    y_cm = 0
    x = 0	#start koordinat
    y = 0	#start koordinat
    while True:
        if pause_event.is_set():
            if not was_paused:
                paused_x = x
                paused_y = y
                was_paused = True
            sleep_core_2(0.01)

        else:
            if was_paused:
                # När pausen slutar: kolla hur mycket som rört sig under pausen
                dx_during_pause = x - paused_x
                dy_during_pause = y - paused_y

                # Ta bort den rörelsen
                x -= dx_during_pause
                y -= dy_during_pause

                was_paused = False

            data = file.read(3)
            buttons, dx, dy = struct.unpack("bbb", data)

            if dx > 127:
                dx -= 256
            if dy > 127:
                dy -= 256

            current = riktning.value
            match current:
                case 0:
                    y += dy
                case 1:
                    x += dy
                case 2:
                    y -= dy
                case 3:
                    x -= dy

            x_cm = round(x * scale_factor)
            y_cm = round(y * scale_factor)

            result = x_cm, y_cm
            while not queue.empty():
                queue.get()
            queue.put(result)

            sleep_core_2(0.01)

        

"""
except KeyboardInterrupt:	#Avslutar, g�r ur while True om Ctrl + C
            print("Avslutar...")
            file.close()
"""

"""
while True:
    update_mouse()
"""