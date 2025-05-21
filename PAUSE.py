import multiprocessing
import time
from multiprocessing import Value
from multiprocessing import Process, Queue, Event
from Test_of_Mouse import update_mouse	#From "filnamn" import "funktion namn"


def mouse_data():
    global data
    while not q_mouse.empty():
        data = q_mouse.get()
    q_mouse.put(data)
    return data

riktning = Value('i', 0) #agv startar i norr

pause_event = Event()
#pause_event = multiprocessing.Event()
pause_event.clear()  # Start i "inte pausad"

q_mouse = multiprocessing.Queue()
p_mouse = multiprocessing.Process(target=update_mouse, args=(q_mouse,riktning,pause_event))
p_mouse.start()


if __name__ == "__main__":
    


    time.sleep(3)
    print(mouse_data())
    print("Pausing...")
    pause_event.set()  # Pausar processen

    time.sleep(3)
    print("Resuming...")
    pause_event.clear()  # Fortsätter processen
    print(mouse_data())
    
    time.sleep(3)
    print(mouse_data())
    
    time.sleep(3)
    print(mouse_data())
    
    
    
    p_mouse.terminate()
    p_mouse.join()
