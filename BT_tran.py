import bluetooth
import time
import os
import multiprocessing
from sleep_core import sleep_core_2


def get_sensor_data():
    counter = 1
    
    current_time = time.strftime("%H:%M:%S:", time.localtime())
    x_led=24
    y_led=8
    #return f"{current_time}POSITION:{x_led},{y_led}"
    #return f"{current_time} Koordinater: x={x_led}, y={y_led}"
    return f"{current_time}FARDIG:{x_led},{y_led}:"

def BT_transmitter(queue):
    os.sched_setaffinity(0, {2})
    while True:
        server_address = "D4:D2:52:ED:2D:43"
        port = 1
        

        print(f"Försöker ansluta till {server_address}:{port}")
            
        client_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        try:
            client_sock.connect((server_address, port))
            print("Ansluten till mottagaren trasnsmitter")
            
            
            while True:
                if not queue.empty():
                    sensor_data = queue.get()
                    print("SKICKAR, NÅGOT===========================================================:", sensor_data)
                    client_sock.sendall(sensor_data.encode('utf-8'))#Skicka data via BT

                	
                
                sleep_core_2(0.1)

            
        except bluetooth.btcommon.BluetoothError as e:
            print("bluetooth-fel:", e)
            

        
        finally:
            client_sock.close()
            print("Transmitter stängd")
            
            time.sleep(5)
