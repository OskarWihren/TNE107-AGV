import bluetooth
import time
import multiprocessing
import os
import RPi.GPIO as GPIO #GPIO biblioteket

bluetooth_pin = 26  #Blå sladd och lampa
GPIO.setwarnings(False) #för att ta bort varningarna

#ställ in gpio läge till bcm (broadcom pin number)
GPIO.setmode(GPIO.BCM)
GPIO.setup(bluetooth_pin, GPIO.OUT)

def BT_reciever(queue):
    os.sched_setaffinity(0, {2})
    while True:
        # skapar server
        server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        port = 2
        server_sock.bind(("",port))

        server_sock.listen(port)

        print("Väntar på anslutning",port)



        try:
            
        #     info om klient som anslutit till servern
            client_sock, client_info = server_sock.accept()
        #     with print_lock:
            print("Accepterat anslutning från reciever ",client_info)
            GPIO.output(bluetooth_pin,True)
            
            while True:
        #         läser data från klient
                
                
                data = client_sock.recv(1024)
                if not data:
                    break
                #print("Mottaget meddelande: ",data.decode('utf-8'))
                queue.put(data.decode('utf-8'))	#Eller ska man dela upp den i vektor Fram:x:y	--> ["Fram", x, y]???
                #print(data.decode('utf-8'))
                #time.sleep(1) #kanske behövs
                
        except bluetooth.btcommon.BluetoothError as e:
            print("bluetooth-fel:", e)
            GPIO.output(bluetooth_pin,False)

        finally:
            #client_sock.close()
            server_sock.close()
            print("Reciever stängd")
            GPIO.output(bluetooth_pin,False)
            time.sleep(5)
    
