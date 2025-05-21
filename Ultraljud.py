import RPi.GPIO as GPIO #GPIO biblioteket
import time
from multiprocessing import Process
import os
import multiprocessing
from sleep_core import sleep_core_2


def update_ultraljud(queue):
                     
    #os.sched_setaffinity(0, {2})

    GPIO.setwarnings(False) #för att ta bort varningarna

        #ställ in gpio läge till bcm (broadcom pin number)
    GPIO.setmode(GPIO.BCM)

    trigg_pin = 17
    echo_pin = 27

    hastighet_ljud = 34300

    GPIO.setup(trigg_pin, GPIO.OUT) #konfigurera gpio 17 som input, trigg

    GPIO.setup(echo_pin, GPIO.IN) #konfigurera gpio 27 som output, echo

    GPIO.output(trigg_pin,False)

    sleep_core_2(0.5)


    while True:
        
        GPIO.output(trigg_pin,True)

        sleep_core_2(0.00001) #10 microsekunder

        
        GPIO.output(trigg_pin,False)
        
        while GPIO.input(echo_pin) == 0:
            #print("hej")
            pass
            
        start_time = time.time()
            
        while GPIO.input(echo_pin) == 1:
            pass
        stop_time = time.time()
        
        res_time = stop_time - start_time
        
        distans = round((res_time*hastighet_ljud)/2,2) #avrunda till 2 decimaler
        
        while not queue.empty():	#Resna kön innan lägger in nytt värde
            queue.get()
        
        queue.put(distans)
        #print(distans)
        
        sleep_core_2(0.2)


