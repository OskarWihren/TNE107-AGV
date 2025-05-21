import RPi.GPIO as GPIO #GPIO biblioteket
import time
import serial
import os
from sleep_core import sleep_core_2

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
    ser.write("120,120\n".encode('UTF-8'))

def AGV_STOP():
    ser.write("0,0\n".encode('UTF-8'))
    sleep_core_2(0.05)
    
def AGV_BAK():
    ser.write("-80,-80\n".encode('UTF-8'))

def AGV_Right():
    ser.write("-150,150\n".encode('UTF-8'))

def AGV_Left():
    ser.write("150,-150\n".encode('UTF-8'))

def AGV_SLOW_RIGHT():
    ser.write("-50,50\n".encode('UTF-8'))

def AGV_SLOW_LEFT():
    ser.write("50,-50\n".encode('UTF-8'))

def AGV_line_left():
    ser.write("100,0\n".encode('UTF-8'))

def AGV_line_right():
    ser.write("0,100\n".encode('UTF-8'))

def setup_linjefoljare():
    global left_linjefoljare
    global right_linjefoljare
    global lamp_pin
    #GPIO.setwarnings(False) #för att ta bort varningarna
    GPIO.setmode(GPIO.BCM)

    left_linjefoljare = 24
    right_linjefoljare = 23
    lamp_pin = 25

    GPIO.setup(left_linjefoljare, GPIO.IN) #konfigurera gpio 23 som input, left linjeföljare
    GPIO.setup(right_linjefoljare, GPIO.IN) #konfigurera gpio 24 som input, right linjeföljare
    GPIO.setup(lamp_pin, GPIO.IN) #konfigurera gpio 25 som input, lamp_pin

#HIGH när den ser tejpen


def rotate_to(target_dir,q_IMU):
    #pause_event.set()
    current_dir = q_IMU.get()
    diff = target_dir - current_dir
    print(current_dir)
    # with riktning.get_lock():      # Skickar vilken riktining AGV har till datormusen
    #     riktning.value = target_dir
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

ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
) 
    #======================================================================================





"""
while True:
    print("HÖGER",GPIO.input(right_linjefoljare))
    print("Vänster",GPIO.input(left_linjefoljare))
    time.sleep(0.5)

"""







def start_linjefoljare(q_IMU, AGV_riktning):
    os.sched_setaffinity(0, {2})
    threash = 5
    klar = False

    setup_ultra()
    ultraljud = get_ultraljud()
    setup_linjefoljare()
    AGV_Right()
    
    time_start = time.time()
    
    time_final = 0
    while GPIO.input(left_linjefoljare) == 0 and time.time()-time_start < 10: #medan left inte ser tejpen
        time_final = time.time()-time_start
        
    print("FINAL TIME:",time_final)
    AGV_STOP()
    if(time_final >= 9.9):
        print("Roterar till: ",AGV_riktning)
        rotate_to(AGV_riktning, q_IMU)
        #
        time.sleep(0.3)
        AGV_FRAM()
        print("KÖR FRAMÅT")
        time.sleep(0.8)
        print("STOPPAR")
        AGV_STOP()
    time.sleep(0.1)


    if GPIO.input(left_linjefoljare) == 0:
        AGV_Right()
        while GPIO.input(left_linjefoljare) == 0:
            pass
        AGV_STOP()
    
    
    
    while ultraljud > threash and not klar:
       
        AGV_line_left()
        print("SVÄNGER VÄNSTER")
        while GPIO.input(right_linjefoljare) == 0:
            pass

        while GPIO.input(right_linjefoljare) == 1 and GPIO.input(left_linjefoljare) == 1:
            pass
            
            
        AGV_STOP()
        print("STANNAR")
        time.sleep(0.01)
        # if GPIO.input(lamp_pin) == 1:
        #     ultraljud = get_ultraljud()
        #     klar = True
        #     break
        ultraljud = get_ultraljud()
        print(ultraljud)

        if ultraljud < threash:
            print("AVSTÅND LITET, avstånd:",ultraljud)
            break
        
        AGV_line_right()
        print("SVÄNGER HÖGER")
        while GPIO.input(left_linjefoljare) == 0:
            pass

        while GPIO.input(right_linjefoljare) == 1 and GPIO.input(left_linjefoljare) == 1:
            pass

        AGV_STOP()
        print("STANNAR")
        # if GPIO.input(lamp_pin) == 1:
        #     ultraljud = get_ultraljud()
        #     klar = True
        #     break
        ultraljud = get_ultraljud()
        if ultraljud < threash and not klar:
            print("AVSTÅND LITET, avstånd:",ultraljud)
            break
        

    if not klar:
        while not klar:
            AGV_Left()
            while GPIO.input(right_linjefoljare) == 0 and GPIO.input(lamp_pin) == 0:
                pass
            AGV_STOP()
            if GPIO.input(lamp_pin) == 1:
                    klar = True
                    break
            time.sleep(0.01)
            AGV_Right()
            while GPIO.input(left_linjefoljare) == 0 and GPIO.input(lamp_pin) == 0:
                pass
            AGV_STOP()
            if GPIO.input(lamp_pin) == 1:
                    klar = True
                    break
            time.sleep(0.01)

    if klar:
        print("Lyckad hämtning!")
        return ultraljud #SKICKA AVSTÅNDET TILL BESÖKSPLATS, FÖR KALIBRERING AV POSITION


    AGV_STOP()

