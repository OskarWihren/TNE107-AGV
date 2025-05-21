from datetime import datetime,timedelta
import os
import time

def sleep_core_2(t):
    os.sched_setaffinity(0,{2})	#kör på kärna 2
    time.sleep(t)
    #slut_tid = datetime.now() + timedelta(seconds=t)
    #while datetime.now() < slut_tid:
        #time.sleep