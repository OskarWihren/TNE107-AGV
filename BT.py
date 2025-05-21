import multiprocessing

def BT(queue):
    os.sched_setaffinity(0, {2})	#Kör på kärna 2