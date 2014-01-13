import time
import threading

def myfunc(i):
    print "sleeping 5 sec from thread %d" % i
    print "active threads: %d" % threading.active_count()
    time.sleep(5)
    print "finished sleeping from thread %d" % i

for i in range(10):
    t = threading.Thread(target=myfunc, args=(i,))
    t.start()