evt_file = open("/dev/input/event1", "rb")
cnt = 0
while True:
    evt = evt_file.read(16) # Read the event
    evt_file.read(16)       # Discard the debounce event 
    code = ord(evt[10])
    direction  = "down" if ord(evt[12]) else "up"
    if code == 1:
        print "Button " + direction
        if direction == "up":
            cnt = 0
	    print "Count " + str(cnt)
    if code == 2:
	if direction == "up":
            cnt = cnt + 1
            print "Count " + str(cnt)
