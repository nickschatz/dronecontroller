import network
import webrepl
import time
import secure


webrepl.start()

# For debugging, connect to my home network
wlan = network.WLAN(network.STA_IF) # create station interface
wlan.active(True)       # activate the interface
wlan.connect(secure.SSID, secure.wifi_pass) # connect to an AP

wait = 30
count = 0
while not wlan.isconnected() and count < wait:
    print(".", end="")
    count += 1
    time.sleep(1)


print("IP: {}".format(wlan.ifconfig()[0]))
