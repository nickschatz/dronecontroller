import network
import time
import uos

# webrepl pw "ente"

uos.dupterm(None, 1)  # Disable the REPL so that we can use uart

wlan = network.WLAN(network.AP_IF)
wlan.config(essid="Die Ente 0", authmode=network.AUTH_WPA_WPA2_PSK, password="vogel000")
wlan.active(True)       # activate the interface
