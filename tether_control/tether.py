import pygame, serial, os


def to2s(i):
    if i < 0:
        return i + (1 << 8) - 1
    return i

# set SDL to use the dummy NULL video driver, so it doesn't need a windowing system.
os.environ["SDL_VIDEODRIVER"] = "dummy"
# init pygame
pygame.init()
# create a 1x1 pixel screen, its not used so it doesnt matter
screen = pygame.display.set_mode((1, 1))
# init the joystick control

pygame.joystick.init()
_joystick = pygame.joystick.Joystick(0)
_joystick.init()

wport = serial.Serial("/dev/ttyACM1", 230400)
wport.timeout = 0
#wport.open()
# Main event loop
ready = False
while True:
    pygame.event.get()
    x = _joystick.get_axis(0)
    y = _joystick.get_axis(1)
    rx = _joystick.get_axis(2)
    ry = _joystick.get_axis(3)

    
    yaw = int(127 * x)
    pitch = int(127 * ry)
    roll = int(127 * rx)
    throttle = int(127 * y)
    
    if ready:
        data = b''.join(map(lambda d: d.to_bytes(1, "big", signed=True), [yaw, pitch, roll, throttle, 0]))
        assert len(data) == 5
        wport.write(data)

    resp = wport.readline().decode("U8")
    if resp:
        if resp == "READY\r\n":
            ready = True
        print(resp, end="")
        if ready:
            print("yaw: {}  pitch: {} roll: {} throttle: {}".format(yaw, pitch, roll, throttle))