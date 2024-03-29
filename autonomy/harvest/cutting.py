from gpiozero import OutputDevice

def cut():
    valve = OutputDevice(pin=16)
    valve.off()
    time.sleep(0.7)
    valve.on()

def open_scissors():
    valve = OutputDevice(pin=16)
    valve.off()

def close_scissors():
    valve = OutputDevice(pin=16)
    valve.on()
