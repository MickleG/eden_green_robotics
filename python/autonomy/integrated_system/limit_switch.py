from gpiozero import Button

def check_bounds(button1, button2):
    if(button1.is_pressed or button2.is_pressed):
        return True
    else:
        return False
