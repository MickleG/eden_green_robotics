def check_bounds(button1, button2, button3, button4):
    if(button1.is_pressed or button2.is_pressed or button3.is_pressed or button4.is_pressed):
        return True
    else:
        return False
