from dynamixel_sdk import *
import time
sys.path.append("autonomy/gripping")

from open_grip import open_grip
from close_grip import close_grip

open_grip()
time.sleep(5)
close_grip()

