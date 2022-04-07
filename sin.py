import time
import math
t = 0
amplitude = 0.15
omega = 1/5
while True:
    wave = amplitude * math.sin(t * omega) 
    print(wave)
    time.sleep(0.01)
    t+=1
