"""
Reference to test GPIO in Raspberry Pi with Ubuntu 22.04 OS 
"""


import time
import lgpio

LED = 26

# open the gpio chip and set the LED pin as output
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, LED)

try:
    while True:
         # Turn the GPIO pin on
         lgpio.gpio_write(h, LED, 1)
         time.sleep(2)
         print("runnning")

         # Turn the GPIO pin off
         lgpio.gpio_write(h, LED, 0)
         time.sleep(1)
        

except KeyboardInterrupt:
     lgpio.gpio_write(h, LED, 0)
     lgpio.gpiochip_close(h)