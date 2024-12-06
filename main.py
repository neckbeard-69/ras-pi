
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

DO_PIN = 18 
GPIO.setup(DO_PIN, GPIO.IN)

try:
    while True:
        gas_present = GPIO.input(DO_PIN)

        if gas_present == GPIO.LOW:
            gas_state = "Gas Present"
        else:
            gas_state = "No Gas"

        print(f"Gas State: {gas_state}")

        time.sleep(0.5)  

except KeyboardInterrupt:
    print("Gas detection stopped by user")

finally:
    GPIO.cleanup()
