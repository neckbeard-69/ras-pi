import RPi.GPIO as GPIO
import Adafruit_DHT
import time


GPIO.setmode(GPIO.BCM)
sensor = Adafruit_DHT.DHT11
GAS_PIN = 18 
TEMP_HUMIDITY_PIN = 23
GPIO.setup(GAS_PIN, GPIO.IN)

try:
    while True:
        gas_present = GPIO.input(GAS_PIN)
        humidity, temp = Adafruit_DHT.read_retry(sensor, TEMP_HUMIDITY_PIN)
        if humidity is not None and temp is not None:
            print(f"Temperature: {temp:.1f} °C, Humidity: {humidity:.1f} %")
        else:
            print("Failed to retrieve data from the DHT11 sensor.")
        if gas_present == GPIO.LOW:
            gas_state = "Gas Present"
        else:
            gas_state = "No Gas"

        print(f"Gas State: {gas_state}")

        time.sleep(2)  

except KeyboardInterrupt:
    print("Gas detection stopped by user")

finally:
    GPIO.cleanup()
