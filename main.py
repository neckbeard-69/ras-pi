import RPi.GPIO as GPIO
import Adafruit_DHT
import time


GPIO.setmode(GPIO.BCM)
sensor = Adafruit_DHT.DHT11
GAS_PIN = 18 
TEMP_HUMIDITY_PIN = 23
PIR_PIN = 12
GPIO.setup(GAS_PIN, GPIO.IN)
GPIO.setup(PIR_PIN, GPIO.IN)
def detect_motion():
    pir_value = GPIO.input(PIR_PIN)
    if pir_value == GPIO.HIGH:
        print("motion detected")
        return
    print("no motion detected")

def detect_temp_humidity():
    humidity, temp = Adafruit_DHT.read(sensor, TEMP_HUMIDITY_PIN)

    if humidity is not None and temp is not None:
        print(f"Temperature: {temp:.1f} °C, Humidity: {humidity:.1f} %")
        return
    print("Failed to retrieve data from the DHT11 sensor.")

def detect_gas():
    gas_present = GPIO.input(GAS_PIN)
    if gas_present == GPIO.LOW:
        print("gas detected")
        return
    print("no gas detected")

try:
    while True:
        detect_motion()
        detect_temp_humidity()
        detect_gas()
        time.sleep(2)  

except KeyboardInterrupt:
    print("Gas detection stopped by user")

finally:
    GPIO.cleanup()
