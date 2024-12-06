import RPi.GPIO as GPIO
import Adafruit_DHT
import time
GPIO.setmode(GPIO.BCM)
sensor = Adafruit_DHT.DHT11

PULSE = 0.00001 # 10 micro second
GAS_PIN = 18 
TEMP_HUMIDITY_PIN = 23
PIR_PIN = 12
TRIGGER_PIN = 5
ECHO_PIN = 6
BUZZ_PIN = 22

GPIO.setup(GAS_PIN, GPIO.IN)
GPIO.setup(PIR_PIN, GPIO.IN)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(BUZZ_PIN, GPIO.OUT)
GAS_FREQ = 261
DISTANCE_FREQ = 196
MOTION_FREQ = 220
TEMP_FREQ = 247
HUMIDITY_FREQ = 300
def buzz(frequency):
    pwm = GPIO.PWM(BUZZ_PIN, frequency)
    pwm.start(50)
    time.sleep(1)
    pwm.stop()

def detect_motion():
    pir_value = GPIO.input(PIR_PIN)
    if pir_value == GPIO.HIGH:
        print("motion detected")
        buzz(MOTION_FREQ)
        return
    print("no motion detected")

def detect_temp_humidity():
    humidity, temp = Adafruit_DHT.read(sensor, TEMP_HUMIDITY_PIN)

    if temp is not None and temp >= 50:
        print("temp is greator than 50")
        buzz(TEMP_FREQ)
    if humidity is not None:
        print("himidity is detected")
        buzz(HUMIDITY_FREQ)

def detect_gas():
    gas_present = GPIO.input(GAS_PIN)
    if gas_present == GPIO.LOW:
        print("gas detected")
        buzz(GAS_FREQ)
        return
    print("no gas detected")

def detect_distance():
    GPIO.output(TRIGGER_PIN, GPIO.HIGH)
    time.sleep(PULSE)
    GPIO.output(TRIGGER_PIN, GPIO.LOW)
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 34300 / 2
    print(f"Distance: {distance:.2f} cm")
    if distance > 20:
        buzz(DISTANCE_FREQ)
        print("High distance")

try:
    while True:
        detect_motion()
        detect_temp_humidity()
        detect_gas()
        detect_distance()
        time.sleep(2)  

finally:
    GPIO.cleanup()
