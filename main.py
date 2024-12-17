import RPi.GPIO as GPIO
import Adafruit_DHT
import time
import threading

GPIO.setmode(GPIO.BCM)

sensor = Adafruit_DHT.DHT11

PULSE = 0.00001  # 10 microseconds
GAS_PIN = 18
TEMP_HUMIDITY_PIN = 23
PIR_PIN = 12
TRIGGER_PIN = 5
ECHO_PIN = 6

DISTANCE_BUZZ_PIN = 22
GAS_BUZZ_PIN = 17
MOTION_BUZZ_PIN = 27
TEMP_HUMIDITY_BUZZ_PIN = 4

GAS_FREQ = 300
DISTANCE_FREQ = 500
MOTION_FREQ = 700
TEMP_FREQ = 1000
HUMIDITY_FREQ = 1500 

GPIO.setup(GAS_PIN, GPIO.IN)
GPIO.setup(PIR_PIN, GPIO.IN)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(TRIGGER_PIN, GPIO.OUT)

GPIO.setup(DISTANCE_BUZZ_PIN, GPIO.OUT)
GPIO.setup(GAS_BUZZ_PIN, GPIO.OUT)
GPIO.setup(MOTION_BUZZ_PIN, GPIO.OUT)
GPIO.setup(TEMP_HUMIDITY_BUZZ_PIN, GPIO.OUT)

def buzz(buzz_pin, frequency, duration=1):
    pwm = GPIO.PWM(buzz_pin, frequency)
    pwm.start(50)  
    time.sleep(duration)
    pwm.stop()

def detect_motion():
    while True:
        if GPIO.input(PIR_PIN) == GPIO.HIGH:
            print("Motion detected")
            buzz(MOTION_BUZZ_PIN, MOTION_FREQ)
        time.sleep(0.1)  # Avoid rapid buzzing

def detect_gas():
    while True:
        if GPIO.input(GAS_PIN) == GPIO.LOW:
            print("Gas detected")
            buzz(GAS_BUZZ_PIN, GAS_FREQ)
        time.sleep(0.1)

def detect_distance():
    while True:
        GPIO.output(TRIGGER_PIN, GPIO.HIGH)
        time.sleep(PULSE)
        GPIO.output(TRIGGER_PIN, GPIO.LOW)
        
        pulse_start, pulse_end = 0, 0
        while GPIO.input(ECHO_PIN) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO_PIN) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 34300 / 2
        print(f"Distance: {distance:.2f} cm")
        
        if distance > 20:
            buzz(DISTANCE_BUZZ_PIN, DISTANCE_FREQ)
            print("High distance")
        time.sleep(0.5)

def detect_temp_humidity():
    while True:
        humidity, temp = Adafruit_DHT.read(sensor, TEMP_HUMIDITY_PIN)
        
        if temp is not None and temp >= 50:
            print("Temperature is greater than 50°C")
            buzz(TEMP_HUMIDITY_BUZZ_PIN, TEMP_FREQ)
        
        if humidity is not None:
            print("Humidity detected")
            buzz(TEMP_HUMIDITY_BUZZ_PIN, HUMIDITY_FREQ)
        
        time.sleep(2) 

try:
    motion_thread = threading.Thread(target=detect_motion)
    gas_thread = threading.Thread(target=detect_gas)
    distance_thread = threading.Thread(target=detect_distance)
    temp_humidity_thread = threading.Thread(target=detect_temp_humidity)

    motion_thread.start()
    gas_thread.start()
    distance_thread.start()
    temp_humidity_thread.start()

    motion_thread.join()
    gas_thread.join()
    distance_thread.join()
    temp_humidity_thread.join()

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    GPIO.cleanup()
