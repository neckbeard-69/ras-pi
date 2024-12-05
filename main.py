import RPi.GPIO as GPIO
import time
import smbus2
import spidev  # For MCP3008 ADC (moisture sensor)
import busio
import board
from adafruit_bme280 import basic as adafruit_bme280  # Updated import

# GPIO Pin Setup
GPIO.setmode(GPIO.BCM)

# Sensor Pins
GAS_PIN = 17  # Digital Output pin of the gas sensor
PIR_PIN = 27  # PIR sensor pin
TRIG_PIN = 22  # Ultrasonic TRIG pin
ECHO_PIN = 23  # Ultrasonic ECHO pin
MOISTURE_PIN = 5  # Moisture sensor digital output pin (DO)
BUZZER_PIN = 25  # Buzzer pin

# Pin Configurations
GPIO.setup(GAS_PIN, GPIO.IN)  # Digital input for gas sensor
GPIO.setup(PIR_PIN, GPIO.IN)  # Motion sensor
GPIO.setup(TRIG_PIN, GPIO.OUT)  # Ultrasonic sensor trigger
GPIO.setup(ECHO_PIN, GPIO.IN)  # Ultrasonic sensor echo
GPIO.setup(MOISTURE_PIN, GPIO.IN)  # Digital input for moisture sensor
GPIO.setup(BUZZER_PIN, GPIO.OUT)  # Buzzer output

# I2C Setup for BME280 (Temperature Sensor)
i2c = busio.I2C(board.SCL, board.SDA)
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

# SPI Setup for MCP3008 (Moisture Sensor) is no longer needed as we are using digital input

# Function to sound the buzzer
def buzz():
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(BUZZER_PIN, GPIO.LOW)

# Function to trigger ultrasonic sensor and calculate distance
def ultrasonic_trigger():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    start_time = time.time()
    stop_time = time.time()
    
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()
    
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()
    
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2
    return distance

# Main loop
try:
    print("System initialized. Monitoring sensors...")
    while True:
        # Gas Sensor Alert (Digital Output)
        if GPIO.input(GAS_PIN):  # HIGH means gas is detected
            print("Gas detected!")
            buzz()
        
        # PIR Sensor Alert
        if GPIO.input(PIR_PIN):
            print("Motion detected!")
            buzz()
        
        # Ultrasonic Sensor Alert
        distance = ultrasonic_trigger()
        if distance < 10:  # Change threshold as required
            print(f"Object detected at {distance:.2f} cm!")
            buzz()
        
        # Moisture Sensor Alert (Digital Signal)
        if GPIO.input(MOISTURE_PIN):  # HIGH means dry soil
            print("Moisture level is normal!")
        else:  # LOW means moisture is critical
            print("Moisture level critical!")
            buzz()
        
        # Temperature and Humidity from BME280
        temperature = bme280.temperature
        humidity = bme280.humidity
        if temperature and humidity:
            print(f"Temperature: {temperature:.1f}°C, Humidity: {humidity:.1f}%")
            if temperature > 30:  # Change threshold as required
                print("Warning! High Temperature Detected!")
                buzz()
        else:
            print("Failed to read from the temperature sensor.")
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Exiting program...")
finally:
    GPIO.cleanup()
