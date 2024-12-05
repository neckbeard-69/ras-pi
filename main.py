import RPi.GPIO as GPIO
import time
import smbus
import spidev  # For MCP3008 ADC (moisture sensor)
import adafruit_bme280  # For BME280 sensor

# GPIO Pin Setup
GPIO.setmode(GPIO.BCM)

# Sensor Pins
GAS_PIN = 17  # Digital Output pin of the gas sensor
PIR_PIN = 27  # PIR sensor pin
TRIG_PIN = 22  # Ultrasonic TRIG pin
ECHO_PIN = 23  # Ultrasonic ECHO pin
MOISTURE_PIN = 0  # Moisture sensor pin through MCP3008 (CH0)
BUZZER_PIN = 25  # Buzzer pin

# Pin Configurations
GPIO.setup(GAS_PIN, GPIO.IN)  # Digital input for gas sensor
GPIO.setup(PIR_PIN, GPIO.IN)  # Motion sensor
GPIO.setup(TRIG_PIN, GPIO.OUT)  # Ultrasonic sensor trigger
GPIO.setup(ECHO_PIN, GPIO.IN)  # Ultrasonic sensor echo
GPIO.setup(BUZZER_PIN, GPIO.OUT)  # Buzzer output

# I2C Setup for BME280 (Temperature Sensor)
i2c_bus = smbus.SMBus(1)  # 1 is the default I2C bus on the Pi
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c_bus)  # Initialize the BME280 sensor

# SPI Setup for MCP3008 (Moisture Sensor)
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device (CS) 0
spi.max_speed_hz = 1000000  # Set SPI speed (1MHz)

# Function to read analog value from MCP3008 (Moisture Sensor)
def read_adc(channel):
    if channel < 0 or channel > 7:
        return -1
    # Send the command to the MCP3008 to read the selected channel
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    # Combine the result and return the ADC value
    return ((adc[1] & 3) << 8) + adc[2]

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
    print "System initialized. Monitoring sensors..."
    while True:
        # Gas Sensor Alert (Digital Output)
        if GPIO.input(GAS_PIN):  # HIGH means gas is detected
            print "Gas detected!"
            buzz()
        
        # PIR Sensor Alert
        if GPIO.input(PIR_PIN):
            print "Motion detected!"
            buzz()
        
        # Ultrasonic Sensor Alert
        distance = ultrasonic_trigger()
        if distance < 10:  # Change threshold as required
            print "Object detected at {0:.2f} cm!".format(distance)
            buzz()
        
        # Moisture Sensor Alert
        moisture_level = read_adc(MOISTURE_PIN)
        print "Moisture Level: {}".format(moisture_level)
        if moisture_level < 300:  # Change threshold based on your sensor
            print "Moisture level critical!"
            buzz()
        
        # Temperature and Humidity from BME280
        temperature = bme280.temperature
        humidity = bme280.humidity
        if temperature and humidity:
            print "Temperature: {:.1f}°C, Humidity: {:.1f}%".format(temperature, humidity)
            if temperature > 30:  # Change threshold as required
                print "Warning! High Temperature Detected!"
                buzz()
        else:
            print "Failed to read from the temperature sensor."
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print "Exiting program..."
finally:
    GPIO.cleanup()
