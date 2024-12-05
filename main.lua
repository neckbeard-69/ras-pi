-- Load the required libraries
local gpio = require("wiringpi")
local i2c = require("i2c")

-- GPIO Pin Setup
local GAS_PIN = 17 -- Digital Output pin of the gas sensor
local PIR_PIN = 27 -- PIR sensor pin
local TRIG_PIN = 22 -- Ultrasonic TRIG pin
local ECHO_PIN = 23 -- Ultrasonic ECHO pin
local MOISTURE_PIN = 5 -- Moisture sensor digital output pin (DO)
local BUZZER_PIN = 25 -- Buzzer pin

-- Setup WiringPi
gpio.wiringPiSetupGpio()

-- Set pin modes
gpio.pinMode(GAS_PIN, gpio.INPUT) -- Gas sensor
gpio.pinMode(PIR_PIN, gpio.INPUT) -- PIR sensor
gpio.pinMode(TRIG_PIN, gpio.OUTPUT) -- Ultrasonic TRIG pin
gpio.pinMode(ECHO_PIN, gpio.INPUT) -- Ultrasonic ECHO pin
gpio.pinMode(MOISTURE_PIN, gpio.INPUT) -- Moisture sensor
gpio.pinMode(BUZZER_PIN, gpio.OUTPUT) -- Buzzer

-- I2C setup for BME280
local bus = i2c.open(1, 0x76) -- 0x76 is the default I2C address for BME280

-- Function to initialize the BME280 sensor (setup)
local function init_bme280()
	-- Initialize the BME280 sensor (write necessary registers to configure the sensor)
	bus:write(0xF2, 0x01) -- Humidity oversampling x1
	bus:write(0xF4, 0x27) -- Control humidity, pressure, and temperature measurement
	bus:write(0xF5, 0xA0) -- Configuration for standby time and filter
end

-- Function to read BME280 sensor values (temperature and humidity)
local function read_bme280()
	local temp_data = bus:read(0xFA, 3)
	local hum_data = bus:read(0xFD, 2)

	-- Convert data into temperature and humidity values (you may need to adjust for calibration)
	local temperature = ((temp_data[1] * 256) + temp_data[2]) / 100
	local humidity = ((hum_data[1] * 256) + hum_data[2]) / 100
	return temperature, humidity
end

-- Function to sound the buzzer
local function buzz()
	gpio.digitalWrite(BUZZER_PIN, gpio.HIGH)
	os.execute("sleep 0.5") -- Sleep for 0.5 seconds
	gpio.digitalWrite(BUZZER_PIN, gpio.LOW)
end

-- Function to trigger ultrasonic sensor and calculate distance
local function ultrasonic_trigger()
	gpio.digitalWrite(TRIG_PIN, gpio.HIGH)
	os.execute("sleep 0.00001") -- Trigger pulse
	gpio.digitalWrite(TRIG_PIN, gpio.LOW)

	local start_time = os.time()
	while gpio.digitalRead(ECHO_PIN) == 0 do
		start_time = os.time()
	end
	local stop_time = os.time()
	while gpio.digitalRead(ECHO_PIN) == 1 do
		stop_time = os.time()
	end

	local elapsed_time = stop_time - start_time
	local distance = (elapsed_time * 34300) / 2 -- Calculate distance in cm
	return distance
end

-- Main loop
init_bme280()

while true do
	-- Gas Sensor Alert (Digital Output)
	if gpio.digitalRead(GAS_PIN) == gpio.HIGH then
		print("Gas detected!")
		buzz()
	end

	-- PIR Sensor Alert
	if gpio.digitalRead(PIR_PIN) == gpio.HIGH then
		print("Motion detected!")
		buzz()
	end

	-- Ultrasonic Sensor Alert
	local distance = ultrasonic_trigger()
	if distance < 10 then -- Change threshold as required
		print("Object detected at " .. string.format("%.2f", distance) .. " cm!")
		buzz()
	end

	-- Moisture Sensor Alert (Digital Signal)
	if gpio.digitalRead(MOISTURE_PIN) == gpio.HIGH then
		print("Moisture level is normal!")
	else
		print("Moisture level critical!")
		buzz()
	end

	-- Temperature and Humidity from BME280
	local temperature, humidity = read_bme280()
	if temperature and humidity then
		print(string.format("Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity))
		if temperature > 30 then -- Change threshold as required
			print("Warning! High Temperature Detected!")
			buzz()
		end
	else
		print("Failed to read from the temperature sensor.")
	end

	os.execute("sleep 0.5") -- Sleep for 0.5 seconds
end
