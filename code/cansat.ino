#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Serial.h>
#include <time.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define SEALEVELPRESSURE_HPA 1013.25
#define SAMPLE_RATE          50

// BME680 GAS sensor [CS:10, MOSI:12, MISO:11, SCK:9]
Adafruit_BME680 bme(10, 12, 11,	9);

// HC12 radio module [TX:5 RX:4]
UART HC12(digitalPinToPinName(4), digitalPinToPinName(5), NC, NC);

/**
 * Connect the GPS Power pin to 5V
 * Connect the GPS Ground pin to ground
 * Connect the GPS TX (transmit) pin to Digital 3
 * Connect the GPS RX (receive) pin to Digital 2
 * You will also have to edit "pings_arduino.txt" which can be found in the variants folder
 * Change
 * #define PIN_SERIAL_RX (1ul)
 * #define PIN_SERIAL_TX (0ul)
 * To
 * #define PIN_SERIAL_RX (3ul)
 * #define PIN_SERIAL_TX (2ul)
 */

// you can change the pin numbers to match your wiring:
Adafruit_GPS GPS(&_UART1_);

/**
 * Status of the onboard sensors
 * 7     5      4     3     2     1     0
 * |-----| BARO | HTS | IMU | GPS | BME |
 */

unsigned char DeviceStatus = 0;

void setup() {
	// Start the serial connection to the HC12
	HC12.begin(9600);

	// Reset the status
	DeviceStatus.status = 0;

	// Start the barometer
	if (!BARO.begin()) {
		DeviceStatus |= 0x10;
	}

	// Start the humidity and temperature sensor
	if (!HTS.begin()) {
		DeviceStatus |= 0x08;
	}

	// Start the IMU
	if (!IMU.begin()) {
		DeviceStatus |= 0x04;
	}

	// Configure the GPS
	if(!GPS.begin(9600)) {
		DeviceStatus |= 0x02;
	} else {
		GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
		GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
		GPS.sendCommand(PGCMD_ANTENNA);
	}
	
	// Configure the BME680
	if (!bme.begin()) {
		DeviceStatus |= 0x01;
	} else {
		// Set up oversampling and filter initialization
		bme.setTemperatureOversampling(BME680_OS_8X);
		bme.setHumidityOversampling(BME680_OS_2X);
		bme.setPressureOversampling(BME680_OS_4X);
		bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
		bme.setGasHeater(320, 150); // 320*C for 150 ms
	}
}

void loop() {
	// Send the device status
	HC12.write(DeviceStatus);

	// Barometer
	if(!(DeviceStatus & 0x10)) {
		// Sensor type
		HC12.write(0x00);

		// Get the pressure
		float pressure = BARO.readPressure();

		// Send the data
		HC12.write(&pressure, sizeof(float));
	}

	// Humidity and Temperature Sensor
	if(!(DeviceStatus & 0x08)) {
		// Sensor type
		HC12.write(0x01);

		// Get the temperature and humidity
		float temperature = HTS.readTemperature();
		float humidity = HTS.readHumidity();

		// Send the data
		HC12.write(&temperature, sizeof(float));
		HC12.write(&humidity, sizeof(float));
	}

	// Inertial Measurement Unit
	if(!(DeviceStatus & 0x04)) {
		// Sensor type
		HC12.write(0x02);

		// Get the gyroscope values
		float Gx, Gy, Gz;
		while (!IMU.gyroscopeAvailable()) {}
		IMU.readGyroscope(Gx, Gy, Gz);

		// Get the magnetometer values
		float Mx, My, Mz;
		while (!IMU.magneticFieldAvailable()) {}
		IMU.readMagneticField(Mx, My, Mz);

		// Get the accelerometer values
		float Ax, Ay, Az;
		while (!IMU.accelerationAvailable()) {}
		IMU.readAcceleration(Ax, Ay, Az);

		// Send the data
		HC12.write(&Gx, sizeof(float));
		HC12.write(&Gy, sizeof(float));
		HC12.write(&Gz, sizeof(float));
		HC12.write(&Mx, sizeof(float));
		HC12.write(&My, sizeof(float));
		HC12.write(&Mz, sizeof(float));
		HC12.write(&Ax, sizeof(float));
		HC12.write(&Ay, sizeof(float));
		HC12.write(&Az, sizeof(float));
	}

	// GPS
	if(!(DeviceStatus & 0x02)) {
		// Sensor type
		HC12.write(0x03);

		// Read data from the GPS if any is avaliable
		int size = GPS.available();
		for(int x = 0; x < size; x++) GPS.read();

		// Parse the NMEA string from the GPS
		if (GPS.newNMEAreceived()) {
			GPS.parse(GPS.lastNMEA());
		}

		// Convert the time to unix timestamp
		struct tm t;
		unsigned long epoch;

		t.tm_year = GPS.year + 100;
		t.tm_mon = GPS.month - 1;
		t.tm_mday = GPS.day;
		t.tm_hour = GPS.hour;
		t.tm_min = GPS.minute;
		t.tm_sec = GPS.seconds;

		epoch = mktime(&t);

		// Send the data
		HC12.write(GPS.fix);
		HC12.write(GPS.fixquality);
		HC12.write(GPS.satellites);
		HC12.write(&epoch, sizeof(unsigned long));
		HC12.write(&GPS.latitude, sizeof(nmea_float_t));
		HC12.write(&GPS.longitude, sizeof(nmea_float_t));
		HC12.write(&GPS.speed, sizeof(nmea_float_t));
		HC12.write(&GPS.angle, sizeof(nmea_float_t));
		HC12.write(&GPS.altitude, sizeof(nmea_float_t));
	}

	// BME680
	if(!(DeviceStatus & 0x01)) {
		if (bme.performReading()) {
			// Sensor type
			HC12.write(0x04);

			// Read the data
			float temperature = bme.temperature;
			float pressure = bme.pressure / 100.0;
			float humidity = bme.humidity;
			float gasResistance = bme.gas_resistance / 1000.0;
			float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

			// Send the data
			HC12.write(&temperature, sizeof(float));
			HC12.write(&pressure, sizeof(float));
			HC12.write(&humidity, sizeof(float));
			HC12.write(&gasResistance, sizeof(float));
			HC12.write(&altitude, sizeof(float));
		}
	}

	HC12.println();
	delay(SAMPLE_RATE);
}
