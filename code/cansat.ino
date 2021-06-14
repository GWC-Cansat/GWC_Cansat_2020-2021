#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Serial.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define BME_SCK 9
#define BME_MISO 11
#define BME_MOSI 12
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 3
// Connect the GPS RX (receive) pin to Digital 2
// You will also have to edit "pings_arduino.txt" which can be found in the variants folder
// Change
// #define PIN_SERIAL_RX (1ul)
// #define PIN_SERIAL_TX (0ul)
// To
// #define PIN_SERIAL_RX (3ul)
// #define PIN_SERIAL_TX (2ul)

// you can change the pin numbers to match your wiring:
Adafruit_GPS GPS(&_UART1_);

//HC12 radio module [TX:5 RX:4]
UART HC12(digitalPinToPinName(4), digitalPinToPinName(5), NC, NC);


// Define this
int timeToWait;
#define LED 13 //on-board led

void setup() {
  //_UART1_.begin(9600);
  HC12.begin(9600);

  // Start the pressure sensor
  if (!BARO.begin()) {
    //Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  // Start the humidity and temperature sensor
  if (!HTS.begin()) {
    //Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }

  // Start the IMU
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Get the speed of all the IMU sensors so we determine how long to wait
  int AccelSampleRate = IMU.accelerationSampleRate();
  int GyroSampleRate = IMU.gyroscopeSampleRate();
  int MagnetSampleRate = IMU.magneticFieldSampleRate();

  // Get the time to wait in milliseconds by (1 / LowestSampleRate)*1000
  int timeToWait = (1 / min(AccelSampleRate, min(GyroSampleRate, MagnetSampleRate))) * 1000;

  // Configure the GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  if (!bme.begin()) {
    //Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  //Serial.println("Started");

  pinMode(LED, OUTPUT);
}

void loop() {
  
  // Read the pressure, temperature and humidity from the onboard sensors
  float onbPressure = BARO.readPressure();
  float onbTemperature = HTS.readTemperature();
  float onbHumidity = HTS.readHumidity();

  // Read the BME680 data
  float bmeTemperature = 0.0, bmePressure = 0.0, bmeHumidity = 0.0, gasResistance = 0.0, altitude = 0.0;
  if (bme.performReading()) {
    bmeTemperature = bme.temperature;
    bmePressure = bme.pressure / 100.0;
    bmeHumidity = bme.humidity;
    gasResistance = bme.gas_resistance / 1000.0;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  }
  
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

  // put the values into an array
  double buf[] = {onbPressure, onbTemperature, onbHumidity, bmeTemperature, bmePressure, bmeHumidity, gasResistance, altitude, Ax, Ay, Az, Mx, My, Mz, Gx, Gy, Gz};

  // Send data from on-board sensors to HC12
  for (int x = 0; x < sizeof(buf) / sizeof(double); x++) {
    HC12.print(buf[x]);
    HC12.print(",");
  }

  // Check for GPS data
  int bytes = GPS.available();
  for(int x = 0; x < bytes; x++) GPS.read();
  if (GPS.newNMEAreceived()) {
    // Decode GPS data
    GPS.parse(GPS.lastNMEA());
  }
  
  // GPS fix and quality data
  HC12.print((int)GPS.fix);
  HC12.print(",");
  HC12.print((int)GPS.fixquality);
  
  // Time
  HC12.print(GPS.hour, DEC); HC12.print(':');
  HC12.print(GPS.minute, DEC); HC12.print(':');
  HC12.print(GPS.seconds, DEC); HC12.print('.');
  HC12.print(GPS.milliseconds);
  
  // Date
  HC12.print(GPS.day, DEC); HC12.print('/');
  HC12.print(GPS.month, DEC); HC12.print("/20");
  HC12.print(GPS.year, DEC);
  HC12.print(",");

  // Location
  HC12.print(GPS.latitude, 4);
  HC12.print(GPS.lat);
  HC12.print(",");
  HC12.print(GPS.longitude, 4);
  HC12.print(GPS.lon);
  HC12.print(",");

  // Speed, Angle and Altitude
  HC12.print(GPS.speed);
  HC12.print(",");
  HC12.print(GPS.angle);
  HC12.print(",");
  HC12.print(GPS.altitude);
  HC12.print(",");
  HC12.print((int)GPS.satellites);

  HC12.println();
  delay(timeToWait);
}
