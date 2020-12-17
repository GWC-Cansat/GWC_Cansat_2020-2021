#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include <Arduino_LSM9DS1.h>

#include <Wire.h>
#include <SPI.h>
#include <Serial.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define BME_SCK 9
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

//Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

#include <stdio.h>

#include <Adafruit_GPS.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 3
// Connect the GPS RX (receive) pin to Digital 2

// you can change the pin numbers to match your wiring:
UART mySerial(digitalPinToPinName(2), digitalPinToPinName(3), NC, NC);
Adafruit_GPS GPS(&mySerial);

//HC12 radio module [TX:5 RX:4]
UART HC12(digitalPinToPinName(4), digitalPinToPinName(5), NC, NC);


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

// Define this
int timeToWait;
#define LED 13 //on-board led

void setup() {
 // Serial.begin(115200);
  HC12.begin(9600);

  // Start the pressure sensor
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  // Start the humidity and temperature sensor
  if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }

  // Start the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Get the speed of all the IMU sensors so we determine how long to wait
  int AccelSampleRate = IMU.accelerationSampleRate();
  int GyroSampleRate = IMU.gyroscopeSampleRate();
  int MagnetSampleRate = IMU.magneticFieldSampleRate();

  // Get the time to wait in milliseconds by (1 / LowestSampleRate)*1000
  int timeToWait = (1 / min(AccelSampleRate, min(GyroSampleRate, MagnetSampleRate))) * 1000;
  Serial.print(timeToWait);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  pinMode(LED, OUTPUT);
}

void loop() {
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  // Read the pressure sensor value
  float pressure = BARO.readPressure();

  // Read the humidity and temperature sensor values
  float temperature = HTS.readTemperature();
  float humidity    = HTS.readHumidity();

  // Values for the IMU
  // Gyroscope
  float Gx, Gy, Gz;

  // Wait until data avialiable
  while (!IMU.gyroscopeAvailable()) {}

  // Get the gyroscope values
  IMU.readGyroscope(Gx, Gy, Gz);

  // Magnetic Field
  float Mx, My, Mz;

  // Wait until data avialiable
  while (!IMU.magneticFieldAvailable()) {}

  // Get the magnetometer values
  IMU.readMagneticField(Mx, My, Mz);

  // Acceleration
  float Ax, Ay, Az;

  // Wait until data avialiable
  while (!IMU.accelerationAvailable()) {}

  // Get the acceleration values
  IMU.readAcceleration(Ax, Ay, Az);

  // put the values into an array
  double buf[12] = {pressure, temperature, humidity, Ax, Ay, Az, Mx, My, Mz, Gx, Gy, Gz};
/*
  for (int x = 0; x < 12; x++) {
    Serial.print(buf[x]);
    if (x < 11) {
      Serial.print(",");
    }
  }

  Serial.println();

  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);
  HC12.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  //print values to serial for debugging

  //Serial.print("\nTime: ");
  if (GPS.hour < 10) {
    Serial.print('0');
  }
  Serial.print(GPS.hour, DEC); Serial.print(':');
  if (GPS.minute < 10) {
    Serial.print('0');
  }
  Serial.print(GPS.minute, DEC); Serial.print(':');
  if (GPS.seconds < 10) {
    Serial.print('0');
  }
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  if (GPS.milliseconds < 10) {
    Serial.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    Serial.print("0");
  }
  Serial.print(GPS.milliseconds);
  Serial.print(",");
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.print(GPS.year, DEC);
  Serial.print(",");
  Serial.print("Fix: ");
  Serial.print((int)GPS.fix);
  Serial.print(" quality: ");
  Serial.print(",");
  Serial.print((int)GPS.fixquality);
  Serial.print(",");
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4);
    Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4);
    Serial.print(GPS.lon);

    Serial.print("Speed (knots): ");
    Serial.print(GPS.speed);
    Serial.print(",");
    Serial.print("Angle: ");
    Serial.print(GPS.angle);
    Serial.print(",");
    Serial.print("Altitude: ");
    Serial.print(GPS.altitude);
    Serial.print(",");
    Serial.print("Satellites: ");
    Serial.print((int)GPS.satellites);
  }
  Serial.println();
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  Serial.println();
*/
  /////////////////////////////////////////////////////////

  //turn on led to show that packet has started sending
  digitalWrite(LED, HIGH);

  //send data from on-board sensors to HC12
  for (int x = 0; x < 12; x++) {
    HC12.print(buf[x]);

    if (x < 11) {
      HC12.print(",");
    }
  }

  HC12.print(",");

  //print gps to HC12
  //Serial.print("\nTime: ");

  if (GPS.hour < 10) {
    HC12.print('0');
  }
  HC12.print(GPS.hour, DEC); HC12.print(':');
  if (GPS.minute < 10) {
    HC12.print('0');
  }
  HC12.print(GPS.minute, DEC); HC12.print(':');
  if (GPS.seconds < 10) {
    HC12.print('0');
  }
  HC12.print(GPS.seconds, DEC); HC12.print('.');
  if (GPS.milliseconds < 10) {
    HC12.print("00");
  } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    HC12.print("0");
  }
  HC12.print(GPS.milliseconds);
  HC12.print(",");
  //Serial.print("Date: ");
  HC12.print(GPS.day, DEC); HC12.print('/');
  HC12.print(GPS.month, DEC); HC12.print("/20");
  HC12.print(GPS.year, DEC);
  HC12.print(",");
  //Serial.print("Fix: ");
  HC12.print((int)GPS.fix);
  //Serial.print(" quality: ");
  HC12.print(",");
  HC12.print((int)GPS.fixquality);
  HC12.print(",");
  if (GPS.fix) {
    //Serial.print("Location: ");
    HC12.print(GPS.latitude, 4);
    HC12.print(GPS.lat);
    HC12.print(",");
    HC12.print(GPS.longitude, 4);
    HC12.print(GPS.lon);
    HC12.print(",");

    //Serial.print("Speed (knots): ");
    HC12.print(GPS.speed);
    HC12.print(",");
    //Serial.print("Angle: ");
    HC12.print(GPS.angle);
    HC12.print(",");
    //Serial.print("Altitude: ");
    HC12.print(GPS.altitude);
    HC12.print(",");
    //Serial.print("Satellites: ");
    HC12.print((int)GPS.satellites);
  }
  //send bme680 data
  HC12.print(bme.temperature);
  HC12.print(",");
  HC12.print(bme.pressure / 100.0);
  HC12.print(",");
  HC12.print(bme.humidity);
  HC12.print(",");
  HC12.print(bme.gas_resistance / 1000.0);
  HC12.print(",");
  HC12.print(bme.readAltitude(SEALEVELPRESSURE_HPA));

  HC12.println();
  //turn off led to show packet has been sent
  digitalWrite(LED, LOW);
  
  delay(timeToWait);
}
