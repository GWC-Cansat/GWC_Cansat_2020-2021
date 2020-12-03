#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include <Arduino_LSM9DS1.h>

#include <Wire.h>
#include <SPI.h>
#include <Serial.h>
#include <RH_RF95.h>

#include <stdio.h>

#include <Adafruit_GPS.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 3
// Connect the GPS RX (receive) pin to Digital 2

// you can change the pin numbers to match your wiring:
UART mySerial(digitalPinToPinName(2), digitalPinToPinName(3), NC, NC);
Adafruit_GPS GPS(&mySerial);

#define RFM95_RST 5

// RFM9X radio
RH_RF95 rf95(4, 9);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

// Define this
int timeToWait;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()){
    Serial.println("init failed");
    while(1){}
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  //  driver.setTxPower(23, false);
  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
  //  driver.setTxPower(14, true);

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
}

void loop() {
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

  // Print all of the sensor values as csv
  double buf[12] = {pressure, temperature, humidity, Ax, Ay, Az, Mx, My, Mz, Gx, Gy, Gz};

  for (int x = 0; x < 12; x++) {
    Serial.print(buf[x]);

    if(x < 11) {
     Serial.print(","); 
    }
  }
 
  //Serial.println();

   char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

    //Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.print(GPS.milliseconds);
    Serial.print(",");
    //Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.print(GPS.year, DEC);
    Serial.print(",");
    //Serial.print("Fix: ");
    Serial.print((int)GPS.fix);
    //Serial.print(" quality: ");
    Serial.print(",");
    Serial.print((int)GPS.fixquality);
    Serial.print(",");
    if (GPS.fix) {
      //Serial.print("Location: ");
      Serial.print(GPS.latitude, 4);
      Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4);
      Serial.print(GPS.lon);

      //Serial.print("Speed (knots): ");
      Serial.print(GPS.speed);
      Serial.print(",");
      //Serial.print("Angle: ");
      Serial.print(GPS.angle);
      Serial.print(",");
      //Serial.print("Altitude: ");
      Serial.print(GPS.altitude);
      Serial.print(",");
      //Serial.print("Satellites: ");
      Serial.print((int)GPS.satellites);
    }
  delay(timeToWait);
  Serial.println();
}
