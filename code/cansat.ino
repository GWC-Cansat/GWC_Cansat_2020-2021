#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_CCS811.h>
#include <Wire.h>
#include <Adafruit_VEML6075.h>

Adafruit_VEML6075 uv = Adafruit_VEML6075();
Adafruit_CCS811 ccs;

// Define this
int timeToWait;

void setup() {
  Serial.begin(9600);
  while (!Serial);

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

  if(!ccs.begin()){
    Serial.println("Failed to start sensor (CCS811)! Please check your wiring.");
    while (1);
  }

  if (! uv.begin()) {
    Serial.println("Failed to communicate with VEML6075 sensor, check wiring?");
  }

  uv.setIntegrationTime(VEML6075_100MS);
  uv.setHighDynamic(true);
  uv.setForcedMode(false);
  uv.setCoefficients(2.22, 1.33,  // UVA_A and UVA_B coefficients
                     2.95, 1.74,  // UVB_C and UVB_D coefficients
                     0.001461, 0.002591); // UVA and UVB responses

  // Get the speed of all the IMU sensors so we determine how long to wait
  int AccelSampleRate = IMU.accelerationSampleRate();
  int GyroSampleRate = IMU.gyroscopeSampleRate();
  int MagnetSampleRate = IMU.magneticFieldSampleRate();

  // Get the time to wait in milliseconds by (1 / LowestSampleRate)*1000
  int timeToWait = (1 / min(AccelSampleRate, min(GyroSampleRate, MagnetSampleRate))) * 1000;
  Serial.print(timeToWait);
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
  while(!IMU.gyroscopeAvailable()) {}

  // Get the gyroscope values
  IMU.readGyroscope(Gx, Gy, Gz);

  // Magnetic Field
  float Mx, My, Mz;

  // Wait until data avialiable
  while(!IMU.magneticFieldAvailable()) {}

  // Get the magnetometer values
  IMU.readMagneticField(Mx, My, Mz);
  
  // Acceleration
  float Ax, Ay, Az;

  // Wait until data avialiable
  while(!IMU.accelerationAvailable()) {}

  // Get the acceleration values
  IMU.readAcceleration(Ax, Ay, Az);

  // Air sensor values that I have no idea what they mean
  float co2, voc;

  // Wait for it to be ready
  while(!ccs.available()) {}

  co2 = ccs.geteCO2();
  voc = ccs.getTVOC();

  // UV Sensors
  float uva, uvb, uvi;

  uva = uv.readUVA();
  uvb = uv.readUVB();
  uvi = uv.readUVI();

  // Print all of the sensor values as csv
  Serial.print(pressure);
  Serial.print(", ");
  
  Serial.print(temperature);
  Serial.print(", ");

  Serial.print(humidity);
  Serial.print(", ");

  Serial.print(Ax);
  Serial.print(", ");

  Serial.print(Ay);
  Serial.print(", ");

  Serial.print(Az);
  Serial.print(", ");

  Serial.print(Mx);
  Serial.print(", ");

  Serial.print(My);
  Serial.print(", ");

  Serial.print(Mz);
  Serial.print(", ");

  Serial.print(Gx);
  Serial.print(", ");

  Serial.print(Gy);
  Serial.print(", ");

  Serial.print(Gz);
  Serial.print(", ");

  Serial.print(co2);
  Serial.print(", ");

  Serial.print(voc);
  Serial.print(", ");

  Serial.print(uva);
  Serial.print(", ");

  Serial.print(uvb);
  Serial.print(", ");

  Serial.print(uvi);
  Serial.print(", ");

  // Print a newline
  Serial.println();

  // wait 1 second to print again
  delay(timeToWait);
}
