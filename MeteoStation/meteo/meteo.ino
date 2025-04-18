#include <DHT.h>
#include <Wire.h>
#include "Seeed_BMP280.h"
#include "DFRobot_RainfallSensor.h"
#include "Air_Quality_Sensor.h"
#include "SdsDustSensor.h"

// DHT11 sensor settings
#define DHTPIN 2
#define DHTTYPE DHT21       //DHT 21  (AM2301)
DHT dht(DHTPIN, DHTTYPE);   //Initialize DHT sensor for normal 16mhz

#define ANALOG_PIN A4  // Define the analog input pin connected to the sensor

// BMP280 sensor settings
BMP280 bmp280;
#define BMP280_ADDRESS 0x77 // BMP280 I2C address

// ZP04 sensor settings
AirQualitySensor sensor(A5);

// Wind speed sensor settings
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2 (compatible with Arduino Mega)
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3 (compatible with Arduino Mega)
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
float wind_gust = 0.0;

//
int rxPin = 15;
int txPin = 14;
SdsDustSensor sds(rxPin, txPin);

// Rainfall sensor settings
DFRobot_RainfallSensor_I2C Sensor(&Wire);

void setup() {
  Serial.begin(9600);

  // Pin mode setup
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), PinA, RISING); // Using digitalPinToInterrupt to map the pin to the correct interrupt
  attachInterrupt(digitalPinToInterrupt(pinB), PinB, RISING); // Using digitalPinToInterrupt to map the pin to the correct interrupt

  // Initialize DHT21 sensor
  dht.begin();

  // Initialize BMP280 sensor
  if (!bmp280.init(BMP280_ADDRESS)) {
    Serial.println("BMP280 Device error!");
  }

  //Initialize ZP04 Polution
  if (sensor.init()) {
        Serial.println("ZP 04 Polution Sensor ready.");
    } else {
        Serial.println("ZP 04 Polution Sensor ERROR!");
    }

  // Rainfall sensor setup
  delay(1000);
  while(!Sensor.begin()) {
    Serial.println("Rainfall Sensor init err!!!");
    delay(1000);
  }
  Serial.print("vid:\t");
  Serial.println(Sensor.vid,HEX);
  Serial.print("pid:\t");
  Serial.println(Sensor.pid,HEX);
  Serial.print("Version:\t");
  Serial.println(Sensor.getFirmwareVersion());
  // Set the rain accumulated value, unit: mm
  Sensor.setRainAccumulatedValue(0.2794);

  //PM 2.5 Sensor
  sds.begin();
  Serial.println(sds.queryFirmwareVersion().toString()); // prints firmware version
  Serial.println(sds.setActiveReportingMode().toString()); // ensures sensor is in 'active' reporting mode
  Serial.println(sds.setContinuousWorkingPeriod().toString()); // ensures sensor has continuous working period - default but not recommended
}

void PinA() {
  cli();
  reading = PIND & 0x0C;
  if (reading == B00001100 && aFlag) {
    encoderPos--;
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00000100)
    bFlag = 1;
  sei();
}

void PinB() {
  cli();
  reading = PIND & 0x0C;
  if (reading == B00001100 && bFlag) {
    encoderPos++;
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00001000)
    aFlag = 1;
  sei();
}

void loop() {
  
  // Read sensor data
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float pressure = bmp280.getPressure() / 100.0;
  float sensorValue = analogRead(A0);
  float voltage = (sensorValue / 1023) * 5.0;
  float wind_speed = mapfloat(voltage, 1.48, 2, 0, 32.4);
  
 
  //UV
  int sensorValueUV = analogRead(A2);
  int uvIndex = mapToUVIndex(sensorValueUV);

  //Air organic Polutent
  int quality = sensor.slope();

  // Update wind gust speed
  if (wind_speed > wind_gust) {
    wind_gust = wind_speed;
  }

//Wind Dir
  int WindDiranalogValue = analogRead(ANALOG_PIN);  // Read the analog voltage from the sensor
  float angle = WindDiranalogValue / 1024.0 * 360;  // Convert analog value to angle

  // Compute dew point
  float dewpoint = calcDewpoint(humidity, temperature);

  // Print sensor data through serial communication
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("WindSpeed: ");
  Serial.println(wind_speed);
  Serial.print("WindGust: ");
  Serial.println(wind_gust);
  Serial.print("DewPoint: ");
  Serial.println(dewpoint);

  //WindDIRECTION
  Serial.print("Angle:");
  Serial.println(angle);
  
  //UV Index
  Serial.print("UV Index = ");
  Serial.println(uvIndex);

  // Organic Polutents
  Serial.print("Sensor value: ");
  Serial.println(sensor.getValue());
  String airQualityString = getAirQualityString(quality);
  Serial.println(airQualityString);

  //PM Polution
  PmResult pm = sds.readPm();
  if (pm.isOk()) {
    Serial.print("PM2.5 = ");
    Serial.print(pm.pm25);
    Serial.print(", PM10 = ");
    Serial.println(pm.pm10);

    // if you want to just print the measured values, you can use toString() method as well
    Serial.println(pm.toString());
  } else {
    // notice that loop delay is set to 0.5s and some reads are not available
    Serial.print("Could not read values from sensor, reason: ");
    Serial.println(pm.statusToString());
  }

//

  // Read rainfall sensor data
  Serial.print("Sensor WorkingTime:\t");
  Serial.print(Sensor.getSensorWorkingTime());
  Serial.println(" H");
  Serial.print("Rainfall:\t");
  Serial.println(Sensor.getRainfall());
  Serial.print("1 Hour Rainfall:\t");
  Serial.print(Sensor.getRainfall(1));
  Serial.println(" mm");
  Serial.print("Daily Rainfall:\t");
  Serial.print(Sensor.getRainfall(24));
  Serial.println(" mm");
  Serial.print("rainfall raw:\t");
  Serial.println(Sensor.getRawData());
  
  delay(3000);  // Delay for 3 seconds between readings
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float calcDewpoint(float humi, float temp) {
  float k;
  k = log(humi/100) + (17.62 * temp) / (243.12 + temp);
  return 243.12 * k / (17.62 - k);
}

int mapToUVIndex(int sensorValue) {
  const int analogValue[] = {10, 47, 65, 83, 103, 124, 142, 162, 180, 200, 221, 240};
  const int uvIndex[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  const int numValues = sizeof(uvIndex) / sizeof(uvIndex[0]);

  int mappedIndex = 0;
  for (int i = 0; i < numValues; ++i) {
    if (sensorValue < analogValue[i]) {
      mappedIndex = uvIndex[i];
      break;
    }
  }
  return mappedIndex;
}

//Air Quality organic polutents

String getAirQualityString(int quality) {
    if (quality == AirQualitySensor::FORCE_SIGNAL) {
        return "High pollution! Force signal active.";
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
        return "High pollution!";
    } else if (quality == AirQualitySensor::LOW_POLLUTION) {
        return "Low pollution!";
    } else if (quality == AirQualitySensor::FRESH_AIR) {
        return "Fresh air.";
    } else {
        return "Unknown air quality."; // Handle any other cases if needed
    }
}
