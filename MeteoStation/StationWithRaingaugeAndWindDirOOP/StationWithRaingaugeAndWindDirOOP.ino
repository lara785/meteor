#include <Wire.h>
#include "Seeed_BMP280.h"
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 2
#define DHTTYPE DHT11

class WeatherStation {
private:
  DHT dht;
  BMP280 bmp280;
  float wind_speed;
  float wind_gust;

public:
  WeatherStation(int dht_pin) : dht(dht_pin, DHTTYPE) {
    wind_speed = 0.0;
    wind_gust = 0.0;
  }

  void setup() {
    Serial.begin(9600);
    pinMode(A0, INPUT);
    dht.begin();
     if(!bmp280.init())
     {
    Serial.println("Device error!");
    }
  }

  void readData() {
    float temperature = bmp280.getTemperature();
    float humidity = dht.readHumidity();
    float pressure = bmp280.getPressure() / 100.0;
    float sensorValue = analogRead(A0);
    float voltage = (sensorValue / 1023.0) * 5.0;
    wind_speed = map(voltage, 0.4, 2.0, 0, 32.4);
    if (wind_speed > wind_gust) {
      wind_gust = wind_speed;
    }
    Serial.println("Weather Data:");
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    Serial.print("Pressure: ");
    Serial.println(pressure);
    Serial.print("Wind Speed: ");
    Serial.println(wind_speed);
    Serial.print("Wind Gust: ");
    Serial.println(wind_gust);
  }
};

class RainSensor {
private:
  const float mmPerPulse;
  float mmTotali;
  int sensore;
  int statoPrecedente;

public:
  RainSensor(float mm_per_pulse) : mmPerPulse(mm_per_pulse) {
    mmTotali = 0;
    sensore = 0;
    statoPrecedente = 0;
  }

  void setup() {
    pinMode(9, INPUT);
  }

  void readData() {
    sensore = digitalRead(9);
    if (sensore != statoPrecedente) {
      mmTotali += mmPerPulse;
    }
    statoPrecedente = sensore;
    Serial.print("Total Rainfall: ");
    Serial.print(mmTotali);
    Serial.println(" mm");
  }
};

LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(4, 0);
  lcd.print("Weather Station");
  delay(1000);
  lcd.clear();
}

void loop() {
  WeatherStation weatherStation(DHTPIN);
  RainSensor rainSensor(0.173);
  weatherStation.setup();
  rainSensor.setup();

  while (true) {
    weatherStation.readData();
    rainSensor.readData();

    // Display sensor data on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    // Read temperature from weather station
    lcd.print("temperature"); // Replace "temperature" with actual temperature reading
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    // Read humidity from weather station
    lcd.print("humidity"); // Replace "humidity" with actual humidity reading
    lcd.print("%");
    lcd.setCursor(0, 2);
    lcd.print("Pressure: ");
    // Read pressure from weather station
    lcd.print("pressure"); // Replace "pressure" with actual pressure reading
    lcd.print(" hPa");
    lcd.setCursor(0, 3);
    lcd.print("Rain: ");
    // Read total rainfall from rain sensor
    lcd.print("rainfall"); // Replace "rainfall" with actual rainfall reading
    lcd.print(" mm");
    delay(5000);  // Delay for 5 seconds between readings
  }
}
