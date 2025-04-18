#include <DHT.h>
#include <Wire.h>
#include "Seeed_BMP280.h"
#include "DFRobot_RainfallSensor.h"

class WeatherStation {
private:
    // DHT sensor settings
    #define DHTPIN 2
    #define DHTTYPE DHT21       //DHT 21  (AM2301)
    DHT dht;
    
    // BMP280 sensor settings
    BMP280 bmp280;
    #define BMP280_ADDRESS 0x77 // BMP280 I2C address
    
    // Wind speed sensor settings
    int pinA;
    int pinB;
    volatile byte aFlag;
    volatile byte bFlag;
    volatile byte encoderPos;
    volatile byte oldEncPos;
    volatile byte reading;

    // Rainfall sensor settings
    DFRobot_RainfallSensor_I2C Sensor;
    
    float wind_gust;
    static WeatherStation* instance;

public:
    WeatherStation(int pinA_, int pinB_) : dht(DHTPIN, DHTTYPE), bmp280(), Sensor(&Wire) {
        pinA = pinA_;
        pinB = pinB_;
        aFlag = 0;
        bFlag = 0;
        encoderPos = 0;
        oldEncPos = 0;
        reading = 0;
        wind_gust = 0.0;
        instance = this;
    }

    void setup() {
        Serial.begin(9600);
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pinA), &WeatherStation::PinAStatic, RISING);
        attachInterrupt(digitalPinToInterrupt(pinB), &WeatherStation::PinBStatic, RISING);
        dht.begin();
        if (!bmp280.init(BMP280_ADDRESS)) {
            Serial.println("BMP280 Device error!");
        }
        delay(1000);
        while (!Sensor.begin()) {
            Serial.println("Rainfall Sensor init err!!!");
            delay(1000);
        }
        Serial.print("vid:\t");
        Serial.println(Sensor.vid, HEX);
        Serial.print("pid:\t");
        Serial.println(Sensor.pid, HEX);
        Serial.print("Version:\t");
        Serial.println(Sensor.getFirmwareVersion());
        Sensor.setRainAccumulatedValue(0.2794);
    }

    static void PinAStatic() {
        instance->PinA();
    }

    static void PinBStatic() {
        instance->PinB();
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
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        float pressure = bmp280.getPressure() / 100.0;
        float sensorValue = analogRead(A0);
        float voltage = (sensorValue / 1023) * 5.0;
        float wind_speed = mapfloat(voltage, 1.48, 2, 0, 32.4);

        if (wind_speed > wind_gust) {
            wind_gust = wind_speed;
        }

        float dewpoint = calcDewpoint(humidity, temperature);

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
        Serial.print("EncoderPos: ");
        Serial.println(encoderPos);

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

        delay(3000);
    }

private:
    float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    float calcDewpoint(float humi, float temp) {
        float k;
        k = log(humi / 100) + (17.62 * temp) / (243.12 + temp);
        return 243.12 * k / (17.62 - k);
    }
};

WeatherStation* WeatherStation::instance = nullptr;

WeatherStation weatherStation(2, 3);

void setup() {
    weatherStation.setup();
}

void loop() {
    weatherStation.loop();
}
