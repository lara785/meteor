void setup()
{
  Serial.begin(9600);
}

void loop()
{
  float sensorValue = analogRead(A0);
  Serial.print("Analog Value =");
  Serial.println(sensorValue);

  float voltage = (sensorValue / 1023) * 5; //Then we are reading the sensor analog value and then converting the value into voltage.
  Serial.print("Voltage =");
  Serial.print(voltage);
  Serial.println(" V");

  float wind_speed = mapfloat(voltage, 0.4, 2, 0, 32.4); 
/*Mapping the voltage to speedy is straightforward.
 The wind speed starts at 1m/s at 0.4V with a maximum of 32.4m/s at around 2.
 Arduino has a built-in map() function, but the map() does not work for floats, 
 so we have a simple mapFloat() function.
*/

  float speed_kmh = wind_speed * 3.6; //This functions is used to map the variable under a float.
 // Print sensor data through serial communication
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(H);
  Serial.print("Pressure: ");
  //Serial.println(pressure);
  Serial.print("WindSpeed: ");
  Serial.println(wind_speed);
  Serial.print("WindGust: ");
  Serial.println(wind_gust);
  Serial.print("DewPointF: ");
  Serial.println(DP);
  delay(3000);  // Delay for 3 seconds between readings
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
