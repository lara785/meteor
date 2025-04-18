#Cada vez que as observações coincidirem com as previsões, a teoria sobrevive e a confiança nela aumenta.
#No entanto, se nalgum momento uma nova observação a contradiz, temos de abandonar a teoria ou modificá-la.
#Stephen Hawking
#
# ROBOTIC - 2023
# Margarida Morim Sobral 
# Francisca José Machado
# Lara Silva
# Ariana Beatriz Sousa
# Dinis Costa
# Prf. António Pedro Cunha Santos
# RoboTIC - Mais que um grupo, uma familia

import serial
import requests
import time

# Arduino connection settings
serial_port = 'COM8'  # Replace with the appropriate serial port
baud_rate = 9600
pressure = float(1.0)
# Wunderground API settings
station_id = "IPVOAD5"
api_key = "a99d276be0384c889d276be0386c8848"
url = f"https://weatherstation.wunderground.com/weatherstation/updateweatherstation.php"

# Initialize serial communication with Arduino
ser = serial.Serial(serial_port, baud_rate, timeout=1)

def read_sensor_data():
    while True:
        line = ser.readline().decode().strip()
        if line.startswith("Temperature"):
            temperature = float(line.split(":")[1].strip())
        elif line.startswith("Humidity"):
            humidity = float(line.split(":")[1].strip())
        elif line.startswith("Pressure"):
            pressure = float(line.split(":")[1].strip())
        elif line.startswith("WindSpeed"):
            wind_speed = float(line.split(":")[1].strip())
        elif line.startswith("WindGust"):
            wind_gust = float(line.split(":")[1].strip())
        elif line.startswith("DewPointF"):
            dewpoint_f = float(line.split(":")[1].strip())
            return temperature, humidity, pressure, wind_speed, wind_gust, dewpoint_f

def transmit_data(temperature, humidity, pressure, wind_speed, wind_gust, dewpoint_f):
    payload = {
        "ID": station_id,
       "PASSWORD": "jJbvmEnA",
        "dateutc": "now",
        "tempf": temperature * 1.8 + 32,  # Convert Celsius to Fahrenheit
        "humidity": humidity,
        "baromin": pressure * 0.0295300,  # Convert hpa to inHg
        "windspeedmph": abs(wind_speed * 0.6213711922),  # Convert m/s to mph
        "windspeedkmh": wind_speed,
        "windgustmph": abs(wind_gust * 0.6213711922),
        "dewptf": dewpoint_f,    
    }

    response = requests.post(url, data=payload)
    if response.status_code == 200:
        print("Data transmitted successfully.")
    else:
        print("Failed to transmit data.")

while True:
    try:
        temperature, humidity, pressure, wind_speed, wind_gust, dewpoint_f = read_sensor_data()
        transmit_data(temperature, humidity, pressure, wind_speed, wind_gust, dewpoint_f)
        time.sleep(30)  # Delay for 1 minute between updates
    except Exception as e:
        print(f"An error occurred: {str(e)}")
        print("Restarting...")
        time.sleep(5)  # Delay before restarting
        continue

    time.sleep(30)  # Delay for 1 minute between updates