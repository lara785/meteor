import serial
import time

# Serial port configuration
SERIAL_PORT = 'COM11'  # Change this to match your serial port
BAUD_RATE = 9600

# Function to send AT command and read response
def send_at_command(ser, command, timeout=1):
    ser.write((command + '\r\n').encode())
    time.sleep(timeout)
    response = ser.read_all().decode().strip()
    return response

# Function to send LoRa message
def send_lora_message(ser, message):
    # Enter LoRa command mode
    response = send_at_command(ser, 'AT+MODE=TEST')
    if 'OK' not in response:
        print("Failed to enter LoRa mode")
        return
    
    # Set LoRa parameters
    response = send_at_command(ser, 'AT+BAND=861000000')
    response = send_at_command(ser, 'AT+ADDR=1')
    response = send_at_command(ser, 'AT+NETWORKID=2')
    response = send_at_command(ser, 'AT+DATA=HEX')

    # Send LoRa message
    response = send_at_command(ser, 'AT+SEND={}'.format(message))
    if 'OK' in response:
        print("Message sent successfully")
    else:
        print("Failed to send message")

def main():
    # Open serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    if ser.isOpen():
        print("Serial port opened successfully")
    else:
        print("Failed to open serial port")
        return
    
    # Send message via LoRa
    message = "Hello, LoRa World!"
    send_lora_message(ser, message)
    
    # Close serial port
    ser.close()
    print("Serial port closed")

if __name__ == "__main__":
    main()
