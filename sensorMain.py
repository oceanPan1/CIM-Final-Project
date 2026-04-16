import serial
import time

ser = serial.Serial(
    port='COM7',       # Change to your ESP32's COM port (check Device Manager)
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=2
)

def read_frame():
    # Wait for start byte 0xAA
    while True:
        byte = ser.read(1)
        if not byte:
            return None, None
        if byte[0] == 0xAA:
            break

    # Read remaining 9 bytes
    rest = ser.read(9)
    if not rest or len(rest) < 9:
        return None, None

    # Validate frame
    if rest[0] != 0xC0 or rest[-1] != 0xAB:
        return None, None

    frame = byte + rest

    pm25 = (frame[2] | (frame[3] << 8)) / 10.0
    pm10 = (frame[4] | (frame[5] << 8)) / 10.0

    return pm25, pm10


try:
    print("Reading SDS011 sensor data...")
    while True:
        pm25, pm10 = read_frame()

        if pm25 is not None:
            print("PM2.5: {}  |  PM10: {}".format(pm25, pm10))
        else:
            print("Waiting for data...")

        time.sleep(1)

except KeyboardInterrupt:
    print("Stopped.")
    ser.close()