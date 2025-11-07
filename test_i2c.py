import smbus
import time

bus = smbus.SMBus(1)

SLAVE_ADDR = 0x08

def request_from_arduino():
    try:
        data = bus.read_i2c_block_data(SLAVE_ADDR, 0, 16)

        received = ''.join([chr(byte) for byte in data if byte != 255 and byte != 0])
        print(f"Data From Arduino: {received}")

    except Exception as e:
        print(f"Error: {e}")

while True:
    request_from_arduino()
    time.sleep(0.1)