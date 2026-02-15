import sys
import unittest
from unittest.mock import MagicMock
from esp_gateway import EspSerial

class MockSerial:
    def __init__(self):
        self.timeout = 1
        self.baudrate = 921600
        self.dtr = False
        self.rts = False
        self.data_to_read = bytearray()

    def read(self, size):
        data = self.data_to_read[:size]
        self.data_to_read = self.data_to_read[size:]
        return bytes(data)

    def write(self, data):
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.data_to_read = bytearray()

class TestEspSerial(unittest.TestCase):
    def setUp(self):
        # Patch serial.Serial to return our MockSerial
        import serial
        serial.Serial = MagicMock(return_value=MockSerial())
        self.esp = EspSerial(port="MOCK", baudrate=921600)
        self.mock_serial = self.esp.serial

    def test_read_data_packet(self):
        # Prepare a data packet: length=6, type=0, payload=b'hi'
        # [0x06, 0x00, 0x00, 0x00, 0x68, 0x69]
        packet = bytes([0x06, 0x00, 0x00, 0x00, 0x68, 0x69])
        self.mock_serial.data_to_read.extend(packet)
        
        data = self.esp.read(2)
        self.assertEqual(data, b'hi')
        self.assertEqual(len(self.esp.rx_buffer), 0)

    def test_read_multiple_packets(self):
        # Packet 1: DATA 'A'
        self.mock_serial.data_to_read.extend(bytes([0x05, 0x00, 0x00, 0x00, ord('A')]))
        # Packet 2: LOG 'LogMsg\n'
        log_payload = b'LogMsg\n'
        log_packet = bytes([len(log_payload)+4, 0x00, 0x03, 0x00]) + log_payload
        self.mock_serial.data_to_read.extend(log_packet)
        # Packet 3: DATA 'B'
        self.mock_serial.data_to_read.extend(bytes([0x05, 0x00, 0x00, 0x00, ord('B')]))
        
        data = self.esp.read(2)
        self.assertEqual(data, b'AB')

    def test_timeout(self):
        # Buffer empty, serial.read returns empty
        data = self.esp.read(1)
        self.assertEqual(data, b'')

if __name__ == '__main__':
    unittest.main()
