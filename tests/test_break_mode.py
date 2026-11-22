import unittest
from unittest.mock import patch

import cc
from cc import CC3200Connection, CC3200Error, Pincfg

VERSION_DATA = bytes([0, 4, 1, 2] + [0] * 12 + [16, 0, 0, 0] + [0] * 8)
GET_VERSION = b'\x00\x03\x2f\x2f'
ACK = b'\x00\xcc'


class FakeBootloaderPort:
    """Serial port connected to a simulated CC3200 ROM bootloader.

    hw_break_works=False mimics an adapter that ignores send_break().
    garble_after_emulated_break mimics the first reply bytes getting lost
    after switching back from the low baud rate.
    """

    def __init__(self, hw_break_works, garble_after_emulated_break=False):
        self.hw_break_works = hw_break_works
        self.garble = garble_after_emulated_break
        self.baudrate = cc.CC3200_BAUD
        self.timeout = 1
        self.dtr = False
        self.rts = False
        self.rx = bytearray()
        self.in_bootloader = False
        self.awaiting_ack = False
        self.emulated_breaks = 0

    def send_break(self, duration=0.25):
        if self.hw_break_works:
            self.in_bootloader = True
            self.rx += ACK

    def write(self, data):
        if self.baudrate == cc.EMULATED_BREAK_BAUD:
            if data == b'\x00':
                # break ACK is sent while the host is still at 300 baud: lost
                self.in_bootloader = True
                self.emulated_breaks += 1
            return len(data)
        if self.baudrate != cc.CC3200_BAUD or not self.in_bootloader:
            return len(data)
        if data == ACK:
            self.awaiting_ack = False
        elif data == GET_VERSION and not self.awaiting_ack:
            reply = ACK + bytes([0, 30, sum(VERSION_DATA) & 0xff]) + VERSION_DATA
            if self.garble:
                reply = reply[1:]
                self.garble = False
            self.rx += reply
            self.awaiting_ack = True
        return len(data)

    def read(self, size=1):
        data, self.rx = bytes(self.rx[:size]), self.rx[size:]
        return data

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.rx.clear()

    flushInput = reset_input_buffer


def connect(port, break_mode):
    conn = CC3200Connection(port, reset=Pincfg(False, 'dtr'), sop2=Pincfg(False, 'none'),
                            reset_wait=0, break_mode=break_mode)
    conn.connect()
    return conn


@patch('cc.time.sleep', lambda s: None)
class TestBreakMode(unittest.TestCase):
    def test_auto_uses_hw_break_when_it_works(self):
        port = FakeBootloaderPort(hw_break_works=True)
        conn = connect(port, "auto")
        self.assertTrue(conn.vinfo.is_cc3200)
        self.assertEqual(port.emulated_breaks, 0)

    def test_auto_falls_back_to_emulated_break(self):
        port = FakeBootloaderPort(hw_break_works=False)
        conn = connect(port, "auto")
        self.assertTrue(conn.vinfo.is_cc3200)
        self.assertEqual(port.emulated_breaks, 1)
        self.assertEqual(port.baudrate, cc.CC3200_BAUD)

    def test_emulated_break_recovers_from_garbled_first_reply(self):
        port = FakeBootloaderPort(hw_break_works=False, garble_after_emulated_break=True)
        conn = connect(port, "emulate")
        self.assertTrue(conn.vinfo.is_cc3200)
        self.assertEqual(port.emulated_breaks, 1)

    def test_hw_mode_does_not_fall_back_and_hints_at_emulate(self):
        port = FakeBootloaderPort(hw_break_works=False)
        with self.assertRaisesRegex(CC3200Error, "--break-mode emulate"):
            connect(port, "hw")
        self.assertEqual(port.emulated_breaks, 0)


if __name__ == '__main__':
    unittest.main()
