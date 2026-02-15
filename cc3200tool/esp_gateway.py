import serial
import time
import sys
import logging

log = logging.getLogger()

class EspSerial:
    # Packet header constants
    PACKET_HEADER_SIZE = 4
    PACKET_TYPE_DATA = 0x00
    PACKET_TYPE_CONFIG = 0x01
    PACKET_TYPE_CONTROL = 0x02
    PACKET_TYPE_LOG = 0x03
    
    # Extended mode magic (length=12, type=0x0A, payload="UARTGWEX")
    EXTMODE_MAGIC = bytes([
        0x0C, 0x00,  # length
        0x0A, 0x00,  # type
        0x55, 0x41, 0x52, 0x54, # U A R T
        0x47, 0x57, 0x45, 0x58  # G W E X
    ])

    CTRL_CMD_SIZE = 16

    def __init__(self, port, baudrate, timeout=None, **kwargs):
        self.serial = serial.Serial(
                    port, baudrate=baudrate, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE, timeout=timeout)
        
        # Ensure ESP32 itself is not held in reset/bootloader mode
        self.serial.dtr = False
        self.serial.rts = False
        
        self.rx_buffer = bytearray()
        self._dtr_state = False
        self._rts_state = False
        self._current_config = None
        
        # Proactively activate extended mode
        self._activate_extended_mode()

    def _activate_extended_mode(self):
        log.debug(f"Activating ESP Gateway Extended Mode with magic: {self.EXTMODE_MAGIC.hex()}")
        # Flush any junk first
        self.serial.reset_input_buffer()
        self.serial.write(self.EXTMODE_MAGIC)
        self.serial.flush()
        
        # Wait a bit for ESP to process and clear its own internal state
        time.sleep(0.5)
        
        # HTML implementation requests config (12 bytes of 0s)
        self._request_config()
        time.sleep(0.1)
        
        # And also sets config with current baudrate
        self._update_baudrate(self.serial.baudrate)
        time.sleep(0.1)

    def _request_config(self):
        log.debug("Requesting ESP Gateway configuration...")
        # HTML requestConfig sends 12 bytes of 0
        payload = bytearray(12)
        pkt = self._build_packet(payload, self.PACKET_TYPE_CONFIG)
        self.serial.write(pkt)
        self.serial.flush()

    def _update_baudrate(self, baudRate):
        log.debug(f"Updating ESP Gateway baudrate to {baudRate}")
        # Build 12-byte payload from config structure
        payload = bytearray(12)
        
        # baud_rate at offset 0-3 (little-endian)
        payload[0] = baudRate & 0xFF
        payload[1] = (baudRate >> 8) & 0xFF
        payload[2] = (baudRate >> 16) & 0xFF
        payload[3] = (baudRate >> 24) & 0xFF
        
        # extended_mode at offset 11
        payload[11] = 0x01
        
        pkt = self._build_packet(payload, self.PACKET_TYPE_CONFIG)
        self.serial.write(pkt)
        self.serial.flush()

    @property
    def timeout(self):
        return self.serial.timeout

    @timeout.setter
    def timeout(self, value):
        self.serial.timeout = value

    @property
    def dtr(self):
        return self._dtr_state

    @dtr.setter
    def dtr(self, value):
        """
        Map DTR changes to Gateway Reset command.
        DTR is Active Low.
        DTR=True  -> Line Low  -> Reset Active   -> R:0
        DTR=False -> Line High -> Reset Inactive -> R:1
        """
        self._dtr_state = value
        cmd = b"R:0" if value else b"R:1"
        self._send_control_packet(cmd)

    @property
    def rts(self):
        return self._rts_state

    @rts.setter
    def rts(self, value):
        self._rts_state = value
        cmd = b"C:1" if value else b"C:0"
        self._send_control_packet(cmd)

    def close(self):
        self.serial.close()

    def flushInput(self):
        self.serial.reset_input_buffer()
        self.rx_buffer = bytearray()

    def _build_packet(self, payload, type):
        length = len(payload) + self.PACKET_HEADER_SIZE
        header = bytes([
            length & 0xFF,
            (length >> 8) & 0xFF,
            type & 0xFF,
            (type >> 8) & 0xFF
        ])
        return header + payload

    def write(self, data):
        # Fragment into 64k chunks if needed
        max_payload = 65530
        offset = 0
        total_written = 0
        while offset < len(data):
            chunk = data[offset:offset+max_payload]
            pkt = self._build_packet(chunk, self.PACKET_TYPE_DATA)
            # Only log small data or snippets to keep logs readable
            if len(chunk) < 32:
                log.debug(f"ESP TX data packet (len={len(chunk)}): {chunk.hex()}")
            else:
                log.debug(f"ESP TX data packet (len={len(chunk)}): {chunk.hex()[:64]}...")
            self.serial.write(pkt)
            offset += len(chunk)
            total_written += len(chunk)
        
        self.serial.flush()
        return total_written

    def send_break(self, duration=0.2):
        """
        Send Break condition via Control Packet.
        """
        total_bits = int(duration * 1000)
        cmd = f"B:{total_bits}"
        self._send_control_packet(cmd)
        time.sleep(duration)

    def _send_control_packet(self, command, flush=True, silent=False):
        if isinstance(command, str):
            command = command.encode('ascii')
            
        payload = bytearray(self.CTRL_CMD_SIZE)
        cmd_len = min(len(command), self.CTRL_CMD_SIZE)
        payload[:cmd_len] = command[:cmd_len]
        
        pkt = self._build_packet(payload, self.PACKET_TYPE_CONTROL)
        if not silent:
            log.debug(f"ESP TX control packet: {command} -> {pkt.hex()}")
        self.serial.write(pkt)
        if flush:
            self.serial.flush()

    def read(self, size=1):
        """
        Read 'size' bytes from the buffer. If buffer is empty, fetch more packets.
        """
        while len(self.rx_buffer) < size:
            if not self._process_rx_packet():
                break # Timeout
                
        ret = bytes(self.rx_buffer[:size])
        del self.rx_buffer[:size]
        return ret

    def _process_rx_packet(self):
        """
        Read one full packet from the serial port.
        Returns True if a packet was processed, False on timeout.
        """
        # 1. Read header (4 bytes)
        header = self.serial.read(self.PACKET_HEADER_SIZE)
        if len(header) < self.PACKET_HEADER_SIZE:
            return False

        length = header[0] | (header[1] << 8)
        type = header[2] | (header[3] << 8)
        
        log.debug(f"ESP RX packet header: len={length}, type={type:04x}")

        if length < 4:
            # Invalid packet length, should not happen. 
            # We might need to resync if this occurs, but for now just return.
            return False
            
        # 2. Read payload
        payload_len = length - self.PACKET_HEADER_SIZE
        payload = self.serial.read(payload_len)
        if len(payload) < payload_len:
            # Incomplete packet
            return False

        # 3. Process payload based on type
        if type == self.PACKET_TYPE_DATA:
            log.debug(f"ESP RX data packet (len={len(payload)}): {payload.hex()}")
            self.rx_buffer.extend(payload)
        elif type == self.PACKET_TYPE_LOG:
            try:
                log_msg = payload.decode('utf-8', errors='replace')
                sys.stderr.write(f"\n[ESP32] {log_msg}") 
                if not log_msg.endswith('\n'):
                    sys.stderr.write('\n')
            except:
                pass
        elif type == self.PACKET_TYPE_CONFIG:
            log.debug(f"ESP RX config packet: {payload.hex()}")
            self._current_config = payload
        else:
            log.warning(f"ESP RX unknown packet type {type:04x}: {payload.hex()}")
            # Treat unknown as data for robustness? Or just discard?
            # User request says "make more robust", usually means don't crash on unknowns.
            self.rx_buffer.extend(payload)
            
        return True
