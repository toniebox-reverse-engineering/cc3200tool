import serial
import time
import sys
import logging

log = logging.getLogger()

class EspSerial:
    # LOG packet constants
    LOG_MAGIC = bytes([0x4C, 0x4F, 0x47, 0x4D, 0x53, 0x47, 0x01, 0x00])
    LOG_HEADER_SIZE = len(LOG_MAGIC) + 4 + len(LOG_MAGIC)
    LOG_INV_OFFSET = len(LOG_MAGIC) + 4
    LOGMSG_MAX_LEN = 256

    # CONTROL packet constants
    CTRL_MAGIC = bytes([0x43, 0x54, 0x52, 0x4C, 0x47, 0x57, 0x01, 0x00])
    CTRL_PACKET_SIZE = len(CTRL_MAGIC) + 16 + len(CTRL_MAGIC)
    CTRL_INV_OFFSET = len(CTRL_MAGIC) + 16
    CTRL_PADDED_SIZE = 64

    def __init__(self, port, baudrate, timeout=None, **kwargs):
        # inter_byte_timeout ensures reads return quickly once data stops arriving,
        # instead of blocking until the full read_size is received.
        self.serial = serial.Serial(
            port, baudrate=baudrate, timeout=timeout or 10,
            inter_byte_timeout=0.1, **kwargs)
        # Ensure ESP32 itself is not held in reset/bootloader mode
        self.serial.dtr = True
        self.serial.rts = False
        self.rx_buffer = bytearray()
        self._internal_buffer = bytearray()
        self._dtr_state = True
        self._rts_state = False

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
        """
        Ignore RTS changes (SOP2) as requested by user.
        """
        self._rts_state = value
        # No-op

    def close(self):
        self.serial.close()

    def flushInput(self):
        self.serial.reset_input_buffer()
        self.rx_buffer = bytearray()
        self._internal_buffer = bytearray()

    def write(self, data):
        # log.info(f"ESP TX Raw: {data.hex()}")
        # Check for control packets (reset/break) vs data
        # Simply log snippet
        if len(data) > 0:
             log.info(f"ESP TX Raw: {data.hex()[:50]}...")
        ret = self.serial.write(data)
        self.serial.flush()
        return ret

    def send_break(self, duration=0.25):
        """
        Send Break condition via Control Packet.
        B:255 sends 255 bit-times of break at the current baud rate.
        """
        self._send_control_packet(b"B:255")
        time.sleep(duration)

    def _pad_to_64(self, packet):
        if len(packet) >= self.CTRL_PADDED_SIZE:
            return packet
        return packet + b'\x00' * (self.CTRL_PADDED_SIZE - len(packet))

    def _build_control_packet(self, command):
        # Command should be bytes
        if isinstance(command, str):
            command = command.encode('ascii')
            
        packet = bytearray(self.CTRL_PACKET_SIZE)
        packet[0:len(self.CTRL_MAGIC)] = self.CTRL_MAGIC
        
        cmd_len = min(len(command), 16)
        packet[len(self.CTRL_MAGIC):len(self.CTRL_MAGIC)+cmd_len] = command[:cmd_len]
        
        # Inverted Magic
        for i in range(len(self.CTRL_MAGIC)):
            packet[self.CTRL_INV_OFFSET + i] = self.CTRL_MAGIC[i] ^ 0xFF
            
        return self._pad_to_64(packet)

    def _send_control_packet(self, command):
        pkt = self._build_control_packet(command)
        #log.info(f"ESP CTRL TX: {command} -> {pkt.hex()}")
        self.serial.write(pkt)

    def read(self, size=1):
        """
        Read 'size' bytes from the stream, filtering out LOG packets.
        """
        start_time = time.time()
        
        while len(self.rx_buffer) < size:
            # Calculate remaining timeout
            remaining = None
            if self.serial.timeout is not None:
                elapsed = time.time() - start_time
                remaining = self.serial.timeout - elapsed
                if remaining <= 0:
                    break
            
            # Read chunk
            read_size = size - len(self.rx_buffer) + 64  # small over-read for LOG headers
            # Use original timeout for the read call if no specialized timeout logic needed
            # but we need to respect the overall timeout for the `read` operation.
            # serial.read respects its timeout. If we loop, we need to adjust.
            
            # Save original timeout
            orig_timeout = self.serial.timeout
            if remaining is not None:
                self.serial.timeout = remaining
                
            chunk = self.serial.read(read_size)
            
            # Restore timeout
            self.serial.timeout = orig_timeout
            
            if not chunk:
                break
                
            try:
                log.info(f"ESP RX Raw: {chunk.hex()[:100]}{'...' if len(chunk)>50 else ''}")
            except:
                pass
            self._process_rx_chunk(chunk)
            
        ret = bytes(self.rx_buffer[:size])
        del self.rx_buffer[:size]
        # if ret:
        #    log.debug(f"ESP Read Returning: {ret.hex()}")
        return ret


    def _check_magic(self, buffer, offset, magic, inverted):
        for i in range(len(magic)):
            expected = (magic[i] ^ 0xFF) if inverted else magic[i]
            if buffer[offset + i] != expected:
                return False
        return True

    def _process_rx_chunk(self, chunk):
        self._internal_buffer.extend(chunk)
        
        while True:
            # Look for magic
            # Only search for magic if we have enough data to potentially be confused
            # Optimization: simple find
            
            magic_idx = self._internal_buffer.find(self.LOG_MAGIC)
            
            if magic_idx == -1:
                # No complete magic found.
                # Check for partial match at the end
                buf_len = len(self._internal_buffer)
                if buf_len == 0:
                    break
                    
                match_len = 0
                # Check trailing bytes against start of LOG_MAGIC
                # Max overlap is len(LOG_MAGIC)-1
                for i in range(min(buf_len, len(self.LOG_MAGIC)-1), 0, -1):
                    if self._internal_buffer[-i:] == self.LOG_MAGIC[:i]:
                        match_len = i
                        break
                
                # Available data is everything up to the potential start of magic
                avail_len = buf_len - match_len
                if avail_len > 0:
                    self.rx_buffer.extend(self._internal_buffer[:avail_len])
                    del self._internal_buffer[:avail_len]
                
                # We kept the partial match. Done processing for now.
                break

            if magic_idx > 0:
                # We have data before the magic
                self.rx_buffer.extend(self._internal_buffer[:magic_idx])
                del self._internal_buffer[:magic_idx]
                continue
            
            # magic_idx == 0. Starts with LOG_MAGIC.
            # Check we have enough for header
            if len(self._internal_buffer) < self.LOG_HEADER_SIZE:
                break # Wait for more data
                
            # Verify inverted magic (robustness check)
            if not self._check_magic(self._internal_buffer, self.LOG_INV_OFFSET, self.LOG_MAGIC, True):
                # Not a valid packet (maybe coincidentally matched magic bytes, or corrupt)
                # Discard 1 byte and retry search
                # log.warning("Invalid LOG packet magic, skipping 1 byte")
                self.rx_buffer.append(self._internal_buffer[0])
                del self._internal_buffer[0]
                continue
                
            # Parse length
            payload_len = (self._internal_buffer[8] | 
                          (self._internal_buffer[9] << 8) | 
                          (self._internal_buffer[10] << 16) | 
                          (self._internal_buffer[11] << 24))
            
            # Sanity check length
            if payload_len < 0 or payload_len > self.LOGMSG_MAX_LEN:
                 # Invalid length
                 log.warning(f"Invalid LOG packet length: {payload_len}")
                 self.rx_buffer.append(self._internal_buffer[0])
                 del self._internal_buffer[0]
                 continue
                 
            total_len = self.LOG_HEADER_SIZE + payload_len
            if len(self._internal_buffer) < total_len:
                break # Wait for more data
                
            # Extract Packet
            msg_bytes = self._internal_buffer[self.LOG_HEADER_SIZE : total_len]
            try:
                # Log to stderr
                log_msg = msg_bytes.decode('utf-8', errors='replace')
                sys.stderr.write(f"\\r[ESP32] {log_msg}") 
                if not log_msg.endswith('\\n'):
                    sys.stderr.write('\\n')
            except Exception:
                pass
                
            # Consume packet
            del self._internal_buffer[:total_len]
