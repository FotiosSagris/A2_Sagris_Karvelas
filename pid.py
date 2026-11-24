import threading
import time
from collections import deque
import serial

CMD_ANALOG_DATA = 0xE0
CMD_ENABLE_ANALOG = 0xC0
CMD_PIN_MODE = 0xF4
CMD_FW_VERSION = 0xF9
SYSEX_BEGIN = 0xF0
SYSEX_END = 0xF7
SYSEX_ANALOG_MAP_REQ = 0x69
SYSEX_ANALOG_MAP_RSP = 0x6A
SYSEX_CAP_REQ = 0x6B
SYSEX_CAP_RSP = 0x6C
SYSEX_EXT_ANALOG = 0x6F
SYSEX_SAMPLE_RATE = 0x7A

PINMODE_ANALOG = 2
PINMODE_PWM = 3
U16_MAX = 65535


class FirmataLink:
    def __init__(self, port, baud=57600, sample_ms=5, handshake_timeout=10.0):
        self.port = port
        self._ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
        self.pin_caps = {}
        self.analog_to_pin = {}
        self.latest_analog = {}
        self._ring_buffers = {}
        self.fw_version = None

        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._active_cmd = None
        self._payload = []
        self._sysex_buf = None
        self._active = True

        self._reader = threading.Thread(target=self._poll_port, daemon=True)
        self._reader.start()
        self._wait_handshake(handshake_timeout)
        self.configure_sampling_ms(sample_ms)

    def _send_bytes(self, packet):
        with self._tx_lock:
            try:
                self._ser.write(bytes(packet))
            except serial.SerialException:
                pass

    def _wait_handshake(self, timeout):
        time.sleep(1.0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._send_bytes([CMD_FW_VERSION])
            self._send_bytes([SYSEX_BEGIN, SYSEX_CAP_REQ, SYSEX_END])
            self._send_bytes([SYSEX_BEGIN, SYSEX_ANALOG_MAP_REQ, SYSEX_END])
            attempt_end = time.monotonic() + 1.0
            while time.monotonic() < attempt_end:
                if self.pin_caps and self.analog_to_pin:
                    return
                time.sleep(0.02)
        self.close()
        raise RuntimeError(f"Firmata handshake failed on {self.port}")

    def _poll_port(self):
        while self._active:
            try:
                chunk = self._ser.read(256)
            except (serial.SerialException, OSError, TypeError):
                break
            for b in chunk:
                self._consume_byte(b)

    def _consume_byte(self, byte):
        if self._sysex_buf is not None:
            if byte == SYSEX_END:
                self._parse_sysex(self._sysex_buf)
                self._sysex_buf = None
            else:
                self._sysex_buf.append(byte)
            return

        if byte == SYSEX_BEGIN:
            self._sysex_buf = []
            return

        if byte & 0x80:
            self._active_cmd = byte
            self._payload = []
            return

        if self._active_cmd is None:
            return

        self._payload.append(byte)
        if len(self._payload) < 2:
            return

        if (self._active_cmd & 0xF0) == CMD_ANALOG_DATA:
            ch = self._active_cmd & 0x0F
            val = self._payload[0] | (self._payload[1] << 7)
            with self._rx_lock:
                self.latest_analog[ch] = val
                self._ring_buffers.setdefault(ch, deque(maxlen=64)).append(val)
        elif self._active_cmd == CMD_FW_VERSION:
            self.fw_version = f"{self._payload[0]}.{self._payload[1]}"

        self._payload = []

    def _parse_sysex(self, msg):
        if not msg:
            return
        msg_type = msg[0]
        if msg_type == SYSEX_CAP_RSP:
            caps = {}
            current_modes = {}
            idx = 1
            while idx < len(msg):
                if msg[idx] == 0x7F:
                    caps[len(caps)] = current_modes
                    current_modes = {}
                    idx += 1
                else:
                    if idx + 1 < len(msg):
                        current_modes[msg[idx]] = msg[idx + 1]
                    idx += 2
            self.pin_caps = caps
        elif msg_type == SYSEX_ANALOG_MAP_RSP:
            self.analog_to_pin = {
                ch: pin for pin, ch in enumerate(msg[1:]) if ch != 0x7F
            }

    def recent_samples(self, channel, count):
        with self._rx_lock:
            buf = self._ring_buffers.get(channel)
            return list(buf)[-count:] if buf else []

    def configure_sampling_ms(self, ms):
        self._send_bytes([
            SYSEX_BEGIN,
            SYSEX_SAMPLE_RATE,
            ms & 0x7F,
            (ms >> 7) & 0x7F,
            SYSEX_END,
        ])

    def set_mode(self, pin, mode):
        self._send_bytes([CMD_PIN_MODE, pin, mode])

    def toggle_analog_stream(self, channel, enabled=True):
        self._send_bytes([CMD_ENABLE_ANALOG | channel, 1 if enabled else 0])

    def write_analog_pin(self, pin, raw_val):
        if pin < 16 and raw_val < 0x4000:
            self._send_bytes([
                CMD_ANALOG_DATA | pin,
                raw_val & 0x7F,
                (raw_val >> 7) & 0x7F,
            ])
        else:
            self._send_bytes([
                SYSEX_BEGIN,
                SYSEX_EXT_ANALOG,
                pin,
                raw_val & 0x7F,
                (raw_val >> 7) & 0x7F,
                (raw_val >> 14) & 0x7F,
                SYSEX_END,
            ])

    def pin_bits(self, pin, mode, fallback_bits):
        return self.pin_caps.get(pin, {}).get(mode, fallback_bits)

    def close(self):
        self._active = False
        if self._reader.is_alive() and threading.current_thread() is not self._reader:
            self._reader.join(timeout=1.0)
        if self._ser.is_open:
            self._ser.close()


class AnalogChannel:
    def __init__(self, link, channel, wait_s=2.0):
        self.link = link
        self.channel = channel
        self.pin = link.analog_to_pin[channel]
        self.bits = link.pin_bits(self.pin, PINMODE_ANALOG, 10)
        self.max_code = (1 << self.bits) - 1

        link.set_mode(self.pin, PINMODE_ANALOG)
        link.toggle_analog_stream(channel, True)

        deadline = time.monotonic() + wait_s
        while channel not in link.latest_analog and time.monotonic() < deadline:
            time.sleep(0.01)

    def current_code(self):
        return self.link.latest_analog.get(self.channel, 0)

    def read_u16(self):
        return round(self.current_code() * U16_MAX / self.max_code)

    def read_avg(self, count):
        samples = self.link.recent_samples(self.channel, count)
        if not samples:
            return self.read_u16()
        mean_code = sum(samples) / len(samples)
        return round(mean_code * U16_MAX / self.max_code)


class PwmChannel:
    def __init__(self, link, pin, initial_u16=0):
        self.link = link
        self.pin = pin
        self.bits = link.pin_bits(pin, PINMODE_PWM, 8)
        self.max_code = (1 << self.bits) - 1
        self._duty_u16 = 0
        self._last_code = -1

        link.set_mode(pin, PINMODE_PWM)
        self.duty_u16(initial_u16)

    def duty_u16(self, val=None):
        if val is None:
            return self._duty_u16
        clamped = max(0, min(U16_MAX, int(val)))
        self._duty_u16 = clamped
        hw_code = round(clamped * self.max_code / U16_MAX)
        if hw_code != self._last_code:
            self._last_code = hw_code
            self.link.write_analog_pin(self.pin, hw_code)
        return clamped

    def deinit(self):
        self.duty_u16(0)


