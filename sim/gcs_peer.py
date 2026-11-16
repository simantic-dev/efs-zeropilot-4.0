# Ground station on ZeroPilot's telemetry link (USART1, RFD900 at 57600 8N1,
# boardfiles/model/src/drivers.cpp). Frames MAVLink v2 off the wire, checks
# each frame's X.25 CRC with the message's CRC_EXTRA, and logs the first valid
# frame of every message id, so a fixture can expect "[gcs] MAVLINK ATTITUDE".
# A frame whose CRC fails is logged as a warning.
CRC_EXTRA = {0: 50, 1: 124, 24: 24, 27: 144, 29: 115, 30: 39, 36: 222, 65: 118, 132: 85, 147: 154, 253: 83}


def x25(data, crc=0xFFFF):
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


NAMES = {0: "HEARTBEAT", 1: "SYS_STATUS", 24: "GPS_RAW_INT", 27: "RAW_IMU", 29: "SCALED_PRESSURE",
         30: "ATTITUDE", 36: "SERVO_OUTPUT_RAW", 65: "RC_CHANNELS",
         132: "DISTANCE_SENSOR", 147: "BATTERY_STATUS", 253: "STATUSTEXT"}


def parse(buf: bytearray):
    """Pop complete MAVLink v2 frames off the front of buf; yield (id, ok)."""
    while True:
        start = buf.find(0xFD)
        if start < 0:
            buf.clear()
            return
        del buf[:start]
        if len(buf) < 12 or len(buf) < 12 + buf[1]:
            return
        ln = buf[1]
        mid = buf[7] | buf[8] << 8 | buf[9] << 16
        crc = buf[10 + ln] | buf[11 + ln] << 8
        ok = mid in CRC_EXTRA and x25(bytes(buf[1:10 + ln]) + bytes([CRC_EXTRA[mid]])) == crc
        payload = bytes(buf[10:10 + ln])
        # A false 0xFD start is resynced one byte later; a good frame is consumed whole.
        del buf[:12 + ln if ok else 1]
        yield mid, ok, payload


class Peer:
    def __init__(self, ctx):
        self.ctx = ctx
        self.buf = bytearray()
        self.seen = set()
        self.armed = False

    def on_byte(self, b):
        self.buf.append(b)
        for mid, ok, payload in parse(self.buf):
            if not ok:
                self.ctx.warning(f"MAVLINK bad crc id={mid}")
                continue
            if mid not in self.seen:
                self.seen.add(mid)
                self.ctx.info(f"MAVLINK {NAMES.get(mid, mid)}")
            # HEARTBEAT base_mode (payload byte 6, zero-truncated on the wire): MAV_MODE_FLAG_SAFETY_ARMED.
            if mid == 0 and not self.armed and len(payload) > 6 and payload[6] & 0x80:
                self.armed = True
                self.ctx.info("MAVLINK ARMED")


if __name__ == "__main__":
    # ATTITUDE (id 30, 28-byte payload) framed by hand, then a byte of noise.
    body = bytes([28, 0, 0, 0, 1, 1, 30, 0, 0]) + bytes(28)
    crc = x25(body + bytes([CRC_EXTRA[30]]))
    buf = bytearray(b"\x00" + b"\xfd" + body + crc.to_bytes(2, "little"))
    assert list(parse(buf)) == [(30, True, bytes(28))] and not buf
    print("ok")
