"""Decode the MAVLink v2 stream the firmware sends on USART1 (hex lines of a
`sim run --output` file) into a table: per message, count and last values; plus
STATUSTEXT lines. CRC is checked with gcs_peer's CRC_EXTRA table.
usage: python3 view.py out/out.txt"""
import re, struct, sys
from gcs_peer import CRC_EXTRA, NAMES, x25

# id: (struct of the full, untruncated payload, field names)
FMT = {
    0: ("<IBBBBB", "custom_mode type autopilot base_mode system_status mavlink_version"),
    24: ("<QiiiHHHHBB", "time_usec lat lon alt eph epv vel cog fix_type sats"),
    27: ("<Q9h", "time_usec xacc yacc zacc xgyro ygyro zgyro xmag ymag zmag"),
    29: ("<IffH", "time_ms press_hPa press_diff temp_cdegC"),
    30: ("<I6f", "time_ms roll pitch yaw rollspeed pitchspeed yawspeed"),
    36: ("<I8HB", "time_us s1 s2 s3 s4 s5 s6 s7 s8 port"),
    65: ("<I18HBB", "time_ms " + " ".join(f"c{k}" for k in range(1, 19)) + " chancount rssi"),
    132: ("<IHHHBBBB", "time_ms min_cm max_cm cur_cm type id orient cov"),
}


def frames(path):
    """Yield (virtual time, message id, payload) for every CRC-valid frame."""
    data, t = bytearray(), []
    for l in open(path):
        m = re.match(r"\[([0-9.]+)s\] \((?:UART1|usart1)\b[^)]*\) ([0-9A-F ]+)$", l.strip())
        if m:
            b = bytes.fromhex(m[2].replace(" ", ""))
            data += b
            t += [float(m[1])] * len(b)
    i = 0
    while i < len(data) - 12:
        if data[i] != 0xFD:
            i += 1
            continue
        ln = data[i + 1]
        mid = data[i + 7] | data[i + 8] << 8 | data[i + 9] << 16
        if i + 12 + ln > len(data):
            break
        crc = data[i + 10 + ln] | data[i + 11 + ln] << 8
        if mid in CRC_EXTRA and x25(bytes(data[i + 1:i + 10 + ln]) + bytes([CRC_EXTRA[mid]])) == crc:
            yield t[i], mid, bytes(data[i + 10:i + 10 + ln])
            i += 12 + ln
        else:
            i += 1


def decode(mid, p):
    f, names = FMT[mid]
    p = p + bytes(max(0, struct.calcsize(f) - len(p)))  # MAVLink v2 strips trailing zeros
    return dict(zip(names.split(), struct.unpack_from(f, p)))


def summary(d, mid):
    if mid == 24:
        return f"lat {d['lat']/1e7:.4f} lon {d['lon']/1e7:.4f} alt {d['alt']/1e3:.0f} m fix {d['fix_type']} sats {d['sats']}"
    if mid == 30:
        return f"roll {d['roll']*57.2958:.1f} pitch {d['pitch']*57.2958:.1f} yaw {d['yaw']*57.2958:.1f} deg"
    if mid == 29:
        return f"{d['press_hPa']:.1f} hPa {d['temp_cdegC']/100:.2f} C"
    if mid == 36:
        return "servo1-4 " + " ".join(str(d[f's{k}']) for k in range(1, 5)) + " us"
    if mid == 65:
        return "ch1-8 " + " ".join(str(d[f'c{k}']) for k in range(1, 9))
    if mid == 132:
        return f"{d['cur_cm']/100:.2f} m"
    if mid == 0:
        return f"base_mode 0x{d['base_mode']:02x} ({'ARMED' if d['base_mode'] & 0x80 else 'disarmed'})"
    if mid == 27:
        return f"acc {d['xacc']} {d['yacc']} {d['zacc']} gyro {d['xgyro']} {d['ygyro']} {d['zgyro']}"
    return ""


if __name__ == "__main__":
    n, last, text = {}, {}, []
    for t, mid, p in frames(sys.argv[1]):
        n[mid] = n.get(mid, 0) + 1
        if mid == 253:
            text.append((t, p[1:].split(b"\0")[0].decode(errors="replace")))
        elif mid in FMT:
            last[mid] = (t, decode(mid, p))
    print(f"{'id':>4} {'message':<17}{'frames':>7}  last value")
    for mid in sorted(n):
        v = ""
        if mid in last:
            t, d = last[mid]
            v = f"{summary(d, mid)}  @{t:.3f}s"
        print(f"{mid:>4} {NAMES.get(mid, '?'):<17}{n[mid]:>7}  {v}")
    print("\nSTATUSTEXT")
    for t, s in text[:40]:
        print(f"  {t:8.3f}s  {s}")
    print(f"\n{sum(n.values())} valid frames")
