"""Frame MAVLink v2 from the (UART1) hex lines of a sim --output file and check
each frame's CRC with the message's CRC_EXTRA; report per-id valid/invalid
counts and dump any invalid frames of the ids given on the command line."""
import re, sys

from gcs_peer import CRC_EXTRA, x25


d = bytearray()
for l in open(sys.argv[1]):
    m = re.match(r'\[[0-9.]+s\] \((UART1|usart1)\) ([0-9A-F ]+)$', l.strip())
    if m:
        d += bytes.fromhex(m[2].replace(' ', ''))
want = {int(a) for a in sys.argv[2:]}
i = 0
stats = {}
while i < len(d) - 12:
    if d[i] != 0xFD:
        i += 1
        continue
    ln = d[i + 1]
    mid = d[i + 7] | d[i + 8] << 8 | d[i + 9] << 16
    frame = d[i:i + 12 + ln]
    if len(frame) < 12 + ln:
        break
    body, crc = frame[1:10 + ln], frame[10 + ln] | frame[11 + ln] << 8
    ok = mid in CRC_EXTRA and x25(bytes(body) + bytes([CRC_EXTRA[mid]])) == crc
    s = stats.setdefault(mid, [0, 0])
    s[0 if ok else 1] += 1
    if mid in want and not ok:
        print(f"BAD id={mid} len={ln} seq={frame[4]} payload={bytes(frame[10:10 + ln]).hex()}")
    i += 12 + ln if ok else 1
for mid, (ok, bad) in sorted(stats.items()):
    print(f"id {mid}: ok={ok} bad={bad}")
