#!/usr/bin/env python3
"""
ads_client.py
PC side for the ESP32 + ADS131E04 firmware (protocol described in main/proto.h).

The ESP32 waits for a START frame, acquires for the requested time and streams
the samples back. This script sends START, receives and checks every frame
(CRC, sequence, sample index), converts to volts and writes a CSV.

Requirements: Python 3.8+, pyserial  (pip install pyserial)

Examples
  # 5 s, 1 kSPS, all 4 channels, gain 1
  python ads_client.py --port /dev/ttyUSB0 --rate 1000 --duration 5

  # 2 s at 4 kSPS, CH1 and CH3 only, gain 8 on both, status word included
  python ads_client.py --port COM5 --rate 4000 --duration 2 --channels 1,3 --gain 8 --status

  # bring up check: internal test signal on all channels
  python ads_client.py --port /dev/ttyUSB0 --rate 1000 --duration 1 --mux test

  # continuous until Ctrl+C
  python ads_client.py --port /dev/ttyUSB0 --rate 2000 --duration 0

  # board info / register dump
  python ads_client.py --port /dev/ttyUSB0 --info
  python ads_client.py --port /dev/ttyUSB0 --regs
"""

import argparse
import binascii
import csv
import datetime as _dt
import struct
import sys
import time

# --------------------------------------------------------------------------
# Protocol constants (must match main/proto.h)
# --------------------------------------------------------------------------
SYNC0, SYNC1 = 0xA5, 0x5A
HDR_LEN, CRC_LEN = 5, 2
MAX_RX_PAYLOAD = 4096

CMD_PING, CMD_START, CMD_STOP, CMD_READ_REGS = 0x01, 0x02, 0x03, 0x04
RSP_ACK, RSP_INFO, RSP_REGS, RSP_DATA, RSP_END = 0x81, 0x82, 0x83, 0x90, 0x91

STATUS_NAMES = {
    0: "OK",
    1: "BAD_FRAME (CRC error on the ESP32 side)",
    2: "BAD_LEN",
    3: "BAD_PARAM",
    4: "BUSY",
    5: "BANDWIDTH (rate x channels too high for the UART baud rate)",
    6: "ADC_FAIL (ADS131E04 not answering or register readback mismatch)",
    7: "NOT_RUNNING",
    8: "UNKNOWN_CMD",
}
END_REASONS = {0: "completed", 1: "stopped by PC", 2: "DRDY timeout (ADC not converting)"}
STATE_NAMES = {0: "idle", 1: "busy", 2: "booting"}

START_FLAG_OFFSETCAL = 0x01
START_FLAG_STATUS = 0x02
DATA_FLAG_STATUS = 0x02

RATE_TO_DR = {32000: 1, 16000: 2, 8000: 3, 4000: 4, 2000: 5, 1000: 6}
VALID_GAINS = (1, 2, 4, 8, 12)
MUX_CODES = {"normal": 0, "shorted": 1, "mvdd": 3, "temp": 4, "test": 5}
TEST_FREQ_CODES = {"slow": 0, "fast": 1, "dc": 3}   # fCLK/2^21, fCLK/2^20, dc

REG_NAMES = [
    "ID", "CONFIG1", "CONFIG2", "CONFIG3", "FAULT",
    "CH1SET", "CH2SET", "CH3SET", "CH4SET", "CH5SET", "CH6SET", "CH7SET", "CH8SET",
    "RSV_0D", "RSV_0E", "RSV_0F", "RSV_10", "RSV_11",
    "FAULT_STATP", "FAULT_STATN", "GPIO",
]


# --------------------------------------------------------------------------
# Framing
# --------------------------------------------------------------------------
def crc16(data, crc=0xFFFF):
    """CRC16 CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
    binascii.crc_hqx is exactly this CRC (implemented in C, fast)."""
    return binascii.crc_hqx(bytes(data), crc)


def build_frame(ftype, payload=b""):
    body = struct.pack("<BH", ftype, len(payload)) + bytes(payload)
    return bytes((SYNC0, SYNC1)) + body + struct.pack("<H", crc16(body))


class FrameParser:
    """Byte stream -> frames. Resynchronizes on garbage (boot messages etc.)."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0
        self.garbage_bytes = 0

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(bytes((SYNC0, SYNC1)))
            if i < 0:
                # keep a trailing 0xA5, it may be the first half of a sync
                keep = 1 if self.buf[-1:] == bytes((SYNC0,)) else 0
                self.garbage_bytes += len(self.buf) - keep
                del self.buf[: len(self.buf) - keep]
                break
            if i > 0:
                self.garbage_bytes += i
                del self.buf[:i]
            if len(self.buf) < HDR_LEN:
                break
            ftype = self.buf[2]
            length = self.buf[3] | (self.buf[4] << 8)
            if length > MAX_RX_PAYLOAD:
                del self.buf[:1]          # not a real header, skip the sync byte
                self.garbage_bytes += 1
                continue
            total = HDR_LEN + length + CRC_LEN
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2: HDR_LEN + length])
            rx_crc = self.buf[HDR_LEN + length] | (self.buf[HDR_LEN + length + 1] << 8)
            if crc16(body) != rx_crc:
                self.bad_crc += 1
                del self.buf[:1]          # resync from the next byte
                continue
            frames.append((ftype, bytes(self.buf[HDR_LEN: HDR_LEN + length])))
            del self.buf[:total]
        return frames


# --------------------------------------------------------------------------
# Payloads
# --------------------------------------------------------------------------
def build_start_payload(dr, ch_mask, gains, muxes, duration_ms, flags, test_cfg):
    """16 bytes, see proto.h CMD_START."""
    if len(gains) != 4 or len(muxes) != 4:
        raise ValueError("gains and muxes need 4 entries")
    return struct.pack("<BB4B4BIBB", dr, ch_mask, *gains, *muxes, duration_ms, flags, test_cfg)


def parse_info(p):
    fw, adc_id, adc_ok, baud, spi_hz, vref_mv, state = struct.unpack("<HBBIIHB", p[:15])
    return {
        "fw": "%d.%d" % (fw >> 8, fw & 0xFF),
        "adc_id": adc_id,
        "adc_ok": bool(adc_ok),
        "baud": baud,
        "spi_hz": spi_hz,
        "vref_mv": vref_mv,
        "state": state,
    }


def parse_ack(p):
    cmd, status = p[0], p[1]
    out = {"cmd": cmd, "status": status}
    if cmd == CMD_START and status == 0 and len(p) >= 17:
        rate, n_samples, mask, bpc, flags, spp, cal, vref_mv = struct.unpack("<IIBBBBBH", p[2:17])
        out.update(rate=rate, n_samples=n_samples, ch_mask=mask, bytes_per_ch=bpc,
                   flags=flags, spp=spp, calibrated=bool(cal), vref_mv=vref_mv)
    return out


def parse_end(p):
    reason, drdy, sent, missed, dropped, status_err, packets = struct.unpack("<BIIIIII", p[:25])
    return {"reason": reason, "drdy_count": drdy, "samples_sent": sent, "missed": missed,
            "dropped": dropped, "status_err": status_err, "packets": packets}


def parse_data(p):
    """Returns (seq, first_index, ch_mask, bytes_per_ch, records).
    records: list of (status_or_None, [codes...]) with signed integer codes."""
    seq, first, n, mask, bpc, flags = struct.unpack_from("<HIBBBB", p, 0)
    nch = bin(mask & 0x0F).count("1")
    has_status = bool(flags & DATA_FLAG_STATUS)
    rec_len = (3 if has_status else 0) + nch * bpc
    expected = 10 + n * rec_len
    if len(p) != expected:
        raise ValueError("DATA length %d, expected %d" % (len(p), expected))
    records = []
    off = 10
    for _ in range(n):
        status = None
        if has_status:
            status = int.from_bytes(p[off: off + 3], "big")
            off += 3
        codes = []
        for _c in range(nch):
            codes.append(int.from_bytes(p[off: off + bpc], "big", signed=True))
            off += bpc
        records.append((status, codes))
    return seq, first, mask, bpc, records


# --------------------------------------------------------------------------
# Link helpers
# --------------------------------------------------------------------------
class Link:
    def __init__(self, ser):
        self.ser = ser
        self.parser = FrameParser()
        self.pending = []

    def send(self, ftype, payload=b""):
        self.ser.write(build_frame(ftype, payload))
        self.ser.flush()

    def poll(self):
        """Returns the frames received so far (non blocking-ish, uses the port timeout)."""
        data = self.ser.read(max(1, self.ser.in_waiting))
        if data:
            self.pending.extend(self.parser.feed(data))
        out, self.pending = self.pending, []
        return out

    def wait_for(self, want_type, timeout, want_cmd=None, keep_others=None):
        """Waits for a frame of want_type (and for ACK, of the given cmd)."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            for ftype, payload in self.poll():
                if ftype == want_type and (want_cmd is None or (payload and payload[0] == want_cmd)):
                    return payload
                if keep_others is not None:
                    keep_others.append((ftype, payload))
        return None


def open_port(port, baud, reset):
    import serial  # pyserial

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    # DevKitC auto program circuit: DTR -> GPIO0, RTS -> EN.
    # Keep both released so opening the port does not reset or stop the ESP32.
    ser.dtr = False
    ser.rts = False
    ser.open()
    if reset:
        ser.rts = True        # EN low
        time.sleep(0.1)
        ser.rts = False       # EN high, GPIO0 high -> normal boot
    return ser


def wait_ready(link, timeout=8.0):
    """PING until the firmware reports idle. Stops a run left over from before."""
    t_end = time.monotonic() + timeout
    last_ping = 0.0
    info = None
    while time.monotonic() < t_end:
        now = time.monotonic()
        if now - last_ping > 0.3:
            link.send(CMD_PING)
            last_ping = now
        for ftype, payload in link.poll():
            if ftype == RSP_INFO:
                info = parse_info(payload)
                if info["state"] == 0:
                    return info
                if info["state"] == 1:
                    link.send(CMD_STOP)
    return info


# --------------------------------------------------------------------------
# Acquisition
# --------------------------------------------------------------------------
def acquire(link, start_payload, gains, out_path, raw=False, quiet=False):
    link.send(CMD_START, start_payload)
    # ACK can take ~0.4 s (OFFSETCAL) or ~1.5 s if the ADC is re-initialized
    ack_payload = link.wait_for(RSP_ACK, 4.0, want_cmd=CMD_START)
    if ack_payload is None:
        raise RuntimeError("no ACK to START (check port, baud rate, firmware)")
    ack = parse_ack(ack_payload)
    if ack["status"] != 0:
        raise RuntimeError("START refused: " + STATUS_NAMES.get(ack["status"], str(ack["status"])))

    rate = ack["rate"]
    mask = ack["ch_mask"]
    bpc = ack["bytes_per_ch"]
    vref = ack["vref_mv"] / 1000.0
    channels = [i for i in range(4) if mask & (1 << i)]
    full_scale = (1 << (8 * bpc - 1)) - 1                   # 2^23 - 1 or 2^15 - 1
    lsb = [vref / gains[c] / full_scale for c in channels]  # volts per code
    has_status = bool(ack["flags"] & START_FLAG_STATUS)

    if not quiet:
        dur = "continuous (Ctrl+C to stop)" if ack["n_samples"] == 0 else \
            "%d samples (%.3f s)" % (ack["n_samples"], ack["n_samples"] / rate)
        print("START ok: %d SPS, %d bit, channels %s, %s, offset cal %s" % (
            rate, 8 * bpc, ",".join("CH%d" % (c + 1) for c in channels), dur,
            "done" if ack["calibrated"] else "kept"))

    header = ["sample", "time_s"] + ["CH%d_V" % (c + 1) for c in channels]
    if raw:
        header += ["CH%d_code" % (c + 1) for c in channels]
    if has_status:
        header += ["status"]

    stats = {"packets": 0, "samples": 0, "seq_gaps": 0, "index_gaps": 0, "lost_samples": 0}
    state = {"expected_seq": None, "next_index": 0, "end": None}

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)

        def handle(frames):
            for ftype, payload in frames:
                if ftype == RSP_DATA:
                    seq, first, _m, _b, records = parse_data(payload)
                    if state["expected_seq"] is not None and seq != state["expected_seq"]:
                        stats["seq_gaps"] += 1
                    state["expected_seq"] = (seq + 1) & 0xFFFF
                    if first != state["next_index"]:
                        stats["index_gaps"] += 1
                        stats["lost_samples"] += first - state["next_index"]
                    for k, (status, codes) in enumerate(records):
                        idx = first + k
                        row = [idx, "%.7f" % (idx / rate)]
                        row += ["%.9e" % (code * lsb[j]) for j, code in enumerate(codes)]
                        if raw:
                            row += codes
                        if has_status:
                            row.append("0x%06X" % status)
                        w.writerow(row)
                    state["next_index"] = first + len(records)
                    stats["packets"] += 1
                    stats["samples"] += len(records)
                elif ftype == RSP_END:
                    state["end"] = parse_end(payload)
                # ACK to STOP and unsolicited INFO frames are ignored here

        stop_sent = False
        last_rx = time.monotonic()
        try:
            while state["end"] is None:
                frames = link.poll()
                now = time.monotonic()
                if frames:
                    last_rx = now
                elif now - last_rx > 2.0:
                    if stop_sent:
                        raise RuntimeError("no END frame from the ESP32")
                    print("No data for 2 s, sending STOP", file=sys.stderr)
                    link.send(CMD_STOP)
                    stop_sent = True
                    last_rx = now
                handle(frames)
        except KeyboardInterrupt:
            print("\nCtrl+C: stopping acquisition...", file=sys.stderr)
            link.send(CMD_STOP)
            t_end = time.monotonic() + 3.0
            while state["end"] is None and time.monotonic() < t_end:
                handle(link.poll())

    return ack, state["end"], stats


def print_summary(ack, end, stats, link, out_path):
    print("Saved %d samples to %s" % (stats["samples"], out_path))
    if end is None:
        print("WARNING: END frame not received")
        return
    print("END: %s | conversions %d, sent %d, missed %d, dropped %d, status errors %d, packets %d" % (
        END_REASONS.get(end["reason"], str(end["reason"])), end["drdy_count"], end["samples_sent"],
        end["missed"], end["dropped"], end["status_err"], end["packets"]))
    problems = []
    if end["packets"] != stats["packets"]:
        problems.append("%d DATA frames lost on the link" % (end["packets"] - stats["packets"]))
    if end["samples_sent"] != stats["samples"]:
        problems.append("ESP32 sent %d samples, PC got %d" % (end["samples_sent"], stats["samples"]))
    if end["missed"]:
        problems.append("%d conversions not read in time by the ESP32 (lower the rate)" % end["missed"])
    if end["dropped"]:
        problems.append("%d samples dropped, TX buffers full (lower rate/channels or raise baud)" % end["dropped"])
    if end["status_err"]:
        problems.append("%d frames with a bad status word (SPI signal integrity?)" % end["status_err"])
    if link.parser.bad_crc:
        problems.append("%d frames with CRC errors on the PC side" % link.parser.bad_crc)
    if stats["seq_gaps"]:
        problems.append("%d sequence gaps" % stats["seq_gaps"])
    if problems:
        print("WARNINGS:\n  " + "\n  ".join(problems))
    else:
        print("Data integrity: OK (no lost, missed or corrupted samples)")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def per_channel(text, conv, name):
    items = [conv(x.strip()) for x in str(text).split(",") if x.strip()]
    if len(items) == 1:
        items *= 4
    if len(items) != 4:
        raise SystemExit("--%s needs 1 value or 4 comma separated values" % name)
    return items


def main(argv=None):
    ap = argparse.ArgumentParser(description="ESP32 + ADS131E04 acquisition client")
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyUSB0 or COM5")
    ap.add_argument("--baud", type=int, default=921600, help="must match PC_UART_BAUD in board.h")
    ap.add_argument("--rate", type=int, default=1000, choices=sorted(RATE_TO_DR), help="samples per second")
    ap.add_argument("--duration", type=float, default=1.0, help="seconds, 0 = until Ctrl+C")
    ap.add_argument("--channels", default="1,2,3,4", help="enabled channels, e.g. 1,3")
    ap.add_argument("--gain", default="1", help="PGA gain 1,2,4,8,12 (one value or four)")
    ap.add_argument("--mux", default="normal", help="normal|shorted|mvdd|temp|test (one or four)")
    ap.add_argument("--status", action="store_true", help="include the 24 bit status word per sample")
    ap.add_argument("--no-cal", action="store_true",
                    help="do not force OFFSETCAL (the firmware still runs it after a gain change)")
    ap.add_argument("--test-amp", type=int, default=1, choices=(1, 2), help="test signal amplitude x1 or x2")
    ap.add_argument("--test-freq", default="slow", choices=sorted(TEST_FREQ_CODES), help="test signal frequency")
    ap.add_argument("--raw", action="store_true", help="also write the raw ADC codes")
    ap.add_argument("--out", default=None, help="CSV file (default ads_<date>_<time>.csv)")
    ap.add_argument("--reset", action="store_true", help="reset the ESP32 through RTS before starting")
    ap.add_argument("--info", action="store_true", help="print board info and exit")
    ap.add_argument("--regs", action="store_true", help="dump the ADS131E04 registers and exit")
    args = ap.parse_args(argv)

    try:
        ch_list = sorted({int(x) for x in args.channels.split(",") if x.strip()})
    except ValueError:
        raise SystemExit("--channels: use numbers 1..4, e.g. 1,2")
    if not ch_list or any(c < 1 or c > 4 for c in ch_list):
        raise SystemExit("--channels: use numbers 1..4, e.g. 1,2")
    ch_mask = 0
    for c in ch_list:
        ch_mask |= 1 << (c - 1)

    gains = per_channel(args.gain, int, "gain")
    if any(g not in VALID_GAINS for g in gains):
        raise SystemExit("--gain: allowed values %s" % (VALID_GAINS,))
    try:
        muxes = per_channel(args.mux, lambda s: MUX_CODES[s.lower()], "mux")
    except KeyError:
        raise SystemExit("--mux: allowed values %s" % ", ".join(MUX_CODES))

    if args.duration < 0:
        raise SystemExit("--duration must be >= 0")
    duration_ms = int(round(args.duration * 1000))
    if args.duration > 0 and duration_ms == 0:
        duration_ms = 1
    if duration_ms > 0xFFFFFFFF:
        raise SystemExit("--duration too long")

    flags = (0 if args.no_cal else START_FLAG_OFFSETCAL) | (START_FLAG_STATUS if args.status else 0)
    test_cfg = (0x04 if args.test_amp == 2 else 0) | TEST_FREQ_CODES[args.test_freq]
    payload = build_start_payload(RATE_TO_DR[args.rate], ch_mask, gains, muxes, duration_ms, flags, test_cfg)

    out_path = args.out or _dt.datetime.now().strftime("ads_%Y%m%d_%H%M%S.csv")

    ser = open_port(args.port, args.baud, args.reset)
    try:
        link = Link(ser)
        info = wait_ready(link)
        if info is None:
            raise SystemExit("ESP32 not answering on %s at %d baud" % (args.port, args.baud))
        if info["state"] != 0:
            raise SystemExit("ESP32 is %s, try again" % STATE_NAMES.get(info["state"], "?"))
        if info["baud"] != args.baud:
            print("WARNING: firmware baud %d differs from --baud %d" % (info["baud"], args.baud))

        if args.info or not info["adc_ok"]:
            print("Firmware %s | ADC ID 0x%02X (%s) | UART %d baud | SPI %d Hz | VREF %d mV | %s" % (
                info["fw"], info["adc_id"], "ok" if info["adc_ok"] else "NOT RESPONDING",
                info["baud"], info["spi_hz"], info["vref_mv"], STATE_NAMES.get(info["state"], "?")))
            if args.info:
                return 0

        if args.regs:
            link.send(CMD_READ_REGS)
            regs = link.wait_for(RSP_REGS, 2.0)
            if regs is None:
                raise SystemExit("no register dump received")
            for i, v in enumerate(regs):
                print("0x%02X %-12s 0x%02X" % (i, REG_NAMES[i] if i < len(REG_NAMES) else "?", v))
            return 0

        ack, end, stats = acquire(link, payload, gains, out_path, raw=args.raw)
        print_summary(ack, end, stats, link, out_path)
        return 0
    except RuntimeError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 1
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
