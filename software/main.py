#!/usr/bin/env python3
"""
SpaceFlex: app desktop (pywebview) che carica login.html e index.html,
parla con la ESP32 sulla seriale e salva lo storico di ogni soggetto in JSON.

La ESP32 può usare due formati:
  - binario: frame 0xA5 0x5A + CRC16 (firmware ADS131E04)
  - testo:   una riga con il valore in mV per ogni campione
"""

import binascii
import re
import json
import os
from pathlib import Path
import struct
import sys
import threading
import time

# Va impostato prima di importare webview, altrimenti GTK è già inizializzato.
if sys.platform.startswith("linux"):
    os.environ.setdefault("GTK_THEME", "Adwaita:dark")

import serial
import serial.tools.list_ports
import webview

SYNC0, SYNC1 = 0xA5, 0x5A
HDR_LEN, CRC_LEN = 5, 2
MAX_RX_PAYLOAD = 4096

CMD_PING, CMD_START, CMD_STOP, CMD_READ_REGS = 0x01, 0x02, 0x03, 0x04
RSP_ACK, RSP_INFO, RSP_REGS, RSP_DATA, RSP_END = 0x81, 0x82, 0x83, 0x90, 0x91

START_FLAG_OFFSETCAL = 0x01

# Codici di stato dell'ACK (uart/main/proto.h)
ACK_STATUS = {
    1: "frame non valido (CRC)",
    2: "lunghezza del comando errata",
    3: "parametri non validi",
    4: "scheda occupata",
    5: "banda UART insufficiente per frequenza e canali richiesti",
    6: "l'ADS131E04 non risponde (alimentazione o collegamenti SPI)",
    7: "nessuna acquisizione in corso",
    8: "comando sconosciuto",
}
INFO_STATE = {0: "pronta", 1: "in acquisizione", 2: "in avvio"}
DATA_FLAG_STATUS = 0x02
RATE_TO_DR = {32000: 1, 16000: 2, 8000: 3, 4000: 4, 2000: 5, 1000: 6}


def get_base_dir():
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


def get_resource_uri(filename):
    if hasattr(sys, "_MEIPASS"):
        base = sys._MEIPASS
    else:
        base = os.path.dirname(os.path.abspath(__file__))
    return Path(os.path.join(base, filename)).resolve().as_uri()


def crc16(data, crc=0xFFFF):
    return binascii.crc_hqx(bytes(data), crc)


_NUM_LINE = re.compile(rb"^\s*[-+]?\d+(?:[.,]\d+)?\s*$")


def _parse_mv_lines(data):
    """Ritorna i numeri trovati nelle righe complete e l'ultima riga ancora a metà."""
    lines = re.split(rb"\r\n|\n|\r", data)
    rest = lines.pop() if lines else b""
    values = []
    for ln in lines:
        if _NUM_LINE.match(ln):
            try:
                values.append(float(ln.strip().replace(b",", b".")))
            except ValueError:
                pass
    return values, rest


class KalmanFilter1D:
    """Kalman scalare con modello a passeggiata casuale: il livello di attivazione
    del muscolo cambia lentamente, il rumore di misura è bianco.

    q = varianza del processo (quanto può muoversi il valore vero tra due letture)
    r = varianza della misura (quanto è rumorosa la singola lettura)
    Rapporto q/r più basso = uscita più liscia ma più lenta a seguire i cambi."""

    def __init__(self, q=1e-3, r=1e-2):
        self.q = q
        self.r = r
        self.reset()

    def reset(self):
        self.x = None
        self.p = 1.0

    def update(self, z):
        if self.x is None:
            # Prima lettura: la prendiamo come stima iniziale
            self.x = z
            self.p = self.r
            return z
        p_pred = self.p + self.q
        k = p_pred / (p_pred + self.r)
        self.x = self.x + k * (z - self.x)
        self.p = (1.0 - k) * p_pred
        return self.x


def build_frame(ftype, payload=b""):
    body = struct.pack("<BH", ftype, len(payload)) + bytes(payload)
    return bytes((SYNC0, SYNC1)) + body + struct.pack("<H", crc16(body))


def build_start_payload(dr, ch_mask, gains, muxes, duration_ms, flags, test_cfg=0):
    return struct.pack("<BB4B4BIBB", dr, ch_mask, *gains, *muxes, duration_ms, flags, test_cfg)


class FrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(bytes((SYNC0, SYNC1)))
            if i < 0:
                keep = 1 if self.buf[-1:] == bytes((SYNC0,)) else 0
                del self.buf[: len(self.buf) - keep]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < HDR_LEN:
                break
            ftype = self.buf[2]
            length = self.buf[3] | (self.buf[4] << 8)
            if length > MAX_RX_PAYLOAD:
                del self.buf[:1]
                continue
            total = HDR_LEN + length + CRC_LEN
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2 : HDR_LEN + length])
            rx_crc = self.buf[HDR_LEN + length] | (self.buf[HDR_LEN + length + 1] << 8)
            if crc16(body) != rx_crc:
                self.bad_crc += 1
                del self.buf[:1]
                continue
            frames.append((ftype, bytes(self.buf[HDR_LEN : HDR_LEN + length])))
            del self.buf[:total]
        return frames


class SpaceFlexBridge:
    def __init__(self):
        self._window = None
        self._ser = None
        self.port_name = None
        self.baud = 921600
        self._parser = FrameParser()
        self._lock = threading.Lock()
        self.acquiring = False
        self._reader_thread = None
        self._stop_reader = False
        self.active_patient = None
        self.mode = "binary"  # oppure "text", deciso in connect_adc
        self._kalman = KalmanFilter1D()

        self.data_dir = os.path.join(get_base_dir(), "SpaceFlex_Data")
        os.makedirs(self.data_dir, exist_ok=True)

        self.sample_rate = 1000
        self.channel_idx = 0
        self.gain = 12
        self.vref_v = 2.4

    def set_window(self, window):
        self._window = window

    # Navigazione tra le due pagine
    def open_console(self, patient_json_str=None):
        if patient_json_str:
            try:
                self.active_patient = json.loads(patient_json_str)
            except Exception:
                pass
        if self._window:
            self._window.load_url(get_resource_uri("index.html"))
        return {"ok": True}

    def open_login(self):
        self.stop_adc_acquisition()
        if self._window:
            self._window.load_url(get_resource_uri("login.html"))
        return {"ok": True}

    def toggle_fullscreen(self):
        if self._window:
            self._window.toggle_fullscreen()
        return {"ok": True}

    def get_active_patient(self):
        # index.html lo chiama appena carica: ne approfittiamo per riapplicare il CSS.
        # evaluate_js da qui bloccherebbe la risposta, quindi va in un thread.
        if self._window:
            threading.Thread(target=_inject_native_controls_css, args=(self._window,), daemon=True).start()
        return self.active_patient

    # Storico su file: un JSON per soggetto in SpaceFlex_Data/
    def _subject_filepath(self, first_name, last_name):
        f_clean = "".join(c for c in first_name.strip().lower() if c.isalnum() or c in ("_", "-")) or "guest"
        l_clean = "".join(c for c in last_name.strip().lower() if c.isalnum() or c in ("_", "-")) or "user"
        return os.path.join(self.data_dir, f"spaceflex_{f_clean}_{l_clean}.json")

    def save_subject_json(self, first_name, last_name, payload_json_str):
        try:
            path = self._subject_filepath(first_name, last_name)
            parsed = json.loads(payload_json_str)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(parsed, f, indent=2, ensure_ascii=False)
            return {"ok": True, "file": os.path.basename(path), "path": path}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def load_subject_json(self, first_name, last_name):
        try:
            path = self._subject_filepath(first_name, last_name)
            if not os.path.exists(path):
                return {"found": False, "file": os.path.basename(path)}
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return {"found": True, "file": os.path.basename(path), "data": data}
        except Exception as e:
            return {"found": False, "error": str(e)}

    # Seriale
    def list_serial_ports(self):
        return [
            {"device": p.device, "description": p.description, "vid": p.vid, "pid": p.pid}
            for p in serial.tools.list_ports.comports()
        ]

    @staticmethod
    def _candidate_ports():
        """Porte USB ordinate per probabilità. Le /dev/ttyS* restano fuori: su Linux
        danno quasi sempre 'Input/output error' e nascondono l'errore vero."""
        known_vids = {0x10C4: 0, 0x1A86: 0, 0x303A: 0, 0x0403: 1}  # CP210x, CH34x, Espressif, FTDI
        ports = []
        for p in serial.tools.list_ports.comports():
            dev = p.device or ""
            is_usb = p.vid is not None or "ttyUSB" in dev or "ttyACM" in dev
            if not is_usb:
                continue
            prio = known_vids.get(p.vid, 2)
            ports.append((prio, dev, p))
        ports.sort(key=lambda t: (t[0], t[1]))
        return [(dev, p) for _prio, dev, p in ports]

    def _probe_port(self, dev, baud):
        """Ritorna (ser, info, None) se la scheda risponde, altrimenti (None, None, motivo)."""
        ser = serial.Serial()
        ser.port = dev
        ser.baudrate = int(baud)
        ser.timeout = 0.05
        ser.dtr = False   # DTR basso = IO0 alto, quindi avvio normale e non bootloader
        ser.rts = False
        ser.open()
        try:
            # Impulso su RTS (collegato a EN sulle DevKit) per riavviare la scheda
            ser.rts = True
            time.sleep(0.12)
            ser.rts = False

            # Il log della ROM al boot non ci interessa
            boot = bytearray()
            t_boot = time.monotonic() + 1.6
            while time.monotonic() < t_boot:
                boot += ser.read(max(1, ser.in_waiting))
            ser.reset_input_buffer()

            parser = FrameParser()
            ser.write(build_frame(CMD_STOP))
            ser.flush()
            time.sleep(0.05)

            rx_total = 0
            sample = bytearray()
            textbuf = bytearray()
            last_state = None
            # Il firmware all'avvio aspetta 1 s le alimentazioni dell'ADC e 200 ms il
            # riferimento interno: in quel tempo risponde con stato "in avvio".
            t_end = time.monotonic() + 6.0
            while time.monotonic() < t_end:
                ser.write(build_frame(CMD_PING))
                ser.flush()
                time.sleep(0.2)
                raw = ser.read(max(1, ser.in_waiting))
                rx_total += len(raw)
                if len(sample) < 120:
                    sample += raw[: 120 - len(sample)]
                textbuf += raw
                if len(textbuf) > 2000:
                    del textbuf[:-2000]
                if len(_parse_mv_lines(bytes(textbuf))[0]) >= 5:
                    ser.reset_input_buffer()
                    return ser, {"fw": "TEXT", "adc_ok": True, "parser": FrameParser(), "mode": "text"}, None
                for ftype, payload in parser.feed(raw):
                    if ftype != RSP_INFO or len(payload) < 15:
                        continue
                    fw, _adc_id, a_ok, _b, _s, vref_mv, state = struct.unpack("<HBBIIHB", payload[:15])
                    last_state = state
                    if state == 1:
                        # Sessione rimasta aperta da prima: la chiudiamo e aspettiamo l'idle
                        ser.write(build_frame(CMD_STOP))
                        ser.flush()
                        continue
                    if state != 0:
                        continue
                    self.vref_v = vref_mv / 1000.0
                    ser.reset_input_buffer()
                    fw_str = f"{fw >> 8}.{fw & 0xFF}" + ("" if a_ok else " · ADC non risponde")
                    return ser, {
                        "fw": fw_str,
                        "adc_ok": bool(a_ok),
                        "parser": FrameParser(),
                        "mode": "binary",
                    }, None

            ser.close()
            if last_state is not None:
                return None, None, (f"il firmware risponde ma resta {INFO_STATE.get(last_state, '?')}: "
                                    "controlla alimentazione e collegamenti dell'ADS131E04")
            if rx_total == 0 and len(boot) == 0:
                diag = ("nessun byte ricevuto: firmware SpaceFlex non in esecuzione "
                        "(premi EN/RST sulla scheda, ricarica il firmware)")
            elif rx_total == 0:
                diag = (f"solo log di avvio ({len(boot)} byte), poi silenzio: "
                        "il firmware non gestisce il PING binario 0xA5 0x5A")
            else:
                txt = "".join(chr(c) if 32 <= c < 127 else "." for c in sample[:60])
                diag = (f"ricevuti {rx_total} byte ma nessun frame valido "
                        f"(baud diverso o firmware che stampa testo): \"{txt}\"")
            return None, None, diag
        except Exception:
            try:
                ser.close()
            except Exception:
                pass
            raise

    def connect_adc(self, port=None, baud=921600):
        with self._lock:
            self._disconnect_adc_internal()

            if port:
                candidates = [port]
            else:
                candidates = [dev for dev, _p in self._candidate_ports()]
            if not candidates:
                return {
                    "ok": False,
                    "error": (
                        "Nessuna porta USB trovata.\n"
                        "• Controlla il cavo (deve essere un cavo dati, non solo ricarica).\n"
                        "• Verifica con: ls /dev/ttyUSB* /dev/ttyACM*\n"
                        "• Su Ubuntu, se la ESP32 usa un CH340, disinstalla brltty: sudo apt remove brltty"
                    ),
                }

            errors = []
            # 115200 è il default dei firmware Arduino che stampano testo
            bauds = [int(baud)] + ([115200] if int(baud) != 115200 else [])
            for dev in candidates:
                try:
                    diags = []
                    for b in bauds:
                        ser, info, diag = self._probe_port(dev, b)
                        if ser is not None:
                            self._ser = ser
                            self.port_name = dev
                            self.baud = b
                            self._parser = info["parser"]
                            self.mode = info.get("mode", "binary")
                            return {
                                "ok": True,
                                "port": dev,
                                "mode": self.mode,
                                "baud": b,
                                "fw": info["fw"],
                                "adc_ok": info["adc_ok"],
                                "vref_mv": int(self.vref_v * 1000),
                            }
                        diags.append(f"@{b}: {diag}")
                    errors.append(f"{dev}: la porta si apre ma la ESP32 non risponde al PING.\n   " + "\n   ".join(diags))
                except serial.SerialException as e:
                    msg = str(e)
                    if "Permission denied" in msg or "Errno 13" in msg:
                        msg = ("permesso negato. Aggiungi il tuo utente al gruppo dialout: "
                               "sudo usermod -aG dialout $USER  poi esci e rientra nella sessione")
                    elif "Device or resource busy" in msg or "Errno 16" in msg:
                        msg = "porta occupata da un altro programma (Arduino IDE / Serial Monitor / screen)"
                    elif "Input/output error" in msg or "Errno 5" in msg:
                        msg = "errore I/O: scollega e ricollega la ESP32 (o prova un'altra porta USB)"
                    errors.append(f"{dev}: {msg}")
                except Exception as e:
                    errors.append(f"{dev}: {e}")

            return {"ok": False, "error": "Impossibile connettersi alla ESP32.\n" + "\n".join(errors)}

    def _disconnect_adc_internal(self):
        self._stop_reader = True
        self.acquiring = False
        if self._ser and self._ser.is_open:
            try:
                if self.mode != "text":
                    self._ser.write(build_frame(CMD_STOP))
                    self._ser.flush()
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.port_name = None
        self.mode = "binary"

    def disconnect_adc(self):
        with self._lock:
            self._disconnect_adc_internal()
        return {"ok": True}

    def start_adc_acquisition(self, duration_sec=10, channel=1, gain=12, rate=1000, mux_mode="normal"):
        if not self._ser or not self._ser.is_open:
            return {"ok": False, "error": "ADC non connesso"}

        self.stop_adc_acquisition()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        self._kalman.reset()

        if self.mode == "text":
            # Il firmware testuale trasmette sempre: basta cominciare a leggere
            self._ser.reset_input_buffer()
            self.acquiring = True
            self._stop_reader = False
            self._reader_thread = threading.Thread(target=self._text_stream_loop, daemon=True)
            self._reader_thread.start()
            return {"ok": True}

        self.sample_rate = int(rate) if int(rate) in RATE_TO_DR else 1000
        self.channel_idx = max(0, min(3, int(channel) - 1))
        self.gain = int(gain)
        ch_mask = 1 << self.channel_idx

        gains = [1, 1, 1, 1]
        gains[self.channel_idx] = self.gain

        mux_code = 5 if mux_mode == "test" else 0
        muxes = [mux_code, mux_code, mux_code, mux_code]

        duration_ms = int(float(duration_sec) * 1000)
        payload = build_start_payload(
            RATE_TO_DR[self.sample_rate],
            ch_mask,
            gains,
            muxes,
            duration_ms,
            START_FLAG_OFFSETCAL,
            0,
        )

        self._parser = FrameParser()
        self._ser.reset_input_buffer()
        self._ser.write(build_frame(CMD_START, payload))
        self._ser.flush()

        # Con OFFSETCAL il firmware risponde dopo circa 400 ms (riferimento + calibrazione)
        t_end = time.monotonic() + 4.0
        ack_ok = False
        bpc = 3
        while time.monotonic() < t_end:
            raw = self._ser.read(max(1, self._ser.in_waiting))
            for ftype, p in self._parser.feed(raw):
                if ftype == RSP_ACK and len(p) >= 2 and p[0] == CMD_START:
                    if p[1] != 0:
                        motivo = ACK_STATUS.get(p[1], f"codice {p[1]}")
                        return {"ok": False, "error": f"La ESP32 ha rifiutato l'acquisizione: {motivo}."}
                    if len(p) >= 17:
                        _r, _n, _m, bpc, _f, _s, _c, vref_mv = struct.unpack("<IIBBBBBH", p[2:17])
                        self.vref_v = vref_mv / 1000.0
                    ack_ok = True
                    break
            if ack_ok:
                break

        if not ack_ok:
            return {"ok": False, "error": "Timeout attesa ACK da ESP32"}

        self.acquiring = True
        self._stop_reader = False
        self._reader_thread = threading.Thread(
            target=self._stream_loop, args=(bpc, self.gain, self.vref_v), daemon=True
        )
        self._reader_thread.start()
        return {"ok": True}

    def stop_adc_acquisition(self):
        self._stop_reader = True
        self.acquiring = False
        if self.mode == "text":
            return {"ok": True}
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(build_frame(CMD_STOP))
                self._ser.flush()
            except Exception:
                pass
        return {"ok": True}

    def _text_stream_loop(self):
        """Una riga = un campione in mV, inoltrato così com'è alla pagina."""
        pending = b""
        while not self._stop_reader and self._ser and self._ser.is_open:
            try:
                raw = self._ser.read(max(1, self._ser.in_waiting))
                if not raw:
                    continue
                values, pending = _parse_mv_lines(pending + raw)
                if self._window:
                    for v in values:
                        mv = self._kalman.update(abs(v))
                        self._window.evaluate_js(
                            f"if(window.riceviCampioneSeriale) riceviCampioneSeriale({mv:.4f});"
                        )
            except Exception:
                break

    def _stream_loop(self, bpc, gain, vref_v):
        full_scale = (1 << (8 * bpc - 1)) - 1
        lsb_mv = (vref_v / gain / full_scale) * 1000.0

        bucket = []
        decimation = max(1, self.sample_rate // 25)

        while not self._stop_reader and self._ser and self._ser.is_open:
            try:
                raw = self._ser.read(max(1, self._ser.in_waiting))
                if not raw:
                    continue
                frames = self._parser.feed(raw)
                for ftype, p in frames:
                    if ftype == RSP_DATA and len(p) >= 10:
                        _seq, _first, n, mask, rx_bpc, flags = struct.unpack_from("<HIBBBB", p, 0)
                        nch = bin(mask & 0x0F).count("1")
                        has_status = bool(flags & DATA_FLAG_STATUS)
                        off = 10
                        for _ in range(n):
                            if has_status:
                                off += 3
                            if off + rx_bpc <= len(p):
                                code = int.from_bytes(p[off : off + rx_bpc], "big", signed=True)
                                bucket.append(code * lsb_mv)
                            off += nch * rx_bpc

                        # Inviluppo a 25 Hz: RMS di ogni blocco dopo aver tolto la media,
                        # così l'offset residuo dell'elettrodo non entra nella misura.
                        if len(bucket) >= decimation and self._window:
                            mean = sum(bucket) / len(bucket)
                            rms_mv = (sum((x - mean) ** 2 for x in bucket) / len(bucket)) ** 0.5
                            rms_mv = self._kalman.update(rms_mv)
                            bucket.clear()
                            self._window.evaluate_js(f"if(window.riceviCampioneSeriale) riceviCampioneSeriale({rms_mv:.4f});")

                    elif ftype == RSP_END:
                        self.acquiring = False
                        self._stop_reader = True
                        if self._window:
                            self._window.evaluate_js("if(window.finishAcquisition && isAcquiring) finishAcquisition();")
                        return
            except Exception:
                break


# WebKitGTK disegna select e input number con un tema chiaro suo, che ignora
# GTK_THEME. Li sistemiamo con questo CSS, iniettato solo dentro l'app.
NATIVE_CONTROLS_CSS = """
:root { color-scheme: dark; }
select {
  -webkit-appearance: none !important;
  appearance: none !important;
  background-color: #042f2e !important;
  color: #ecfdf5 !important;
  background-image: url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='10' height='6' viewBox='0 0 10 6'><path d='M1 1l4 4 4-4' fill='none' stroke='%2334d399' stroke-width='1.6' stroke-linecap='round' stroke-linejoin='round'/></svg>") !important;
  background-repeat: no-repeat !important;
  background-position: right 8px center !important;
  background-size: 10px 6px !important;
  padding-right: 26px !important;
  cursor: pointer;
}
select option { background-color: #031b20 !important; color: #ecfdf5 !important; }
input[type="number"] {
  -webkit-appearance: textfield !important;
  appearance: textfield !important;
  color: #ffffff !important;
}
input[type="number"]::-webkit-inner-spin-button,
input[type="number"]::-webkit-outer-spin-button {
  -webkit-appearance: none !important;
  margin: 0 !important;
  display: none !important;
}
"""

INJECT_CSS_JS = (
    "(function(){"
    "if(document.getElementById('sfx-native-controls'))return;"
    "var st=document.createElement('style');st.id='sfx-native-controls';"
    "st.textContent=" + json.dumps(NATIVE_CONTROLS_CSS) + ";"
    "(document.head||document.documentElement).appendChild(st);"
    # F11 per uscire ed entrare dallo schermo intero, senza toccare le pagine
    "window.addEventListener('keydown',function(e){"
    "if(e.key==='F11'&&window.pywebview&&window.pywebview.api){"
    "e.preventDefault();window.pywebview.api.toggle_fullscreen();}});"
    "})();"
)


def _inject_native_controls_css(window):
    try:
        window.evaluate_js(INJECT_CSS_JS)
    except Exception:
        pass


def _prefer_dark_gtk():
    """Chiede a GTK la variante scura del tema, se PyGObject è installato."""
    if not sys.platform.startswith("linux"):
        return
    try:
        import gi
        gi.require_version("Gtk", "3.0")
        from gi.repository import GLib, Gtk

        def _apply():
            settings = Gtk.Settings.get_default()
            if settings is not None:
                settings.set_property("gtk-application-prefer-dark-theme", True)
            return False

        GLib.idle_add(_apply)
    except Exception:
        pass


def main():
    bridge = SpaceFlexBridge()
    login_uri = get_resource_uri("login.html")

    window = webview.create_window(
        title="SpaceFlex — Neuromuscular sEMG Monitor",
        url=login_uri,
        width=1360,
        height=820,
        min_size=(1024, 680),
        fullscreen=True,
        background_color="#17191F",
        js_api=bridge,
    )
    bridge.set_window(window)
    # window.events.loaded esiste da pywebview 4, prima era window.loaded
    _on_loaded = lambda *a: _inject_native_controls_css(window)
    events = getattr(window, "events", None)
    if events is not None and hasattr(events, "loaded"):
        events.loaded += _on_loaded
    elif hasattr(window, "loaded"):
        window.loaded += _on_loaded
    webview.start(func=_prefer_dark_gtk, debug=False)


if __name__ == "__main__":
    main()