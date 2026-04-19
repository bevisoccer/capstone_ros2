#!/usr/bin/env python3
"""
hand_controller.py — Python driver for the robotic hand controller
Teensy 4.1 running hand_control_v4.ino

Architecture
------------
SerialReader   background thread — reads lines, parses key=val, fills queues
SerialWriter   background thread — drains a command queue, rate-limited
HandState      thread-safe state store updated by SerialReader
HandController public API — finger-level commands, sensor reads, ROS-ready hooks

ROS INTEGRATION NOTES
---------------------
To turn this into a ROS2 node:
1. Create a class ROSHandNode(Node) that instantiates HandController
2. Replace the on_* callbacks below with ROS publisher calls
3. Replace send_* methods with ROS subscriber callbacks
4. The background threads already run independently — just spin() alongside them

Usage
-----
    ctrl = HandController(port="/dev/cu.usbmodem193107001", baud=115200)
    ctrl.connect()
    ctrl.home_all()
    ctrl.move_finger_curve1("thumb", 45.0)
    data = ctrl.get_state()
    ctrl.disconnect()
"""

import serial
import serial.tools.list_ports
import threading
import queue
import time
import logging
from dataclasses import dataclass, field
from typing import Optional, Callable, Dict

TEENSY_VID = 0x16C0  # Van Ooijen Technische Informatica (Teensyduino)


def find_teensy_port() -> Optional[str]:
    """Return the first serial port that belongs to a Teensy, or None."""
    for port in serial.tools.list_ports.comports():
        if port.vid == TEENSY_VID:
            return port.device
    return None

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("hand")


# =============================================================================
# ── CONSTANTS ─────────────────────────────────────────────────────────────────
# =============================================================================

# Finger name → (curve1_ch, curve2_ch, rotate_ch)
# curve2 and rotate are None for thumb (thumb only has curve1 + rotate)
FINGER_CHANNELS: Dict[str, Dict[str, Optional[int]]] = {
    "thumb":  {"curve1": 11, "curve2": None, "rotate": 10},
    "index":  {"curve1": 12, "curve2":    8, "rotate":  9},
    "middle": {"curve1": 13, "curve2":    6, "rotate":  7},
    "ring":   {"curve1": 14, "curve2":    4, "rotate":  5},
    "pinky":  {"curve1": 15, "curve2":    2, "rotate":  3},
}

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]

# FSR index → finger
FSR_FINGER = {0: "thumb", 1: "index", 2: "middle", 3: "ring", 4: "pinky"}

# INA219 index → finger
INA_FINGER = {0: "ring", 1: "middle", 2: "index", 3: "pinky", 4: "thumb"}

SERIAL_TIMEOUT   = 2.0    # seconds
WRITE_RATE_HZ    = 50     # max commands per second to Teensy
STREAM_HZ_INA    = 20     # INA219 stream rate
STREAM_HZ_FSR    = 20     # FSR stream rate


# =============================================================================
# ── DATA CLASSES ──────────────────────────────────────────────────────────────
# =============================================================================

@dataclass
class ServoStatus:
    ch: int
    label: str
    angle: float = 0.0
    target: float = 0.0
    init: float = 0.0
    min_angle: float = 0.0
    max_angle: float = 180.0
    enabled: bool = False
    stopped: bool = False
    ina_ch: int = -1

@dataclass
class InaStatus:
    ch: int
    label: str
    finger: str
    addr: str = ""
    bus: str = ""
    servo_ch: int = -1
    found: bool = False
    bus_v: float = 0.0
    shunt_mv: float = 0.0
    current_ma: float = 0.0
    power_mw: float = 0.0
    last_update: float = 0.0

@dataclass
class FsrStatus:
    ch: int
    label: str
    finger: str
    pin: int = 0
    raw: int = 0
    scaled: int = 0
    pct: float = 0.0
    pressed: bool = False
    pending: bool = False
    last_update: float = 0.0

@dataclass
class SystemStatus:
    estop: bool = False
    speed_dps: float = 60.0
    ina_stream: bool = False
    ina_hz: int = 0
    fsr_stream: bool = False
    fsr_hz: int = 0
    ready: bool = False
    connected: bool = False


# =============================================================================
# ── HAND STATE (thread-safe) ──────────────────────────────────────────────────
# =============================================================================

class HandState:
    """Thread-safe store for all sensor and servo state."""

    def __init__(self):
        self._lock = threading.RLock()
        self.servos: Dict[int, ServoStatus] = {
            ch: ServoStatus(ch=ch, label="") for ch in range(16)
        }
        self.ina: Dict[int, InaStatus] = {
            i: InaStatus(ch=i, label=INA_FINGER.get(i, "?"), finger=INA_FINGER.get(i, "?"))
            for i in range(5)
        }
        self.fsr: Dict[int, FsrStatus] = {
            i: FsrStatus(ch=i, label=f"F{i}", finger=FSR_FINGER.get(i, "?"))
            for i in range(5)
        }
        self.system = SystemStatus()

    def update_servo(self, fields: dict):
        ch = int(fields.get("ch", -1))
        if ch < 0: return
        with self._lock:
            s = self.servos.setdefault(ch, ServoStatus(ch=ch, label=""))
            if "label"   in fields: s.label      = fields["label"]
            if "angle"   in fields: s.angle       = float(fields["angle"])
            if "target"  in fields: s.target      = float(fields["target"])
            if "init"    in fields: s.init        = float(fields["init"])
            if "min"     in fields: s.min_angle   = float(fields["min"])
            if "max"     in fields: s.max_angle   = float(fields["max"])
            if "enabled" in fields: s.enabled     = fields["enabled"] == "1"
            if "stopped" in fields: s.stopped     = fields["stopped"] == "1"
            if "ina_ch"  in fields: s.ina_ch      = int(fields["ina_ch"])

    def update_ina(self, fields: dict):
        ch = int(fields.get("ch", -1))
        if ch < 0: return
        with self._lock:
            s = self.ina.setdefault(ch, InaStatus(ch=ch, label="", finger=""))
            if "label"      in fields: s.label      = fields["label"]
            if "addr"       in fields: s.addr       = fields["addr"]
            if "bus"        in fields: s.bus        = fields["bus"]
            if "servo_ch"   in fields: s.servo_ch   = int(fields["servo_ch"])
            if "found"      in fields: s.found      = fields["found"] == "1"
            if "bus_v"      in fields: s.bus_v      = float(fields["bus_v"])
            if "shunt_mv"   in fields: s.shunt_mv   = float(fields["shunt_mv"])
            if "current_ma" in fields: s.current_ma = float(fields["current_ma"])
            if "power_mw"   in fields: s.power_mw   = float(fields["power_mw"])
            s.last_update = time.time()

    def update_fsr(self, fields: dict):
        ch = int(fields.get("ch", -1))
        if ch < 0: return
        with self._lock:
            s = self.fsr.setdefault(ch, FsrStatus(ch=ch, label="", finger=""))
            if "label"   in fields: s.label   = fields["label"]
            if "finger"  in fields: s.finger  = fields["finger"]
            if "pin"     in fields: s.pin     = int(fields["pin"])
            if "raw"     in fields: s.raw     = int(fields["raw"])
            if "scaled"  in fields: s.scaled  = int(fields["scaled"])
            if "pct"     in fields: s.pct     = float(fields["pct"])
            if "pressed" in fields: s.pressed = fields["pressed"] == "1"
            if "pending" in fields: s.pending = fields["pending"] == "1"
            s.last_update = time.time()

    def update_system(self, fields: dict):
        with self._lock:
            sys = self.system
            if "estop"      in fields: sys.estop      = fields["estop"] == "1"
            if "speed_dps"  in fields: sys.speed_dps  = float(fields["speed_dps"])
            if "ina_stream" in fields: sys.ina_stream = fields["ina_stream"] == "1"
            if "ina_hz"     in fields: sys.ina_hz     = int(fields["ina_hz"])
            if "fsr_stream" in fields: sys.fsr_stream = fields["fsr_stream"] == "1"
            if "fsr_hz"     in fields: sys.fsr_hz     = int(fields["fsr_hz"])

    def snapshot(self) -> dict:
        """Return a deep copy of all state — safe to read from any thread."""
        with self._lock:
            import copy
            return {
                "servos": copy.deepcopy(self.servos),
                "ina":    copy.deepcopy(self.ina),
                "fsr":    copy.deepcopy(self.fsr),
                "system": copy.deepcopy(self.system),
            }


# =============================================================================
# ── LINE PARSER ───────────────────────────────────────────────────────────────
# =============================================================================

def parse_line(line: str) -> tuple[str, dict]:
    """
    Parse 'TYPE key=val key=val ...' into (type_str, fields_dict).
    Returns ("INFO", {"text": ...}) for INFO lines.
    Returns ("", {}) on empty / unparseable lines.
    """
    line = line.strip()
    if not line:
        return "", {}
    tokens = line.split()
    msg_type = tokens[0]
    if msg_type == "INFO":
        return "INFO", {"text": " ".join(tokens[1:])}
    fields = {}
    for tok in tokens[1:]:
        if "=" in tok:
            k, _, v = tok.partition("=")
            fields[k] = v
        else:
            fields[tok] = True   # bare token (shouldn't happen in v4)
    return msg_type, fields


# =============================================================================
# ── SERIAL READER THREAD ──────────────────────────────────────────────────────
# =============================================================================

class SerialReader(threading.Thread):
    """Reads lines from serial, updates HandState, fires callbacks."""

    def __init__(self, ser: serial.Serial, state: HandState,
                 event_queue: queue.Queue):
        super().__init__(daemon=True, name="SerialReader")
        self._ser   = ser
        self._state = state
        self._eq    = event_queue
        self._stop  = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                msg_type, fields = parse_line(line)
                self._dispatch(msg_type, fields, line)

            except serial.SerialException as e:
                log.error(f"SerialReader: port error: {e}")
                self._stop.set()
            except Exception as e:
                log.warning(f"SerialReader: unexpected error: {e}")

    def _dispatch(self, msg_type: str, fields: dict, raw_line: str):
        s = self._state
        eq = self._eq

        if msg_type == "INFO":
            log.info(f"[INFO] {fields.get('text', raw_line)}")
            return

        elif msg_type == "READY":
            s.system.ready     = True
            s.system.connected = True
            log.info(f"Teensy READY: {raw_line}")
            eq.put(("ready", fields))

        elif msg_type in ("MOVE_START", "MOVE_DONE"):
            s.update_servo(fields)
            eq.put((msg_type.lower(), fields))

        elif msg_type == "HOME_START":
            log.info(f"HOME started: {fields.get('channels','?')} channels")
            eq.put(("home_start", fields))

        elif msg_type == "HOME_DONE":
            log.info("HOME complete")
            eq.put(("home_done", {}))

        elif msg_type == "STOP":
            s.update_servo(fields)
            eq.put(("stop", fields))

        elif msg_type == "ESTOP":
            s.system.estop = True
            log.warning(f"ESTOP received: {raw_line}")
            eq.put(("estop", fields))

        elif msg_type == "RESUME":
            s.update_servo(fields)
            eq.put(("resume", fields))

        elif msg_type == "RESUME_ALL":
            s.system.estop = False
            eq.put(("resume_all", {}))

        elif msg_type == "SPEED":
            if "dps" in fields:
                s.system.speed_dps = float(fields["dps"])
            eq.put(("speed", fields))

        elif msg_type == "INA":
            s.update_ina(fields)
            eq.put(("ina", fields))

        elif msg_type == "INA_MISSING":
            ch = int(fields.get("ch", -1))
            if ch >= 0:
                s.ina[ch].found = False
            eq.put(("ina_missing", fields))

        elif msg_type == "INA_STREAM":
            s.system.ina_stream = True
            s.system.ina_hz     = int(fields.get("hz", 0))

        elif msg_type == "INA_STREAM_STOP":
            s.system.ina_stream = False

        elif msg_type == "FSR":
            s.update_fsr(fields)
            eq.put(("fsr", fields))

        elif msg_type == "FSR_STREAM":
            s.system.fsr_stream = True
            s.system.fsr_hz     = int(fields.get("hz", 0))

        elif msg_type == "FSR_STREAM_STOP":
            s.system.fsr_stream = False

        elif msg_type == "STATUS_SERVO":
            s.update_servo(fields)

        elif msg_type == "STATUS_INA":
            s.update_ina(fields)

        elif msg_type == "STATUS_FSR":
            s.update_fsr(fields)

        elif msg_type == "STATUS_SYSTEM":
            s.update_system(fields)

        elif msg_type == "SCAN":
            log.info(f"SCAN: {raw_line}")
            eq.put(("scan", fields))

        elif msg_type in ("SEQ_START", "SEQ_DONE"):
            eq.put((msg_type.lower(), fields))

        elif msg_type == "SYNC_START":
            eq.put(("sync_start", fields))

        elif msg_type == "MOVESET_START":
            eq.put(("moveset_start", fields))

        elif msg_type == "MOVESET_DONE":
            eq.put(("moveset_done", fields))

        elif msg_type == "ERR":
            log.error(f"Teensy ERR: {fields.get('msg', raw_line)}")
            eq.put(("error", fields))

        else:
            log.debug(f"unhandled: {raw_line}")


# =============================================================================
# ── SERIAL WRITER THREAD ──────────────────────────────────────────────────────
# =============================================================================

class SerialWriter(threading.Thread):
    """Drains a command queue and writes to serial at a controlled rate."""

    def __init__(self, ser: serial.Serial, rate_hz: float = WRITE_RATE_HZ):
        super().__init__(daemon=True, name="SerialWriter")
        self._ser      = ser
        self._queue    = queue.Queue()
        self._interval = 1.0 / rate_hz
        self._stop     = threading.Event()

    def stop(self):
        self._stop.set()

    def send(self, cmd: str):
        """Queue a command string (no newline needed)."""
        self._queue.put(cmd)

    def send_urgent(self, cmd: str):
        """Put urgent command at front (use for ESTOP only)."""
        # Queue doesn't support priority natively — drain and re-add
        # For ESTOP, we just write directly bypassing the queue
        try:
            line = (cmd.strip() + "\n").encode("utf-8")
            self._ser.write(line)
            self._ser.flush()
        except serial.SerialException as e:
            log.error(f"urgent write failed: {e}")

    def run(self):
        while not self._stop.is_set():
            try:
                cmd = self._queue.get(timeout=0.1)
                line = (cmd.strip() + "\n").encode("utf-8")
                self._ser.write(line)
                self._ser.flush()
                time.sleep(self._interval)
            except queue.Empty:
                pass
            except serial.SerialException as e:
                log.error(f"SerialWriter: port error: {e}")
                self._stop.set()


# =============================================================================
# ── HAND CONTROLLER (public API) ──────────────────────────────────────────────
# =============================================================================

class HandController:
    """
    High-level finger-oriented API over the Teensy serial protocol.

    Callbacks (set before connecting or any time):
        on_ina_update(ina_dict)      called every INA219 reading
        on_fsr_update(fsr_dict)      called every FSR reading
        on_move_done(ch, label, ang) called when a servo reaches its target
        on_home_done()               called when HOME_ALL completes
        on_estop()                   called on ESTOP
        on_error(msg)                called on ERR

    ROS node pattern:
        class ROSHandNode(Node):
            def __init__(self):
                super().__init__("hand_node")
                self.ctrl = HandController("/dev/cu.usbmodem193107001")
                self.ctrl.on_ina_update = self._pub_ina
                self.ctrl.on_fsr_update = self._pub_fsr
                self.ctrl.connect()
    """

    def __init__(self, port: str = "auto", baud: int = 115200):
        self._port   = port
        self._baud   = baud
        self._ser:   Optional[serial.Serial]  = None
        self._reader: Optional[SerialReader]  = None
        self._writer: Optional[SerialWriter]  = None
        self._events = queue.Queue()
        self._state  = HandState()
        self._event_thread: Optional[threading.Thread] = None
        self._running = False

        # ── Callbacks (override these for ROS or any consumer) ──
        self.on_ina_update:  Callable = lambda ina_fields: None
        self.on_fsr_update:  Callable = lambda fsr_fields: None
        self.on_move_done:   Callable = lambda ch, label, angle: None
        self.on_home_done:   Callable = lambda: None
        self.on_estop:       Callable = lambda: None
        self.on_error:       Callable = lambda msg: None

    # ── Connection ─────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Open serial port and start background threads.

        Does NOT wait for READY — just opens the port and starts reading/writing.
        Streams are started after a short settle delay.

        macOS note: use /dev/cu.usbmodem... NOT /dev/tty.usbmodem...
        The tty. variant blocks on open until carrier detect; cu. opens immediately.
        """
        if self._port == "auto":
            detected = find_teensy_port()
            if detected is None:
                log.error("Could not find a Teensy on any serial port. Is it plugged in?")
                return False
            self._port = detected
            log.info(f"Auto-detected Teensy on {self._port}")
        log.info(f"Connecting to {self._port} at {self._baud} baud")

        try:
            self._ser = serial.Serial(
                self._port, self._baud,
                timeout=SERIAL_TIMEOUT,
                write_timeout=2.0,
            )
        except serial.SerialException as e:
            log.error(f"Could not open port {self._port}: {e}")
            return False

        # Flush anything buffered before we started listening
        time.sleep(0.5)
        self._ser.reset_input_buffer()

        self._writer = SerialWriter(self._ser)
        self._reader = SerialReader(self._ser, self._state, self._events)
        self._running = True

        self._event_thread = threading.Thread(
            target=self._event_loop, daemon=True, name="EventLoop")

        self._writer.start()
        self._reader.start()
        self._event_thread.start()

        self._state.system.connected = True
        log.info("Port open — threads running")

        # Short delay then start streams so Teensy has settled
        time.sleep(1.0)
        self._start_streams()
        return True

    def disconnect(self):
        """Stop all streams, stop threads, close port."""
        log.info("Disconnecting")
        self._running = False
        # Drain any queued MOVE commands so they don't flood the Teensy on shutdown
        if self._writer:
            try:
                while not self._writer._queue.empty():
                    self._writer._queue.get_nowait()
            except Exception:
                pass
        self._send("INOSTOP")
        self._send("FSRSTOP")
        time.sleep(0.3)
        if self._writer: self._writer.stop()
        if self._reader: self._reader.stop()
        if self._ser and self._ser.is_open:
            self._ser.close()

    def _start_streams(self):
        """Start continuous INA219 and FSR streaming after connect."""
        self._send(f"STREAM {STREAM_HZ_INA}")
        self._send(f"FSRSTREAM {STREAM_HZ_FSR}")
        log.info(f"Streams started: INA@{STREAM_HZ_INA}Hz FSR@{STREAM_HZ_FSR}Hz")

    # ── Event loop (fires callbacks) ───────────────────────────────────────────

    def _event_loop(self):
        while self._running:
            try:
                event_type, fields = self._events.get(timeout=0.2)
                self._handle_event(event_type, fields)
            except queue.Empty:
                pass
            except Exception as e:
                log.warning(f"event_loop error: {e}")

    def _handle_event(self, event_type: str, fields: dict):
        if event_type == "ina":
            self.on_ina_update(fields)

        elif event_type == "fsr":
            self.on_fsr_update(fields)

        elif event_type == "move_done":
            ch    = int(fields.get("ch", -1))
            label = fields.get("label", "")
            angle = float(fields.get("angle", 0))
            self.on_move_done(ch, label, angle)

        elif event_type == "home_done":
            self.on_home_done()

        elif event_type == "estop":
            self.on_estop()

        elif event_type == "error":
            self.on_error(fields.get("msg", str(fields)))

    # ── Low-level send ─────────────────────────────────────────────────────────

    def _send(self, cmd: str):
        if self._writer:
            self._writer.send(cmd)

    # ── Servo commands — by channel ────────────────────────────────────────────

    def move(self, ch: int, angle: float):
        """Move PCA9685 channel ch to angle (degrees)."""
        self._send(f"MOVE {ch} {angle:.1f}")

    def stop_ch(self, ch: int):
        """Disable one channel (holds last position)."""
        self._send(f"STOP {ch}")

    def resume_ch(self, ch: int):
        """Re-enable a stopped channel."""
        self._send(f"RESUME {ch}")

    def estop(self):
        """Emergency stop all servos — sent urgently, bypasses queue."""
        if self._writer:
            self._writer.send_urgent("ESTOP")

    def resume_all(self):
        self._send("RESUME_ALL")

    def home_all(self):
        """Move all servos to their INIT_ANGLE positions."""
        self._send("HOME")

    def home_ch(self, ch: int):
        """Move one channel to its INIT_ANGLE position."""
        self._send(f"HOME {ch}")

    def set_speed(self, dps: float):
        """Set global slew rate in degrees/second (5–720)."""
        self._send(f"SPEED {dps:.1f}")

    # ── Servo commands — by finger ─────────────────────────────────────────────

    def _finger_ch(self, finger: str, joint: str) -> int:
        """Look up channel for a finger+joint. Raises ValueError if invalid."""
        finger = finger.lower()
        joint  = joint.lower()
        if finger not in FINGER_CHANNELS:
            raise ValueError(f"Unknown finger: {finger}. Use: {list(FINGER_CHANNELS)}")
        joints = FINGER_CHANNELS[finger]
        if joint not in joints or joints[joint] is None:
            raise ValueError(f"Finger '{finger}' has no joint '{joint}'")
        return joints[joint]

    def move_finger_curve1(self, finger: str, angle: float):
        """Move the 1st curvature joint (monitored) of a finger."""
        self.move(self._finger_ch(finger, "curve1"), angle)

    def move_finger_curve2(self, finger: str, angle: float):
        """Move the 2nd curvature joint of a finger (not thumb)."""
        self.move(self._finger_ch(finger, "curve2"), angle)

    def move_finger_rotate(self, finger: str, angle: float):
        """Move the rotational joint of a finger."""
        self.move(self._finger_ch(finger, "rotate"), angle)

    def move_finger(self, finger: str, curve1: Optional[float] = None,
                    curve2: Optional[float] = None, rotate: Optional[float] = None):
        """
        Move multiple joints of one finger in one call.
        Pass None to leave a joint unchanged.
        Example:
            ctrl.move_finger("index", curve1=90, curve2=45)
        """
        if curve1 is not None:
            self.move_finger_curve1(finger, curve1)
        if curve2 is not None:
            self.move_finger_curve2(finger, curve2)
        if rotate is not None:
            self.move_finger_rotate(finger, rotate)

    def home_finger(self, finger: str):
        """Home all joints of one finger."""
        for joint, ch in FINGER_CHANNELS[finger.lower()].items():
            if ch is not None:
                self.home_ch(ch)

    def grip_all(self, curve1_angle: float = 150.0, curve2_angle: float = 150.0):
        """
        Close all fingers to a grip position.
        Moves curve1 and curve2 joints (not rotations).
        """
        for finger, joints in FINGER_CHANNELS.items():
            if joints["curve1"] is not None:
                self.move(joints["curve1"], curve1_angle)
            if joints["curve2"] is not None:
                self.move(joints["curve2"], curve2_angle)

    def open_all(self):
        """Open hand — home all joints."""
        self.home_all()

    # ── v6 bulk/sync commands ──────────────────────────────────────────────────

    def curvature(self, angle: float):
        """Move all curvature joints sequentially (thumb→index→middle→ring→pinky)."""
        self._send(f"CURVATURE {angle:.1f}")

    def rotate(self, angle: float):
        """Move all rotation joints sequentially."""
        self._send(f"ROTATE {angle:.1f}")

    def curvature_sync(self, angle: float):
        """Move all curvature joints simultaneously."""
        self._send(f"CURVATURE_SYNC {angle:.1f}")

    def rotate_sync(self, angle: float):
        """Move all rotation joints simultaneously."""
        self._send(f"ROTATE_SYNC {angle:.1f}")

    def moveall(self, angle: float):
        """Move ALL joints (curvature + rotation) simultaneously."""
        self._send(f"MOVEALL {angle:.1f}")

    def moveset(self, ch_angles: dict):
        """Move up to 4 channels to different angles simultaneously. {ch: angle}"""
        pairs = " ".join(f"{ch}:{angle:.1f}" for ch, angle in ch_angles.items())
        self._send(f"MOVESET {pairs}")

    # ── Sensor reads (one-shot) ────────────────────────────────────────────────

    def read_ina(self):
        """Request one-shot INA219 read. Results arrive via on_ina_update."""
        self._send("READ")

    def read_fsr(self):
        """Request one-shot FSR read. Results arrive via on_fsr_update."""
        self._send("FSRREAD")

    def scan_i2c(self):
        """Scan I2C buses. Result logged and sent to event queue."""
        self._send("SCAN")

    def request_status(self):
        """Request full STATUS snapshot. State updated automatically."""
        self._send("STATUS")

    # ── State accessors ────────────────────────────────────────────────────────

    def get_state(self) -> dict:
        """Return a thread-safe snapshot of all state."""
        return self._state.snapshot()

    def get_finger_state(self, finger: str) -> dict:
        """
        Return current angle, current, and FSR state for one finger.
        Example return:
            {
              "finger": "index",
              "curve1": {"angle": 45.0, "current_ma": 32.1},
              "curve2": {"angle": 30.0, "current_ma": None},
              "rotate": {"angle": 90.0, "current_ma": None},
              "fsr":    {"scaled": 350, "pct": 35.0, "pressed": False},
            }
        """
        finger = finger.lower()
        result = {"finger": finger}
        snap = self._state.snapshot()

        # Servo joints
        for joint, ch in FINGER_CHANNELS[finger].items():
            if ch is None:
                result[joint] = None
                continue
            sv = snap["servos"].get(ch)
            ina_data = None
            if sv and sv.ina_ch >= 0:
                ina = snap["ina"].get(sv.ina_ch)
                if ina and ina.found:
                    ina_data = ina.current_ma
            result[joint] = {
                "ch":         ch,
                "angle":      sv.angle if sv else None,
                "target":     sv.target if sv else None,
                "current_ma": ina_data,
            }

        # FSR
        fsr_ch = next((k for k, v in FSR_FINGER.items() if v == finger), None)
        if fsr_ch is not None:
            fsr = snap["fsr"].get(fsr_ch)
            result["fsr"] = {
                "scaled":  fsr.scaled  if fsr else None,
                "pct":     fsr.pct     if fsr else None,
                "pressed": fsr.pressed if fsr else None,
            }
        return result

    def is_estop(self) -> bool:
        return self._state.system.estop

    def is_ready(self) -> bool:
        return self._state.system.ready


# =============================================================================
# ── EXAMPLE USAGE / STANDALONE DEMO ──────────────────────────────────────────
# =============================================================================

def demo():
    import argparse
    parser = argparse.ArgumentParser(description="Hand controller demo")
    parser.add_argument("--port", default="/dev/cu.usbmodem193107001")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    # ── Instantiate ──
    ctrl = HandController(port=args.port, baud=args.baud)

    # ── Wire up callbacks (replace with ROS publishers in a ROS node) ──
    def on_ina(fields):
        finger = fields.get("label", "?")
        ma     = fields.get("current_ma", "?")
        log.info(f"INA  finger={finger:6s} current_ma={ma}")

    def on_fsr(fields):
        finger  = fields.get("finger", "?")
        pct     = fields.get("pct", "?")
        pressed = fields.get("pressed", "?")
        if fields.get("pending") == "1": return
        log.info(f"FSR  finger={finger:6s} pct={pct:5s}% pressed={pressed}")

    def on_move_done(ch, label, angle):
        log.info(f"MOVE_DONE ch={ch} label={label} angle={angle:.1f}")

    def on_home_done():
        log.info("HOME complete — all servos at init positions")

    def on_estop():
        log.warning("ESTOP received!")

    ctrl.on_ina_update = on_ina
    ctrl.on_fsr_update = on_fsr
    ctrl.on_move_done  = on_move_done
    ctrl.on_home_done  = on_home_done
    ctrl.on_estop      = on_estop

    # ── Connect ──
    if not ctrl.connect():
        log.error("Failed to connect")
        return

    log.info("Connected — streams running")
    log.info("Sending HOME to start at init positions")
    ctrl.home_all()
    time.sleep(4.0)

    # ── Demo: move each finger curve1 ──
    log.info("Demo: curl each finger")
    for finger in ["thumb", "index", "middle", "ring", "pinky"]:
        log.info(f"  curling {finger}")
        ctrl.move_finger_curve1(finger, 150.0)
        time.sleep(1.0)

    log.info("Demo: open hand")
    ctrl.open_all()
    time.sleep(3.0)

    # ── Print finger state snapshot ──
    log.info("State snapshot:")
    for finger in ["thumb", "index", "middle", "ring", "pinky"]:
        state = ctrl.get_finger_state(finger)
        log.info(f"  {state}")

    # ── Run sensor streams for a few seconds ──
    log.info("Listening to streams for 5 seconds...")
    time.sleep(5.0)

    ctrl.disconnect()
    log.info("Done")


if __name__ == "__main__":
    demo()