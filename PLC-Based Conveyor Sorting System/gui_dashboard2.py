"""
╔══════════════════════════════════════════════════════════════════════════════╗
║         PLC-BASED CONVEYOR SORTING SYSTEM  —  COMPLETE SINGLE FILE         ║
║                                                                              ║
║  Run:    python plc_conveyor_sorter.py                                       ║
║  Needs:  pip install PyQt5                                                   ║
╚══════════════════════════════════════════════════════════════════════════════╝

Architecture
────────────
  • SystemState FSM     : IDLE → RUNNING → SORTING → FAULT
  • SensorInputs        : Proximity (I:0.0/0), Height (I:0.0/1), E-Stop (I:0.0/2)
  • OutputCoils         : Conveyor Motor (O:0.0/0), Pneumatic Pusher (O:0.0/1),
                          Warning Light (O:0.0/2)
  • PLCTimer            : TON – 1.5 s pusher delay, 0.6 s pusher hold
  • PLCController       : 100 ms scan-cycle thread with safety interlocks
  • ConveyorWidget      : Animated QPainter belt, box, pusher
  • MainDashboard       : Full PyQt5 HMI with counters, sensors, log, controls

Safety Interlocks
─────────────────
  • E-Stop de-energises ALL outputs immediately (hardwired rung priority)
  • Fault state blocks START until explicit RESET after E-Stop release
  • Pusher cannot fire unless conveyor motor is ON (permissive check)

Author  : PLC Conveyor Sorter Project
License : MIT
"""

# ─────────────────────────────────────────────────────────────────────────────
#  STANDARD LIBRARY
# ─────────────────────────────────────────────────────────────────────────────
import sys
import math
import time
import random
import threading
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Dict

# ─────────────────────────────────────────────────────────────────────────────
#  PyQt5
# ─────────────────────────────────────────────────────────────────────────────
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QGroupBox, QGridLayout,
    QSizePolicy, QScrollArea, QSpacerItem
)
from PyQt5.QtCore  import Qt, QTimer, QRectF, QPointF, QSize, pyqtSignal
from PyQt5.QtGui   import (
    QPainter, QPen, QBrush, QColor, QFont, QPainterPath,
    QLinearGradient, QRadialGradient, QPolygonF
)


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 1 — COLOUR PALETTE  (industrial dark-panel HMI)
# ══════════════════════════════════════════════════════════════════════════════

C = {
    "bg":          "#12151c",
    "panel":       "#1c2030",
    "panel_deep":  "#161a26",
    "border":      "#2a3045",
    "border_hi":   "#3a4460",

    "text_pri":    "#dde3ee",
    "text_sec":    "#7a869e",
    "text_dim":    "#3d4560",

    "green":       "#22d46e",
    "green_dim":   "#0d2e1e",
    "amber":       "#f0a020",
    "amber_dim":   "#3a2800",
    "red":         "#e03535",
    "red_dim":     "#381010",
    "blue":        "#3a8eff",
    "blue_dim":    "#0e2545",
    "grey":        "#4a5568",
    "grey_dim":    "#1e2535",

    "belt":        "#252c3d",
    "belt_stripe": "#1e2535",
    "belt_edge":   "#333d55",
    "box_tall":    "#e03535",
    "box_normal":  "#3a8eff",
    "roller":      "#2e3650",
    "frame":       "#161a26",
    "pusher":      "#f0a020",
}


def _qc(key: str, alpha: int = 255) -> QColor:
    """Return a QColor from the palette, with optional alpha."""
    col = QColor(C[key])
    col.setAlpha(alpha)
    return col


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 2 — PLC CORE  (enums, sensors, outputs, timers, FSM controller)
# ══════════════════════════════════════════════════════════════════════════════

# ── 2.1  Enumerations ─────────────────────────────────────────────────────────

class SystemState(Enum):
    """PLC Finite State Machine states."""
    IDLE     = "IDLE"
    RUNNING  = "RUNNING"
    SORTING  = "SORTING"
    FAULT    = "FAULT"


class BoxType(Enum):
    """Height classification of a detected box."""
    TALL   = "TALL"
    NORMAL = "NORMAL"
    NONE   = "NONE"


# ── 2.2  Event Logger ─────────────────────────────────────────────────────────

@dataclass
class LogEntry:
    """One timestamped log record."""
    timestamp: float
    level:     str      # INFO | WARN | FAULT | ACTION
    message:   str

    def formatted(self) -> str:
        t  = time.strftime("%H:%M:%S", time.localtime(self.timestamp))
        ms = int((self.timestamp % 1) * 1000)
        return f"[{t}.{ms:03d}] [{self.level:6s}] {self.message}"


class EventLogger:
    """Thread-safe ring-buffer log (max 400 entries)."""

    MAX = 400

    def __init__(self) -> None:
        self._entries: List[LogEntry] = []
        self._lock = threading.Lock()

    def _add(self, level: str, msg: str) -> None:
        e = LogEntry(time.time(), level, msg)
        with self._lock:
            self._entries.append(e)
            if len(self._entries) > self.MAX:
                self._entries.pop(0)

    def info  (self, m: str) -> None: self._add("INFO",   m)
    def warn  (self, m: str) -> None: self._add("WARN",   m)
    def fault (self, m: str) -> None: self._add("FAULT",  m)
    def action(self, m: str) -> None: self._add("ACTION", m)

    def get_all(self) -> List[LogEntry]:
        with self._lock:
            return list(self._entries)

    def clear(self) -> None:
        with self._lock:
            self._entries.clear()


# ── 2.3  Sensor Inputs ────────────────────────────────────────────────────────

class SensorInputs:
    """
    Simulated PLC digital inputs.

    I:0.0/0  proximity     — box present under sensor
    I:0.0/1  height_tall   — True when box is tall
    I:0.0/2  estop         — Emergency Stop button

    Rising-edge detection (OSR equivalent) counts each box exactly once.
    """

    _BOX_PROB    = 0.016   # probability per scan of a new box arriving
    _MIN_GAP     = 22      # minimum scan cycles between boxes
    _BOX_HOLD_LO = 9
    _BOX_HOLD_HI = 15

    def __init__(self) -> None:
        self.proximity:    bool = False
        self.height_tall:  bool = False
        self.estop:        bool = False

        self._prox_prev:   bool = False
        self.rising_edge:  bool = False   # one-shot

        self._box_active:  bool = False
        self._box_hold:    int  = 0
        self._gap:         int  = 0

        self._lock = threading.Lock()

    def scan(self, conveyor_on: bool) -> None:
        """Execute one sensor read cycle."""
        with self._lock:
            self._simulate(conveyor_on)
            self._edge_detect()

    def trigger_estop(self) -> None:
        with self._lock: self.estop = True

    def release_estop(self) -> None:
        with self._lock: self.estop = False

    def inject_box(self, tall: bool) -> None:
        """Force-inject a box for testing."""
        with self._lock:
            if not self._box_active and self._gap >= self._MIN_GAP:
                self._box_active = True
                self._box_hold   = random.randint(self._BOX_HOLD_LO, self._BOX_HOLD_HI)
                self.height_tall = tall
                self._gap        = 0

    # ── internals ──

    def _simulate(self, conveyor_on: bool) -> None:
        if not conveyor_on:
            self._box_active = False
            self._box_hold   = 0
            self.proximity   = False
            return

        if self._box_active:
            self.proximity  = True
            self._box_hold -= 1
            if self._box_hold <= 0:
                self._box_active = False
                self.proximity   = False
                self._gap        = 0
        else:
            self.proximity = False
            self._gap += 1
            if self._gap >= self._MIN_GAP and random.random() < self._BOX_PROB:
                self._box_active = True
                self._box_hold   = random.randint(self._BOX_HOLD_LO, self._BOX_HOLD_HI)
                self.height_tall = random.random() < 0.45
                self._gap        = 0

    def _edge_detect(self) -> None:
        self.rising_edge = self.proximity and not self._prox_prev
        self._prox_prev  = self.proximity


# ── 2.4  Output Coils ─────────────────────────────────────────────────────────

class OutputCoils:
    """
    Simulated PLC digital outputs.

    O:0.0/0  conveyor_motor   — belt drive contactor
    O:0.0/1  pneumatic_pusher — diverter cylinder solenoid
    O:0.0/2  warning_light    — amber beacon
    """

    def __init__(self) -> None:
        self.conveyor_motor:   bool = False
        self.pneumatic_pusher: bool = False
        self.warning_light:    bool = False

    def reset_all(self) -> None:
        """Safety de-energise pattern — all outputs off."""
        self.conveyor_motor   = False
        self.pneumatic_pusher = False
        self.warning_light    = False

    def as_dict(self) -> dict:
        return {
            "conveyor_motor":   self.conveyor_motor,
            "pneumatic_pusher": self.pneumatic_pusher,
            "warning_light":    self.warning_light,
        }


# ── 2.5  PLC TON Timer ────────────────────────────────────────────────────────

class PLCTimer:
    """Simulated TON (Timer ON-Delay) instruction."""

    def __init__(self, preset: float) -> None:
        self.preset:      float = preset
        self.accumulated: float = 0.0
        self.done:        bool  = False
        self.running:     bool  = False
        self._t0: Optional[float] = None

    def update(self, enable: bool) -> None:
        if enable:
            if not self.running:
                self.running = True
                self._t0     = time.monotonic()
            self.accumulated = time.monotonic() - self._t0
            self.done        = self.accumulated >= self.preset
        else:
            self.reset()

    def reset(self) -> None:
        self.running     = False
        self.done        = False
        self.accumulated = 0.0
        self._t0         = None


# ── 2.6  Box Counters ─────────────────────────────────────────────────────────

class BoxCounters:
    """Production counters."""

    def __init__(self) -> None:
        self.total:  int = 0
        self.tall:   int = 0
        self.normal: int = 0
        self.faults: int = 0

    def count(self, bt: BoxType) -> None:
        self.total += 1
        if bt == BoxType.TALL:
            self.tall += 1
        else:
            self.normal += 1

    def reset(self) -> None:
        self.total = self.tall = self.normal = self.faults = 0


# ── 2.7  PLC Controller ───────────────────────────────────────────────────────

class PLCController:
    """
    Main PLC scan-cycle controller.

    Scan cycle (every 100 ms):
        1. Read Inputs   — sensor simulation tick
        2. Execute Logic — FSM + ladder rungs
        3. Write Outputs — fire GUI callbacks

    State transitions:
        IDLE ──[START]──────────► RUNNING
        RUNNING ──[box edge]────► SORTING
        SORTING ──[sort done]───► RUNNING
        any ──[E-STOP]──────────► FAULT
        FAULT ──[RESET]─────────► IDLE

    Safety interlocks:
        • E-Stop hardwired rung runs BEFORE state logic — highest priority
        • Pusher blocked unless motor is ON (permissive)
        • FAULT locks out START until RESET + E-Stop released
    """

    SCAN_MS       = 100     # scan cycle period (ms)
    DELAY_PRESET  = 1.5     # pusher delay timer preset (s)
    HOLD_PRESET   = 0.6     # pusher hold timer preset (s)
    BLINK_RATE    = 4       # fault warning light blinks per second

    def __init__(self) -> None:
        self.inputs   = SensorInputs()
        self.outputs  = OutputCoils()
        self.counters = BoxCounters()
        self.logger   = EventLogger()

        self._state:         SystemState = SystemState.IDLE
        self._fault_msg:     str         = ""
        self._pending_type:  BoxType     = BoxType.NONE
        self._pusher_armed:  bool        = False
        self._pusher_active: bool        = False
        self._blink_ct:      int         = 0

        self._delay_tmr = PLCTimer(self.DELAY_PRESET)
        self._hold_tmr  = PLCTimer(self.HOLD_PRESET)

        self._lock      = threading.Lock()
        self._stop_evt  = threading.Event()
        self._thread:   Optional[threading.Thread] = None

        # GUI callbacks (set by dashboard)
        self.cb_state:   Optional[Callable] = None
        self.cb_sorted:  Optional[Callable] = None
        self.cb_outputs: Optional[Callable] = None
        self.cb_fault:   Optional[Callable] = None

        self.logger.info("PLC Controller initialised — IDLE.")

    # ── public properties ──

    @property
    def state(self) -> SystemState:
        return self._state

    @property
    def fault_msg(self) -> str:
        return self._fault_msg

    @property
    def pending_type(self) -> BoxType:
        return self._pending_type

    @property
    def delay_timer(self) -> PLCTimer:
        return self._delay_tmr

    # ── operator commands ──

    def cmd_start(self) -> bool:
        with self._lock:
            if self._state == SystemState.FAULT:
                self.logger.warn("START rejected — clear FAULT first.")
                return False
            if self.inputs.estop:
                self.logger.warn("START rejected — E-Stop active.")
                return False
            if self._state == SystemState.IDLE:
                self._go(SystemState.RUNNING)
                self.logger.action("Operator: START")
                return True
            return False

    def cmd_stop(self) -> None:
        with self._lock:
            if self._state in (SystemState.RUNNING, SystemState.SORTING):
                self._go(SystemState.IDLE)
                self.logger.action("Operator: STOP")

    def cmd_estop(self) -> None:
        """Emergency stop — immediate, highest priority."""
        with self._lock:
            self.inputs.trigger_estop()
            self._kill_all()
            self._fault_msg = "EMERGENCY STOP ACTIVATED"
            self._go(SystemState.FAULT)
            self.counters.faults += 1
            self.logger.fault("E-STOP — all outputs de-energised immediately.")
            if self.cb_fault:
                self.cb_fault(self._fault_msg)

    def cmd_reset(self) -> bool:
        with self._lock:
            if self._state != SystemState.FAULT:
                return False
            if self.inputs.estop:
                self.logger.warn("RESET rejected — release E-Stop first.")
                return False
            self._fault_msg = ""
            self._go(SystemState.IDLE)
            self.logger.action("System RESET → IDLE.")
            return True

    def cmd_release_estop(self) -> None:
        self.inputs.release_estop()
        self.logger.info("E-Stop released. Press RESET to resume.")

    def cmd_inject(self, tall: bool) -> None:
        self.inputs.inject_box(tall)
        self.logger.info(f"Manual inject: {'TALL' if tall else 'NORMAL'}")

    def cmd_clear_counters(self) -> None:
        with self._lock:
            self.counters.reset()
        self.logger.info("Production counters reset.")

    # ── scan thread ──

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="PLC-Scan")
        self._thread.start()
        self.logger.info("Scan thread started (100 ms cycle).")

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            self._read_inputs()
            self._execute()
            self._write_outputs()
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, self.SCAN_MS / 1000.0 - elapsed))

    # ── scan rungs ──

    def _read_inputs(self) -> None:
        self.inputs.scan(conveyor_on=self.outputs.conveyor_motor)

    def _execute(self) -> None:
        # ── RUNG 0: Safety interlock (hardwired, highest priority) ────────
        if self.inputs.estop:
            if self._state != SystemState.FAULT:
                self.cmd_estop()
            return

        # ── RUNG 1: State machine ──────────────────────────────────────────
        s = self._state
        if   s == SystemState.IDLE:    self._rung_idle()
        elif s == SystemState.RUNNING: self._rung_running()
        elif s == SystemState.SORTING: self._rung_sorting()
        elif s == SystemState.FAULT:   self._rung_fault()

    def _rung_idle(self) -> None:
        self._kill_all()
        self._delay_tmr.reset()
        self._hold_tmr.reset()
        self._pusher_armed  = False
        self._pusher_active = False
        self._pending_type  = BoxType.NONE

    def _rung_running(self) -> None:
        self.outputs.conveyor_motor   = True
        self.outputs.warning_light    = False
        self.outputs.pneumatic_pusher = False

        if self.inputs.rising_edge:
            self._pending_type  = BoxType.TALL if self.inputs.height_tall else BoxType.NORMAL
            self._pusher_armed  = True
            self._delay_tmr.reset()
            self.logger.info(
                f"Box detected ▶ {self._pending_type.value}  "
                f"| Delay timer armed ({self.DELAY_PRESET}s)")
            self._go(SystemState.SORTING)

    def _rung_sorting(self) -> None:
        self.outputs.conveyor_motor = True   # belt keeps moving
        self.outputs.warning_light  = True   # sort-in-progress lamp

        if self._pusher_armed and not self._pusher_active:
            # Phase 1: delay counting
            self._delay_tmr.update(enable=True)
            if self._delay_tmr.done:
                self._delay_tmr.reset()
                self._pusher_armed = False
                if self._pending_type == BoxType.TALL:
                    # Permissive: motor must be ON
                    if self.outputs.conveyor_motor:
                        self._pusher_active           = True
                        self.outputs.pneumatic_pusher = True
                        self._hold_tmr.reset()
                        self.logger.action("Pusher EXTENDED — diverting TALL box.")
                else:
                    # Normal box — no push needed
                    self.logger.action("NORMAL box — passes through on belt.")
                    self._finish_sort()

        elif self._pusher_active:
            # Phase 2: hold timing
            self._hold_tmr.update(enable=True)
            if self._hold_tmr.done:
                self.outputs.pneumatic_pusher = False
                self._pusher_active = False
                self._hold_tmr.reset()
                self.logger.action("Pusher RETRACTED.")
                self._finish_sort()

    def _rung_fault(self) -> None:
        self.outputs.conveyor_motor   = False
        self.outputs.pneumatic_pusher = False
        # Blink warning light
        blink_half = max(1, int((1000 / self.SCAN_MS) / (2 * self.BLINK_RATE)))
        self._blink_ct += 1
        self.outputs.warning_light = (self._blink_ct % (blink_half * 2)) < blink_half

    def _write_outputs(self) -> None:
        if self.cb_outputs:
            self.cb_outputs(self.outputs.as_dict())

    # ── helpers ──

    def _finish_sort(self) -> None:
        bt = self._pending_type
        self.counters.count(bt)
        self._pending_type = BoxType.NONE
        self.logger.info(
            f"Sort complete ▶ {bt.value} | "
            f"Total:{self.counters.total}  Tall:{self.counters.tall}  "
            f"Normal:{self.counters.normal}")
        if self.cb_sorted:
            self.cb_sorted(bt)
        self._go(SystemState.RUNNING)

    def _kill_all(self) -> None:
        self.outputs.reset_all()
        self._delay_tmr.reset()
        self._hold_tmr.reset()
        self._pusher_armed  = False
        self._pusher_active = False
        self._pending_type  = BoxType.NONE

    def _go(self, new: SystemState) -> None:
        if self._state != new:
            self.logger.info(f"FSM: {self._state.value} → {new.value}")
            self._state = new
            if self.cb_state:
                self.cb_state(new)


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 3 — GUI WIDGETS
# ══════════════════════════════════════════════════════════════════════════════

# ── 3.1  LED Indicator ────────────────────────────────────────────────────────

class LEDIndicator(QWidget):
    """Panel-mount indicator lamp."""

    def __init__(self, color_on: str = "green", sz: int = 16,
                 parent=None) -> None:
        super().__init__(parent)
        self._on       = False
        self._color_on = color_on
        self.setFixedSize(sz, sz)

    def set_state(self, on: bool) -> None:
        if self._on != on:
            self._on = on
            self.update()

    def paintEvent(self, _) -> None:
        p  = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        sz = self.width()
        ck = self._color_on if self._on else "grey_dim"
        bc = _qc(ck)

        if self._on:
            glow = QRadialGradient(sz / 2, sz / 2, sz / 2)
            glow.setColorAt(0.0, bc)
            glow.setColorAt(0.5, QColor(bc.red(), bc.green(), bc.blue(), 160))
            glow.setColorAt(1.0, QColor(bc.red(), bc.green(), bc.blue(), 0))
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(glow))
            p.drawEllipse(0, 0, sz, sz)
        else:
            p.setPen(QPen(_qc("border"), 1))
            p.setBrush(QBrush(bc))
            p.drawEllipse(2, 2, sz - 4, sz - 4)

        # Specular
        hi = QRadialGradient(sz * 0.36, sz * 0.30, sz * 0.25)
        hi.setColorAt(0, QColor(255, 255, 255, 90 if self._on else 25))
        hi.setColorAt(1, QColor(255, 255, 255, 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(hi))
        p.drawEllipse(2, 2, sz - 4, sz - 4)


# ── 3.2  Status Card (counter tile) ──────────────────────────────────────────

class StatusCard(QWidget):
    """Large-value metric tile with coloured left bar."""

    def __init__(self, label: str, col: str = "blue", parent=None) -> None:
        super().__init__(parent)
        self._col = col
        self.setMinimumSize(108, 70)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(16, 8, 10, 6)
        lay.setSpacing(1)

        self._val = QLabel("0")
        self._val.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._val.setFont(QFont("Consolas", 30, QFont.Bold))
        self._val.setStyleSheet(f"color:{C[col]};background:transparent;")

        self._lbl = QLabel(label.upper())
        self._lbl.setAlignment(Qt.AlignLeft)
        self._lbl.setFont(QFont("Consolas", 8))
        self._lbl.setStyleSheet(
            f"color:{C['text_sec']};letter-spacing:2px;background:transparent;")

        lay.addWidget(self._val)
        lay.addWidget(self._lbl)

    def set_value(self, v) -> None:
        self._val.setText(str(v))

    def set_col(self, col: str) -> None:
        self._col = col
        self._val.setStyleSheet(f"color:{C[col]};background:transparent;")
        self.update()

    def paintEvent(self, ev) -> None:
        super().paintEvent(ev)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = self.rect()
        p.setPen(Qt.NoPen)
        p.setBrush(_qc("panel"))
        p.drawRoundedRect(r.adjusted(1, 1, -1, -1), 7, 7)
        p.setBrush(_qc(self._col))
        p.drawRoundedRect(QRectF(1, 10, 4, r.height() - 20), 2, 2)
        p.setPen(QPen(_qc("border"), 1))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(r.adjusted(1, 1, -1, -1), 7, 7)


# ── 3.3  Conveyor Visualisation Widget ───────────────────────────────────────

class ConveyorWidget(QWidget):
    """
    Animated 2-D conveyor belt with:
      • Moving chevron stripes (when running)
      • Box sprite (TALL/NORMAL) travelling across
      • Pneumatic pusher arm (side ejector, below belt)
      • Proximity sensor beam
      • Height sensor indicator
      • Divert-lane labels
    """

    # Relative X positions (0..1 across belt span)
    _PROX_REL  = 0.38
    _PUSH_REL  = 0.62
    _BOX_START = 0.06
    _BOX_END   = 0.96

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(720, 270)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # state fed from GUI refresh
        self._running:   bool        = False
        self._state:     SystemState = SystemState.IDLE
        self._prox:      bool        = False
        self._height:    bool        = False
        self._pusher_on: bool        = False
        self._box_type:  BoxType     = BoxType.NONE

        # animation vars
        self._belt_off:    float = 0.0
        self._box_x:       Optional[float] = None
        self._box_opacity: float = 0.0
        self._push_ext:    float = 0.0   # 0..1
        self._push_ret:    bool  = False
        self._anim_t:      int   = 0
        self._sort_phase:  float = 0.0

    def push_state(self, running: bool, state: SystemState,
                   prox: bool, height: bool,
                   pusher_on: bool, box_type: BoxType) -> None:
        """Called every GUI tick to feed animation state."""
        self._running   = running
        self._state     = state
        self._prox      = prox
        self._height    = height
        self._pusher_on = pusher_on
        self._box_type  = box_type

        # Belt offset
        if running:
            self._belt_off = (self._belt_off + 2.8) % 40

        # Box appearance
        if prox and self._box_x is None:
            self._box_x       = self._BOX_START + 0.12
            self._box_opacity = 0.0

        if self._box_x is not None:
            if running:
                speed = 0.0055
                if self._push_ext > 0.4 and abs(self._box_x - self._PUSH_REL) < 0.14:
                    speed *= 0.08
                self._box_x     += speed
                self._box_opacity = min(1.0, self._box_opacity + 0.12)
            if self._box_x > self._BOX_END:
                self._box_x = None

        if not prox and self._box_x is not None and self._box_x < self._BOX_START + 0.05:
            self._box_x = None

        # Pusher animation
        if pusher_on:
            self._push_ret = False
            self._push_ext = min(1.0, self._push_ext + 0.10)
        else:
            if self._push_ext > 0.0:
                self._push_ret = True
            if self._push_ret:
                self._push_ext = max(0.0, self._push_ext - 0.09)
                if self._push_ext == 0.0:
                    self._push_ret = False

        # Sorting flash
        if state == SystemState.SORTING:
            self._sort_phase = (self._sort_phase + 0.20) % (2 * math.pi)
        else:
            self._sort_phase = 0.0

        self._anim_t += 1
        self.update()

    # ── paint ──

    def paintEvent(self, _) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        p.fillRect(self.rect(), _qc("panel_deep"))
        self._draw_frame(p, W, H)
        self._draw_belt(p, W, H)
        self._draw_prox(p, W, H)
        self._draw_height(p, W, H)
        self._draw_box(p, W, H)
        self._draw_pusher(p, W, H)
        self._draw_labels(p, W, H)
        self._draw_badge(p, W, H)

    def _belt_top(self, H):  return int(H * 0.41)
    def _belt_ht(self):      return 72
    def _margin(self):       return 34

    def _belt_span(self, W):
        m = self._margin()
        return W - 2 * m

    def _abs_x(self, rel, W):
        return self._margin() + rel * self._belt_span(W)

    def _draw_frame(self, p, W, H):
        m  = self._margin()
        by = self._belt_top(H)
        bh = self._belt_ht()
        p.setPen(Qt.NoPen)
        p.setBrush(_qc("frame"))
        p.drawRoundedRect(QRectF(m - 4, by - 12, W - 2*m + 8, bh + 24), 9, 9)
        # rollers
        for rx in [m + 14, W - m - 14]:
            g = QRadialGradient(rx - 4, by + bh//2 - 4, 14)
            g.setColorAt(0, _qc("border_hi"))
            g.setColorAt(1, _qc("roller"))
            p.setBrush(QBrush(g))
            p.setPen(QPen(_qc("border_hi"), 1))
            p.drawEllipse(QPointF(rx, by + bh//2), 14, 14)

    def _draw_belt(self, p, W, H):
        m  = self._margin() + 14
        by = self._belt_top(H)
        bh = self._belt_ht()
        bw = W - 2*m

        p.setClipRect(m, by, bw, bh)
        g = QLinearGradient(0, by, 0, by + bh)
        g.setColorAt(0,    _qc("belt_edge"))
        g.setColorAt(0.12, _qc("belt"))
        g.setColorAt(0.88, _qc("belt"))
        g.setColorAt(1,    _qc("belt_stripe"))
        p.fillRect(QRectF(m, by, bw, bh), QBrush(g))

        sc = _qc("belt_stripe", 210)
        p.setPen(QPen(sc, 1.5))
        pitch  = 38
        offset = int(self._belt_off) % pitch
        for i in range(-2, int(bw / pitch) + 4):
            x0 = m + i * pitch + offset
            p.drawLine(int(x0), by, int(x0 + pitch * 0.65), by + bh)

        p.setClipping(False)
        p.setPen(QPen(_qc("belt_edge"), 2))
        p.drawLine(m, by,      m + bw, by)
        p.drawLine(m, by + bh, m + bw, by + bh)

    def _draw_prox(self, p, W, H):
        px = int(self._abs_x(self._PROX_REL, W))
        by = self._belt_top(H)
        bh = self._belt_ht()
        top = by - 40

        # sensor body
        p.setPen(QPen(_qc("border_hi"), 1))
        p.setBrush(_qc("panel"))
        p.drawRoundedRect(QRectF(px - 11, top, 22, 24), 4, 4)

        # beam
        a = 200 if self._prox else 40
        bc = QColor(C["green"]); bc.setAlpha(a)
        p.setPen(QPen(bc, 2, Qt.DashLine))
        p.drawLine(px, top + 24, px, by + bh + 6)

        # LED dot
        lc = _qc("green" if self._prox else "grey")
        p.setPen(Qt.NoPen); p.setBrush(lc)
        p.drawEllipse(QPointF(px, top + 9), 4, 4)

        p.setPen(_qc("text_sec"))
        p.setFont(QFont("Consolas", 7))
        p.drawText(QRectF(px - 22, top - 15, 44, 12), Qt.AlignCenter, "I:0.0/0")

    def _draw_height(self, p, W, H):
        px = int(self._abs_x(self._PUSH_REL, W))
        by = self._belt_top(H)
        top = by - 40

        lc = _qc("amber" if self._height else "grey")
        p.setPen(QPen(lc, 1)); p.setBrush(lc)
        p.drawEllipse(QPointF(px, top + 9), 5, 5)

        p.setPen(_qc("text_sec"))
        p.setFont(QFont("Consolas", 7))
        p.drawText(QRectF(px - 24, top - 15, 48, 12), Qt.AlignCenter, "I:0.0/1")
        p.drawText(QRectF(px - 24, top - 3, 48, 12), Qt.AlignCenter, "HEIGHT")

    def _draw_box(self, p, W, H):
        if self._box_x is None:
            return
        by = self._belt_top(H)
        px = int(self._abs_x(self._box_x, W))
        is_tall  = (self._box_type == BoxType.TALL)
        BW       = 36
        BH       = 54 if is_tall else 36
        box_top  = by - BH
        ck       = "box_tall" if is_tall else "box_normal"
        base     = QColor(C[ck]); base.setAlpha(int(self._box_opacity * 220))

        # shadow
        sh = QColor(0,0,0, int(self._box_opacity * 70))
        p.setPen(Qt.NoPen); p.setBrush(sh)
        p.drawRoundedRect(QRectF(px - BW//2 + 4, box_top + 4, BW, BH), 3, 3)

        # gradient body
        g = QLinearGradient(px - BW//2, box_top, px + BW//2, box_top)
        lite = QColor(base); lite.setAlpha(int(self._box_opacity * 255))
        dark = QColor(int(base.red()*.6), int(base.green()*.6),
                      int(base.blue()*.6), int(self._box_opacity * 200))
        g.setColorAt(0, lite); g.setColorAt(1, dark)
        p.setBrush(QBrush(g))
        oc = QColor(220,230,255, int(self._box_opacity * 160))
        p.setPen(QPen(oc, 1.2))
        p.drawRoundedRect(QRectF(px - BW//2, box_top, BW, BH), 3, 3)

        # label
        p.setPen(QColor(255,255,255, int(self._box_opacity * 200)))
        p.setFont(QFont("Consolas", 7, QFont.Bold))
        label = "TALL" if is_tall else "NORM"
        p.drawText(QRectF(px - BW//2, box_top + BH//2 - 8, BW, 16),
                   Qt.AlignCenter, label)

    def _draw_pusher(self, p, W, H):
        px   = int(self._abs_x(self._PUSH_REL, W))
        by   = self._belt_top(H)
        bh   = self._belt_ht()
        hy   = by + bh + 10          # housing top
        MAX  = 52                    # max extension pixels (upward)

        # housing
        p.setPen(QPen(_qc("border_hi"), 1.5))
        p.setBrush(_qc("panel"))
        p.drawRoundedRect(QRectF(px - 18, hy, 36, 30), 5, 5)

        # rod
        ext_px = int(self._push_ext * MAX)
        rc     = QColor(C["pusher"]); rc.setAlpha(180 + int(self._push_ext * 75))
        p.setPen(QPen(rc, 7, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(px, hy, px, hy - ext_px)

        # plate
        if self._push_ext > 0.05:
            py2 = hy - ext_px - 7
            pc  = QColor(C["pusher"]); pc.setAlpha(230)
            p.setPen(QPen(pc, 1.5))
            p.setBrush(pc)
            p.drawRoundedRect(QRectF(px - 17, py2, 34, 9), 2, 2)

        # label
        p.setPen(_qc("text_sec"))
        p.setFont(QFont("Consolas", 7))
        p.drawText(QRectF(px - 24, hy + 32, 48, 12), Qt.AlignCenter, "O:0.0/1")
        p.drawText(QRectF(px - 24, hy + 42, 48, 12), Qt.AlignCenter, "PUSHER")

    def _draw_labels(self, p, W, H):
        fa = int(100 + 100 * math.sin(self._sort_phase))

        p.setFont(QFont("Consolas", 9, QFont.Bold))

        # TALL divert lane (upper right)
        tc = QColor(C["box_tall"])
        tc.setAlpha(fa if self._state == SystemState.SORTING
                    and self._box_type == BoxType.TALL else 90)
        p.setPen(tc)
        p.drawText(QRectF(W - 90, H * 0.10, 76, 20),
                   Qt.AlignRight | Qt.AlignVCenter, "▶ TALL")

        # NORMAL lane (lower right)
        nc = QColor(C["box_normal"])
        nc.setAlpha(fa if self._state == SystemState.SORTING
                    and self._box_type == BoxType.NORMAL else 90)
        p.setPen(nc)
        p.drawText(QRectF(W - 90, H * 0.76, 76, 20),
                   Qt.AlignRight | Qt.AlignVCenter, "▶ NORM")

    def _draw_badge(self, p, W, H):
        COLORS = {
            SystemState.IDLE:    ("grey",  "IDLE"),
            SystemState.RUNNING: ("green", "RUNNING"),
            SystemState.SORTING: ("amber", "SORTING"),
            SystemState.FAULT:   ("red",   "FAULT"),
        }
        ck, lbl = COLORS.get(self._state, ("grey", "?"))
        bw, bh  = 88, 24
        p.setPen(QPen(_qc(ck), 1.5))
        p.setBrush(_qc(ck + "_dim"))
        p.drawRoundedRect(QRectF(12, 12, bw, bh), 5, 5)
        p.setPen(_qc(ck))
        p.setFont(QFont("Consolas", 10, QFont.Bold))
        p.drawText(QRectF(12, 12, bw, bh), Qt.AlignCenter, lbl)


# ── 3.4  Log Panel ────────────────────────────────────────────────────────────

class LogPanel(QScrollArea):
    """Scrolling event log with colour-coded severity."""

    COLOURS = {"INFO":"#7a869e","WARN":"#f0a020","FAULT":"#e03535","ACTION":"#22d46e"}

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setStyleSheet(f"""
            QScrollArea {{ border:none; background:{C['panel_deep']}; }}
            QScrollBar:vertical {{ background:{C['panel_deep']}; width:5px; }}
            QScrollBar::handle:vertical {{ background:{C['border_hi']}; border-radius:2px; }}
        """)
        self._inner = QWidget()
        self._inner.setStyleSheet(f"background:{C['panel_deep']};")
        self._lay   = QVBoxLayout(self._inner)
        self._lay.setContentsMargins(8, 6, 6, 6)
        self._lay.setSpacing(0)
        self._lay.addStretch(1)
        self.setWidget(self._inner)
        self._shown = 0
        self._MAX   = 100

    def sync(self, entries: List[LogEntry]) -> None:
        new = len(entries) - self._shown
        if new <= 0:
            return
        for e in entries[-new:]:
            col = self.COLOURS.get(e.level, C["text_sec"])
            lbl = QLabel(e.formatted())
            lbl.setFont(QFont("Consolas", 8))
            lbl.setStyleSheet(f"color:{col};padding:1px 0;")
            lbl.setWordWrap(True)
            self._lay.insertWidget(self._lay.count() - 1, lbl)
        self._shown = len(entries)
        # trim
        while self._lay.count() - 1 > self._MAX:
            item = self._lay.takeAt(0)
            if item and item.widget():
                item.widget().deleteLater()
            self._shown = max(0, self._shown - 1)
        QTimer.singleShot(15, lambda: self.verticalScrollBar().setValue(
            self.verticalScrollBar().maximum()))


# ── 3.5  Small widgets ────────────────────────────────────────────────────────

def _row_sensor(addr: str, label: str, col: str) -> tuple:
    """Returns (QWidget row, LED, value_label)."""
    w   = QWidget()
    lay = QHBoxLayout(w)
    lay.setContentsMargins(6, 2, 6, 2)
    lay.setSpacing(8)
    al  = QLabel(addr)
    al.setFont(QFont("Consolas", 8))
    al.setFixedWidth(66)
    al.setStyleSheet(f"color:{C['text_dim']};")
    led = LEDIndicator(col, 14)
    nl  = QLabel(label)
    nl.setFont(QFont("Consolas", 9))
    nl.setStyleSheet(f"color:{C['text_sec']};")
    vl  = QLabel("0")
    vl.setFont(QFont("Consolas", 9, QFont.Bold))
    vl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    vl.setStyleSheet(f"color:{C['grey']};")
    lay.addWidget(al); lay.addWidget(led); lay.addWidget(nl, 1); lay.addWidget(vl)
    return w, led, vl, col


def _row_output(label: str, col: str) -> tuple:
    w   = QWidget()
    lay = QHBoxLayout(w)
    lay.setContentsMargins(6, 2, 6, 2)
    lay.setSpacing(8)
    led = LEDIndicator(col, 16)
    nl  = QLabel(label)
    nl.setFont(QFont("Consolas", 9))
    nl.setStyleSheet(f"color:{C['text_sec']};")
    sl  = QLabel("OFF")
    sl.setFont(QFont("Consolas", 9, QFont.Bold))
    sl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    sl.setStyleSheet(f"color:{C['grey']};")
    lay.addWidget(led); lay.addWidget(nl, 1); lay.addWidget(sl)
    return w, led, sl, col


def _make_btn(text: str, col: str, fsize: int = 11, h: int = 42) -> QPushButton:
    btn = QPushButton(text)
    btn.setMinimumHeight(h)
    btn.setFont(QFont("Consolas", fsize, QFont.Bold))
    fg  = C[col]
    bg  = C[col + "_dim"]
    btn.setStyleSheet(f"""
        QPushButton {{
            background:{bg}; color:{fg};
            border:1.5px solid {fg}; border-radius:6px;
            padding:5px 12px; letter-spacing:1px; text-align:left;
        }}
        QPushButton:hover  {{ background:{C['border']}; }}
        QPushButton:pressed{{ background:{bg}; }}
        QPushButton:disabled {{
            color:{C['text_dim']}; border-color:{C['border']};
            background:{C['panel_deep']};
        }}
    """)
    return btn


def _group(title: str) -> QGroupBox:
    g = QGroupBox(title)
    g.setFont(QFont("Consolas", 8))
    g.setStyleSheet(f"""
        QGroupBox {{
            border:1px solid {C['border']};border-radius:7px;
            margin-top:16px; padding:10px 4px 6px 4px;
            color:{C['text_dim']};
        }}
        QGroupBox::title{{subcontrol-origin:margin;left:10px;padding:0 5px;letter-spacing:1px;}}
    """)
    return g


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 4 — MAIN DASHBOARD
# ══════════════════════════════════════════════════════════════════════════════

class MainDashboard(QMainWindow):
    """
    Top-level HMI window.

    ┌──────────────────────────────────────────────────────────┐
    │  TITLE BAR                                 ● STATE       │
    ├──────────────────────────────┬─────────────────────────  │
    │  CONVEYOR VISUALISATION      │  COUNTERS + CONTROLS      │
    ├──────────────────────────────┴─────────────────────────  │
    │  SENSORS (I:)  │  OUTPUTS (O:)  │  EVENT LOG             │
    └──────────────────────────────────────────────────────────┘
    """

    def __init__(self) -> None:
        super().__init__()
        self._plc = PLCController()
        self._plc.cb_state   = lambda s: QTimer.singleShot(0, lambda: self._btn_states(s))
        self._plc.cb_sorted  = lambda _: None
        self._plc.cb_outputs = lambda _: None
        self._plc.cb_fault   = lambda _: None

        self._build_ui()
        self._style()

        self._gui  = QTimer(self)
        self._gui.timeout.connect(self._tick)
        self._gui.start(75)          # ~13 fps

        self._plc.start()
        self._btn_states(SystemState.IDLE)

    # ── build UI ──────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.setWindowTitle(
            "PLC Conveyor Sorting System  ·  Industrial HMI Dashboard")
        self.setMinimumSize(1060, 700)

        root = QWidget()
        self.setCentralWidget(root)
        rl = QVBoxLayout(root)
        rl.setContentsMargins(10, 8, 10, 8)
        rl.setSpacing(8)

        # ── title row ──
        tr = QHBoxLayout(); tr.setSpacing(12)
        t  = QLabel("PLC CONVEYOR SORTING SYSTEM")
        t.setFont(QFont("Consolas", 15, QFont.Bold))
        t.setStyleSheet(f"color:{C['text_pri']};letter-spacing:3px;")
        self._badge = QLabel("● IDLE")
        self._badge.setFont(QFont("Consolas", 11, QFont.Bold))
        self._badge.setStyleSheet(f"color:{C['grey']};")
        self._badge.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        tr.addWidget(t); tr.addStretch(1); tr.addWidget(self._badge)
        rl.addLayout(tr)

        sep = QFrame(); sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"color:{C['border']};"); rl.addWidget(sep)

        # ── middle row ──
        mr = QHBoxLayout(); mr.setSpacing(10)

        # Conveyor group (left, expanding)
        cg = _group("Conveyor Belt  ·  Sorting Visualisation")
        ci = QVBoxLayout(); ci.setContentsMargins(6,6,6,6)
        self._conv = ConveyorWidget()
        ci.addWidget(self._conv); cg.setLayout(ci)
        mr.addWidget(cg, stretch=3)

        # Right column
        rc = QVBoxLayout(); rc.setSpacing(8)

        # Counter cards
        cnt = _group("Production Counts")
        cni = QGridLayout(); cni.setSpacing(6); cni.setContentsMargins(8,8,8,8)
        self._c_total  = StatusCard("TOTAL",  "blue")
        self._c_tall   = StatusCard("TALL",   "red")
        self._c_normal = StatusCard("NORMAL", "green")
        self._c_fault  = StatusCard("FAULTS", "amber")
        cni.addWidget(self._c_total,  0,0); cni.addWidget(self._c_tall,   0,1)
        cni.addWidget(self._c_normal, 1,0); cni.addWidget(self._c_fault,  1,1)
        cnt.setLayout(cni); rc.addWidget(cnt)

        # Controls
        ctg = _group("Operator Controls")
        ctl = QVBoxLayout(); ctl.setContentsMargins(8,8,8,8); ctl.setSpacing(7)

        self._b_start   = _make_btn("▶   START",           "green")
        self._b_stop    = _make_btn("■   STOP",            "grey")
        self._b_estop   = _make_btn("⚠   EMERGENCY STOP",  "red", 12, 52)
        self._b_reset   = _make_btn("↺   RESET FAULT",     "amber")
        self._b_release = _make_btn("🔓  RELEASE E-STOP",   "blue")
        self._b_clear   = _make_btn("🗑  CLEAR COUNTERS",   "grey", 9, 30)

        self._b_start.clicked.connect(lambda: self._plc.cmd_start())
        self._b_stop.clicked.connect(lambda: self._plc.cmd_stop())
        self._b_estop.clicked.connect(lambda: self._plc.cmd_estop())
        self._b_reset.clicked.connect(self._do_reset)
        self._b_release.clicked.connect(lambda: self._plc.cmd_release_estop())
        self._b_clear.clicked.connect(lambda: self._plc.cmd_clear_counters())

        for b in (self._b_start, self._b_stop, self._b_estop,
                  self._b_reset, self._b_release, self._b_clear):
            ctl.addWidget(b)

        # Manual inject
        ir = QHBoxLayout(); ir.setSpacing(5)
        il = QLabel("Inject:"); il.setFont(QFont("Consolas", 8))
        il.setStyleSheet(f"color:{C['text_dim']};")
        bi_t = _make_btn("+ TALL",   "red",  8, 28)
        bi_n = _make_btn("+ NORMAL", "blue", 8, 28)
        bi_t.clicked.connect(lambda: self._plc.cmd_inject(True))
        bi_n.clicked.connect(lambda: self._plc.cmd_inject(False))
        ir.addWidget(il); ir.addWidget(bi_t); ir.addWidget(bi_n)
        ctl.addLayout(ir)
        ctg.setLayout(ctl); rc.addWidget(ctg)
        rc.addStretch(1)

        mr.addLayout(rc, stretch=1)
        rl.addLayout(mr)

        # ── bottom row: sensors / outputs / log ──
        br = QHBoxLayout(); br.setSpacing(8)

        # Sensors
        sg = _group("Input Registers  (I:)")
        si = QVBoxLayout(); si.setContentsMargins(2,4,2,4); si.setSpacing(0)
        self._s_prox   = _row_sensor("I:0.0/0","Proximity Sensor","green")
        self._s_height = _row_sensor("I:0.0/1","Height  (Tall)","amber")
        self._s_estop  = _row_sensor("I:0.0/2","E-Stop Button","red")
        self._s_edge   = _row_sensor("EDGE","Rising Edge Det.","blue")
        for r in (self._s_prox, self._s_height, self._s_estop, self._s_edge):
            si.addWidget(r[0])
        si.addStretch(1); sg.setLayout(si); br.addWidget(sg, stretch=1)

        # Outputs
        og = _group("Output Coils  (O:)")
        oi = QVBoxLayout(); oi.setContentsMargins(2,4,2,4); oi.setSpacing(0)
        self._o_motor   = _row_output("O:0.0/0  Conv. Motor","green")
        self._o_pusher  = _row_output("O:0.0/1  Pneum. Pusher","amber")
        self._o_warning = _row_output("O:0.0/2  Warning Light","red")
        for r in (self._o_motor, self._o_pusher, self._o_warning):
            oi.addWidget(r[0])
        oi.addSpacing(8)
        self._timer_lbl = QLabel("Delay timer: 0.00 / 1.50 s")
        self._timer_lbl.setFont(QFont("Consolas", 8))
        self._timer_lbl.setStyleSheet(
            f"color:{C['text_sec']};padding-left:8px;")
        oi.addWidget(self._timer_lbl)
        oi.addStretch(1); og.setLayout(oi); br.addWidget(og, stretch=1)

        # Log
        lg = _group("System Event Log")
        li = QVBoxLayout(); li.setContentsMargins(0,4,0,0)
        self._log = LogPanel()
        self._log.setMinimumHeight(150)
        li.addWidget(self._log); lg.setLayout(li); br.addWidget(lg, stretch=2)

        rl.addLayout(br)

        # ── fault bar ──
        self._fbar = QLabel("  ✔  System nominal — no active faults.")
        self._fbar.setFont(QFont("Consolas", 9))
        self._fbar.setStyleSheet(
            f"background:{C['panel_deep']};color:{C['text_sec']};"
            f"padding:4px 12px;border-top:1px solid {C['border']};")
        rl.addWidget(self._fbar)

    def _style(self) -> None:
        self.setStyleSheet(f"""
            QMainWindow,QWidget {{
                background:{C['bg']};
                color:{C['text_pri']};
                font-family:Consolas,'Courier New',monospace;
            }}
        """)

    # ── GUI tick ──────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        plc   = self._plc
        inp   = plc.inputs
        out   = plc.outputs
        state = plc.state
        ctrs  = plc.counters
        tmr   = plc.delay_timer

        # Conveyor widget
        self._conv.push_state(
            running   = out.conveyor_motor,
            state     = state,
            prox      = inp.proximity,
            height    = inp.height_tall,
            pusher_on = out.pneumatic_pusher,
            box_type  = plc.pending_type,
        )

        # Counter cards
        self._c_total.set_value(ctrs.total)
        self._c_tall.set_value(ctrs.tall)
        self._c_normal.set_value(ctrs.normal)
        self._c_fault.set_value(ctrs.faults)

        # Sensor rows
        self._update_sensor(self._s_prox,   inp.proximity)
        self._update_sensor(self._s_height, inp.height_tall)
        self._update_sensor(self._s_estop,  inp.estop)
        self._update_sensor(self._s_edge,   inp.rising_edge)

        # Output rows
        self._update_output(self._o_motor,   out.conveyor_motor)
        self._update_output(self._o_pusher,  out.pneumatic_pusher)
        self._update_output(self._o_warning, out.warning_light)

        # Timer label
        acc = tmr.accumulated if tmr.running else 0.0
        self._timer_lbl.setText(
            f"Delay timer: {acc:.2f} / {tmr.preset:.2f} s"
            + ("  ✓ DONE" if tmr.done else ""))

        # Status badge
        BADGE = {
            SystemState.IDLE:    (C["grey"],  "● IDLE"),
            SystemState.RUNNING: (C["green"], "● RUNNING"),
            SystemState.SORTING: (C["amber"], "● SORTING"),
            SystemState.FAULT:   (C["red"],   "● FAULT"),
        }
        bc, bl = BADGE.get(state, (C["text_sec"], state.value))
        self._badge.setText(bl)
        self._badge.setStyleSheet(f"color:{bc};")

        # Fault bar
        fm = plc.fault_msg
        if fm:
            self._fbar.setText(f"  ⚠  {fm}")
            self._fbar.setStyleSheet(
                f"background:{C['red_dim']};color:{C['red']};"
                f"padding:4px 12px;border-top:1px solid {C['red']};")
        else:
            self._fbar.setText("  ✔  System nominal — no active faults.")
            self._fbar.setStyleSheet(
                f"background:{C['panel_deep']};color:{C['text_sec']};"
                f"padding:4px 12px;border-top:1px solid {C['border']};")

        # Log
        self._log.sync(plc.logger.get_all())

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _update_sensor(row: tuple, on: bool) -> None:
        _, led, vl, col = row
        led.set_state(on)
        vl.setText("1" if on else "0")
        vl.setStyleSheet(f"color:{C[col]};" if on else f"color:{C['grey']};")

    @staticmethod
    def _update_output(row: tuple, on: bool) -> None:
        _, led, sl, col = row
        led.set_state(on)
        sl.setText("ON" if on else "OFF")
        sl.setStyleSheet(
            f"color:{C[col]};font-weight:bold;" if on
            else f"color:{C['grey']};font-weight:bold;")

    def _do_reset(self) -> None:
        ok = self._plc.cmd_reset()
        if ok:
            self._btn_states(SystemState.IDLE)

    def _btn_states(self, state: SystemState) -> None:
        idle    = state == SystemState.IDLE
        running = state in (SystemState.RUNNING, SystemState.SORTING)
        fault   = state == SystemState.FAULT
        self._b_start.setEnabled(idle)
        self._b_stop.setEnabled(running)
        self._b_reset.setEnabled(fault)
        self._b_release.setEnabled(fault)

    # ── cleanup ───────────────────────────────────────────────────────────────

    def closeEvent(self, ev) -> None:
        self._gui.stop()
        self._plc.stop()
        ev.accept()


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION 5 — ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    # High-DPI support
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps,    True)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    win = MainDashboard()
    win.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
ENDOFFILE