#!/usr/bin/env python3
"""
PS100 Servo Driver — RS-485 Modbus RTU communication wrapper.
Provides position control, status reading, emergency stop, and Err29 recovery.
Configured for single-segment internal position mode (P4-0=1, PA-14=3).
"""
import time
import logging
import minimalmodbus
import serial

logger = logging.getLogger("ps100_driver")

# ── Hardware constants (from实测 calibration) ──
PITCH_MM_PER_REV = 5.0       # Lead screw pitch (mm/rev), verified 2026-07-31
PPR = 10000                   # Pulses per revolution (PA-11)
PULSE_PER_MM = PPR / PITCH_MM_PER_REV  # 2000 pulses/mm

# ── Register addresses ──
REG_PA53_SERVO_ENABLE  = 0x0035   # 1 = enable, 0 = free stop
REG_PA61_CLEAR_ALARM   = 0x003D   # Write 1 to clear alarm
REG_PA60_SOFT_RESET    = 0x003C   # Write 1 to soft-reset driver (clears Err--)
REG_PA54_EEPROM_SAVE   = 0x0036   # Write 1 to save PA-xx params to EEPROM (persist across power cycles)

REG_P4_4_SPEED    = 0x0204   # Target speed (r/min), unsigned
REG_P4_2_TURNS    = 0x0202   # Target revolutions (signed 16-bit)
REG_P4_3_REMAIN   = 0x0203   # Remainder pulses within one revolution (signed 16-bit)
REG_P3_31_TRIGGER = 0x011F   # Virtual trigger: 0→1 rising edge starts motion
REG_P3_34_CLR_ENC = 0x0122   # Clear absolute encoder multi-turn count

REG_SPEED      = 0x1000   # Actual speed (r/min)
REG_POS_LO     = 0x1001   # Position low 16 bits (pulses)
REG_POS_HI     = 0x1002   # Position high 16 bits (pulses)
REG_TORQUE     = 0x1007   # Torque (%)
REG_CURRENT    = 0x1008   # Current (A)
REG_ALARM      = 0x1013   # Alarm code (0 = normal)

# Absolute encoder single-turn / multi-turn values (when P3-37=1)
REG_ABS_SINGLE_LO = 0x1018
REG_ABS_SINGLE_HI = 0x1019
REG_ABS_MULTI_LO  = 0x101A
REG_ABS_MULTI_HI  = 0x101B

# ── Alarm codes ──
ALARM_OVERCURRENT   = 1
ALARM_OVERVOLTAGE   = 2
ALARM_POS_DEVIATION = 4
ALARM_OVERLOAD      = 13
ALARM_TORQUE_OVER   = 29   # Err29: stall protection (PA-30/PA-31)
ALARM_ENCODER       = 50
ALARM_MODBUS_LONG   = 56
ALARM_MODBUS_FORMAT = 57

RECOVERABLE_ALARMS = {1, 4, 13, 29}


class PS100Driver:
    """PS100 servo driver via RS-485 Modbus RTU."""

    def __init__(self, port: str = "/dev/ttyUSB1", slave_addr: int = 1,
                 baudrate: int = 9600, timeout: float = 0.5):
        self.port = port
        self.slave_addr = slave_addr
        self._drv = None
        self._position_mm = 0.0  # Internal position tracking

        # Store serial params for reconnect
        self._baudrate = baudrate
        self._timeout = timeout

    # ── Connection ──

    def connect(self) -> bool:
        """Open RS-485 serial port and verify communication."""
        try:
            self._drv = minimalmodbus.Instrument(self.port, self.slave_addr)
            self._drv.serial.baudrate = self._baudrate
            self._drv.serial.bytesize = 8
            self._drv.serial.parity = serial.PARITY_NONE
            self._drv.serial.stopbits = 2
            self._drv.serial.timeout = self._timeout
            self._drv.serial.rts = True  # RS-485 half-duplex direction control
            # Verify communication
            val = self._drv.read_register(REG_PA53_SERVO_ENABLE)
            logger.info(f"PS100 connected on {self.port}, PA-53={val}")
            self.save_to_eeprom()
            return True
        except Exception as e:
            logger.error(f"Failed to connect PS100 on {self.port}: {e}")
            self._drv = None
            return False

    def disconnect(self):
        """Close serial port."""
        if self._drv and self._drv.serial:
            self._drv.serial.close()
            self._drv = None

    @property
    def connected(self) -> bool:
        return self._drv is not None

    # ── Read operations ──

    def read_position_mm(self) -> float:
        """Return internally tracked position in mm."""
        return self._position_mm

    def set_zero(self):
        """Reset internal position tracking to zero (call at physical origin)."""
        self._position_mm = 0.0
        logger.info("Position set to 0.0 mm")

    def read_speed_rpm(self) -> int:
        """Read actual motor speed in r/min."""
        return self._drv.read_register(REG_SPEED)

    def read_alarm(self) -> int:
        """Read alarm code. 0 = normal."""
        return self._drv.read_register(REG_ALARM)

    def read_torque_pct(self) -> int:
        """Read torque in %."""
        return self._drv.read_register(REG_TORQUE)

    def is_moving(self) -> bool:
        """Check if motor is currently moving."""
        return self._drv.read_register(REG_SPEED) != 0

    # ── Write / control operations ──

    def _write_trigger(self, value: int):
        """Write P3-31 virtual trigger (0 or 1)."""
        self._drv.write_register(REG_P3_31_TRIGGER, value)

    def clear_trigger(self):
        """Clear trigger to prevent residual auto-run."""
        self._write_trigger(0)

    def move_to(self, target_mm: float, speed_rpm: int = 200):
        """
        Absolute positioning — move from current position to target_mm.
        Uses relative single-segment mode: calculates delta and triggers.

        Direction convention: positive target_mm = chassis forward.
        Internally inverts sign (P4-2 positive = backward on hardware).
        """
        delta_mm = target_mm - self._position_mm
        if abs(delta_mm) < 0.001:
            return  # Already at target

        delta_pulses = int(delta_mm * PULSE_PER_MM)
        delta_pulses = -delta_pulses  # Direction fix: P4-2 positive = backward

        turns = delta_pulses // PPR
        remain = delta_pulses % PPR

        # Write parameters (docs §6 verified sequence)
        self._drv.write_register(REG_P4_4_SPEED, speed_rpm)          # ① speed
        self._drv.write_register(REG_P4_2_TURNS, turns, signed=True) # ② turns
        self._drv.write_register(REG_P4_3_REMAIN, remain, signed=True)  # ③ remainder
        self.clear_trigger()                          # ④ clear trigger (critical!)
        time.sleep(0.05)                              # ⑤ 50ms gap
        self._write_trigger(1)                        # ⑥ rising edge → GO!

        self._position_mm = target_mm  # Track internally (abs encoder unreliable)

        logger.debug(f"move_to: {target_mm}mm → {turns}rev+{remain}pulses, "
                     f"speed={speed_rpm}rpm")

    def emergency_stop(self):
        """Free stop — disable servo (PA-53=0)."""
        self._drv.write_register(REG_PA53_SERVO_ENABLE, 0)
        logger.warning("EMERGENCY STOP: servo disabled")

    def re_enable(self):
        """Re-enable servo after emergency stop (PA-53=1)."""
        self._drv.write_register(REG_PA53_SERVO_ENABLE, 1)

    # ── Err29 stall recovery ──

    def save_to_eeprom(self):
        """Persist current PA-xx parameters to EEPROM (PA-54=1).
        
        This must be called after setting PA-30/PA-31 (stall protection)
        or any other PA-xx parameters to make them survive power cycles.
        """
        self._drv.write_register(REG_PA54_EEPROM_SAVE, 1)
        logger.info("PS100: PA-54=1 — parameters saved to EEPROM")

    def recover_err29(self) -> bool:
        """
        Recover from Err29 (stall torque overload).
        Sequence MUST be: clear trigger → disable → wait → clear alarm → re-enable.
        """
        logger.info("Recovering from Err29...")
        try:
            self.clear_trigger()                     # ① clear P3-31 (prevents auto-run!)
            self._drv.write_register(REG_PA53_SERVO_ENABLE, 0)  # ② disable
            time.sleep(1.0)
            self._drv.write_register(REG_PA61_CLEAR_ALARM, 1)  # ③ clear alarm
            time.sleep(0.5)
            self._drv.write_register(REG_PA53_SERVO_ENABLE, 1)  # ④ re-enable
            # Soft reset to clear Err-- display residue
            self._drv.write_register(REG_PA60_SOFT_RESET, 1)
            time.sleep(0.5)
            alarm = self.read_alarm()
            logger.info(f"Err29 recovery done, alarm={alarm}")
            return alarm == 0
        except Exception as e:
            logger.error(f"Err29 recovery failed: {e}")
            return False

    def recover_alarm(self) -> bool:
        """General alarm recovery for recoverable alarms (1,4,13,29)."""
        alarm = self.read_alarm()
        if alarm == 0:
            return True
        if alarm not in RECOVERABLE_ALARMS:
            logger.error(f"Alarm {alarm} is not auto-recoverable")
            return False
        return self.recover_err29()

    # ── Homing ──

    def home(self, speed_rpm: int = 60) -> bool:
        """
        Find origin: move slowly in negative direction until hitting
        mechanical stop (triggers Err29 stall protection), then clear encoder.

        This is required after power cycle because multi-turn count is lost.
        """
        logger.info("Homing: moving to mechanical origin...")
        # Move far enough in negative direction to hit the end
        self.move_to(target_mm=-500.0, speed_rpm=speed_rpm)

        # Wait for stall or completion
        timeout = time.time() + 30.0
        while time.time() < timeout:
            alarm = self.read_alarm()
            if alarm == ALARM_TORQUE_OVER:
                logger.info("Homing: hit mechanical stop (Err29)")
                self.recover_err29()
                break
            if not self.is_moving():
                logger.warning("Homing: motor stopped without stall")
                break
            time.sleep(0.1)

        # Clear encoder multi-turn count
        self.clear_trigger()
        time.sleep(0.1)
        self._drv.write_register(REG_P3_34_CLR_ENC, 1)
        logger.info("Homing: encoder cleared, position = 0")
        return True
