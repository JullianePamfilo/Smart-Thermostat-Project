#!/usr/bin/env python3
"""
SysTec Smart Thermostat — CS 350 Final Project (Raspberry Pi 4B)
File: thermostat.py
Python: 3.9+ on Raspberry Pi OS

I wrote this as a clean, single-file prototype that matches the rubric:
- I2C temperature read (AHT20/AHT21 compatible) via smbus2
- GPIO buttons (mode toggle, temp up, temp down) via gpiozero with interrupt-style callbacks
- PWM LED fade for HEAT (red) and COOL (blue) and solid when set==ambient
- 16x2 LCD (I2C backpack PCF8574) with date/time, ambient, setpoint
- UART (115200-8N1) JSON line every second simulating cloud push
- Explicit state machine (OFF, HEATING, COOLING) driving actions

Hardware assumptions (change pins/addresses if your kit differs):
- AHT20 sensor on I2C bus 1, address 0x38
- LCD 16x2 with PCF8574 backpack at 0x27 (common) on I2C bus 1
- Red LED on BCM pin 17 (PWM capable), Blue LED on BCM pin 27 (PWM capable)
- Buttons (BCM): MODE=22, UP=23, DOWN=24 (internal pull-ups via gpiozero)
- UART: /dev/serial0 at 115200 baud (enable in raspi-config -> Interface Options -> Serial, login shell disabled)

If you don't have certain libraries installed (RPLCD, smbus2, pyserial, gpiozero),
pip install them: pip3 install RPLCD smbus2 pyserial gpiozero

Run:
  python3 thermostat.py
Stop:
  CTRL+C

"""

import time
import json
import threading
from datetime import datetime

# --- External libs ---
try:
    from smbus2 import SMBus, i2c_msg
except Exception as e:
    raise SystemExit("Missing smbus2. Install with: pip3 install smbus2") from e

try:
    from gpiozero import PWMLED, Button
except Exception as e:
    raise SystemExit("Missing gpiozero. Install with: pip3 install gpiozero") from e

try:
    import serial
except Exception as e:
    raise SystemExit("Missing pyserial. Install with: pip3 install pyserial") from e

try:
    # RPLCD is nice for I2C 16x2
    from RPLCD.i2c import CharLCD
except Exception as e:
    raise SystemExit("Missing RPLCD. Install with: pip3 install RPLCD") from e


# ------------------ Constants & Config ------------------
I2C_BUS = 1
AHT20_ADDR = 0x38

LCD_ADDR = 0x27           # Change to 0x3F if your backpack uses that
LCD_COLS, LCD_ROWS = 16, 2

PIN_LED_RED = 17          # HEAT LED
PIN_LED_BLUE = 27         # COOL LED

PIN_BTN_MODE = 22         # cycle OFF -> HEATING -> COOLING -> OFF
PIN_BTN_UP = 23           # increment setpoint
PIN_BTN_DOWN = 24         # decrement setpoint

UART_DEVICE = "/dev/serial0"
UART_BAUD = 115200

# State machine modes
MODE_OFF = 0
MODE_HEATING = 1
MODE_COOLING = 2

# Control loop period (s)
LOOP_PERIOD = 1.0

# Fading timing for PWMLED.pulse()
PULSE_FADE_IN = 1.0
PULSE_FADE_OUT = 1.0
PULSE_N = None            # repeat forever

# Temperature bounds and step
SETPOINT_MIN = 50.0
SETPOINT_MAX = 90.0
SETPOINT_STEP = 0.5


# ------------------ AHT20 Sensor Driver ------------------
class AHT20:
    """Minimal AHT20/AHT21 temperature+humidity reader.

    I follow the datasheet sequence:
      - soft reset (optional)
      - measure command: 0xAC, 0x33, 0x00
      - wait ~80ms and read 6 bytes
      - convert raw T per formula
    I'm only using temperature for this project.
    """

    def __init__(self, bus: int, addr: int = AHT20_ADDR):
        self.addr = addr
        self.bus = SMBus(bus)
        self._init_sensor()

    def _write(self, data: bytes):
        self.bus.write_i2c_block_data(self.addr, data[0], list(data[1:]) if len(data) > 1 else [])

    def _read(self, n: int) -> bytes:
        read = i2c_msg.read(self.addr, n)
        self.bus.i2c_rdwr(read)
        return bytes(list(read))

    def _init_sensor(self):
        # Soft reset
        try:
            self._write(bytes([0xBA]))
            time.sleep(0.02)
        except Exception:
            pass  # Some AHT20s don't require it

        # Issue a status read to wake (optional)
        try:
            _ = self._read(1)
        except Exception:
            pass
        time.sleep(0.01)

    def read_temperature_c(self) -> float:
        # Trigger measurement
        self._write(bytes([0xAC, 0x33, 0x00]))
        time.sleep(0.085)  # datasheet ~80ms

        data = self._read(6)
        if len(data) != 6:
            raise IOError("AHT20: bad read length")

        # data[0] contains status in top bit; ignore here
        # Extract 20-bit temp: bits: T_raw = ((data[3] & 0x0F)<<16) | (data[4]<<8) | data[5]
        t_raw = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        temp_c = (t_raw / 1048576.0) * 200.0 - 50.0
        return round(temp_c, 2)


# ------------------ LCD Helper ------------------
class Display:
    """Simple wrapper around RPLCD CharLCD. I like to centralize the formatting here."""
    def __init__(self, i2c_addr=LCD_ADDR, cols=LCD_COLS, rows=LCD_ROWS):
        self.lcd = CharLCD(i2c_expander='PCF8574', address=i2c_addr, port=I2C_BUS,
                           cols=cols, rows=rows, charmap='A02', auto_linebreaks=False)
        self.clear()

    def clear(self):
        self.lcd.clear()

    def show(self, line1: str, line2: str):
        # I make sure each line fits 16 chars
        l1 = (line1[:LCD_COLS]).ljust(LCD_COLS)
        l2 = (line2[:LCD_COLS]).ljust(LCD_COLS)
        self.lcd.home()
        self.lcd.cursor_pos = (0, 0)
        self.lcd.write_string(l1)
        self.lcd.cursor_pos = (1, 0)
        self.lcd.write_string(l2)
        


# ------------------ State Machine Controller ------------------
class ThermostatController:
    """I keep all runtime state and IO objects in here."""
    def __init__(self):
        # IO
        self.sensor = AHT20(bus=I2C_BUS, addr=AHT20_ADDR)
        self.lcd = Display()
        self.led_heat = PWMLED(PIN_LED_RED)
        self.led_cool = PWMLED(PIN_LED_BLUE)

        self.btn_mode = Button(PIN_BTN_MODE, pull_up=True, bounce_time=0.2)
        self.btn_up = Button(PIN_BTN_UP, pull_up=True, bounce_time=0.15)
        self.btn_down = Button(PIN_BTN_DOWN, pull_up=True, bounce_time=0.15)

        # Serial
        self.ser = serial.Serial(UART_DEVICE, UART_BAUD, timeout=0.1)

        # State
        self.mode = MODE_OFF
        self.setpoint = 72.0
        self.ambient_c = 22.0
        self.target_units = "F"    # Display in Fahrenheit for users in the U.S.

        # Wiring up interrupts (gpiozero calls them from background threads)
        self.btn_mode.when_pressed = self._on_mode
        self.btn_up.when_pressed = self._on_up
        self.btn_down.when_pressed = self._on_down

        # I keep a lock since interrupts can fire during loop updates
        self._lock = threading.Lock()

    # ----- Button interrupts -----
    def _on_mode(self):
        with self._lock:
            self.mode = (self.mode + 1) % 3  # OFF->HEAT->COOL->OFF
            # I stop any ongoing pulses so outputs reflect new mode immediately
            self.led_heat.off()
            self.led_cool.off()

    def _on_up(self):
        with self._lock:
            self.setpoint = min(SETPOINT_MAX, self.setpoint + SETPOINT_STEP)

    def _on_down(self):
        with self._lock:
            self.setpoint = max(SETPOINT_MIN, self.setpoint - SETPOINT_STEP)

    # ----- Helpers -----
    @staticmethod
    def c_to_f(c):
        return c * 9.0/5.0 + 32.0

    def _format_temp(self, c):
        if self.target_units.upper() == "F":
            return f"{self.c_to_f(c):.1f}F"
        return f"{c:.1f}C"

    # ----- Outputs -----
    def _drive_leds(self):
        """I drive the LEDs based on state.
        - In HEATING: red fades unless ambient==setpoint (solid on)
        - In COOLING: blue fades unless ambient==setpoint (solid on)
        - In OFF: both off
        """
        # First, kill any ongoing pulses (so we don't stack calls)
        self.led_heat.off()
        self.led_cool.off()

        # Determine equality in the same units (use Fahrenheit)
        amb_f = self.c_to_f(self.ambient_c)
        eq = abs(amb_f - self.setpoint) < 0.25  # small deadband

        if self.mode == MODE_OFF:
            self.led_heat.off()
            self.led_cool.off()
        elif self.mode == MODE_HEATING:
            if eq:
                self.led_heat.value = 1.0  # solid on
            else:
                self.led_heat.pulse(fade_in_time=PULSE_FADE_IN, fade_out_time=PULSE_FADE_OUT, n=PULSE_N,
                                    background=True)
            self.led_cool.off()
        elif self.mode == MODE_COOLING:
            if eq:
                self.led_cool.value = 1.0
            else:
                self.led_cool.pulse(fade_in_time=PULSE_FADE_IN, fade_out_time=PULSE_FADE_OUT, n=PULSE_N,
                                    background=True)
            self.led_heat.off()

    def _update_lcd(self):
        now = datetime.now().strftime("%m-%d %H:%M")
        line1 = f"{now}  {self._format_temp(self.ambient_c)}"
        line2 = f"Set:{self.setpoint:4.1f}{self.target_units} "
        mode_txt = {MODE_OFF:"OFF ", MODE_HEATING:"HEAT", MODE_COOLING:"COOL"}[self.mode]
        line2 += mode_txt
        self.lcd.show(line1, line2)

    def _push_uart(self):
        payload = {
            "ts": datetime.now().isoformat(timespec="seconds"),
            "mode": ["OFF", "HEAT", "COOL"][self.mode],
            "ambient_c": round(self.ambient_c, 2),
            "ambient_f": round(self.c_to_f(self.ambient_c), 2),
            "setpoint_f": round(self.setpoint, 1)
        }
        line = json.dumps(payload) + "\n"
        try:
            self.ser.write(line.encode("utf-8"))
            print(line)
        except Exception:
            # If UART isn't up, I fail soft so the rest keeps running
            pass

    # ----- Main loop -----
    def run(self):
        try:
            while True:
                with self._lock:
                    # Read sensor
                    self.ambient_c = self.sensor.read_temperature_c()

                    # Update outputs
                    self._drive_leds()
                    self._update_lcd()
                    self._push_uart()

                time.sleep(LOOP_PERIOD)
        except KeyboardInterrupt:
            pass
        finally:
            # I like to leave hardware in a known, safe state
            self.led_heat.off()
            self.led_cool.off()
            self.lcd.clear()
            try:
                self.ser.close()
            except Exception:
                pass


if __name__ == "__main__":
    ctrl = ThermostatController()
    ctrl.run()


