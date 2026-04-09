#!/usr/bin/env python3
"""
Raw motor control web dashboard for SatNOGS rotator bench testing.

Opens a browser UI at http://localhost:8080
Keyboard controls work when the page is focused.

  AZ: A/D = -/+ 5%    Q/E = -/+ 1%
  EL: W/S = +/- 5%    R/F = +/- 1%
  SPACE = stop all     0 = zero encoders
  1-9 = AZ duty 10%-90%
  P = park (PID)

Usage:
    python3 motor_control.py [serial_port]
"""

import serial
import time
import sys
import glob
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

AZ_TICKS_PER_DEG = 244.6
EL_TICKS_PER_DEG = 126.1
MAX_DUTY = 0.95


def auto_detect_serial():
    for p in sorted(glob.glob("/dev/cu.usbmodem*")):
        return p
    for p in sorted(glob.glob("/dev/ttyACM*")):
        return p
    return None


class MotorController:
    def __init__(self, port_name):
        self.ser = serial.Serial(port_name, 115200, timeout=0.02)
        self.lock = threading.Lock()
        time.sleep(2)
        while self.ser.in_waiting:
            self.ser.readline()

        self.az_duty = 0.0
        self.el_duty = 0.0
        self.az_ticks = 0
        self.el_ticks = 0
        self.prev_az_ticks = 0
        self.prev_el_ticks = 0
        self.az_speed = 0.0
        self.el_speed = 0.0
        self.az_deg = 0.0
        self.el_deg = 0.0
        self.firmware = "?"
        self.pid_mode = False
        self.last_tick_time = time.time()

        lines = self._cmd("VE")
        for l in lines:
            if l.startswith("VE"):
                self.firmware = l[2:]

    def _cmd(self, cmd):
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(f"{cmd}\n".encode())
            time.sleep(0.03)
            lines = []
            deadline = time.time() + 0.08
            while time.time() < deadline:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        lines.append(line)
                else:
                    time.sleep(0.003)
            return lines

    def poll(self):
        lines = self._cmd("TICKS")
        now = time.time()
        dt = now - self.last_tick_time

        for line in lines:
            if "AZ_TICKS=" in line:
                try:
                    parts = line.split()
                    self.az_ticks = int(parts[0].split("=")[1])
                    self.el_ticks = int(parts[1].split("=")[1])
                except (ValueError, IndexError):
                    pass

        if dt > 0.05:
            self.az_speed = (self.az_ticks - self.prev_az_ticks) / AZ_TICKS_PER_DEG / dt
            self.el_speed = (self.el_ticks - self.prev_el_ticks) / EL_TICKS_PER_DEG / dt
            self.prev_az_ticks = self.az_ticks
            self.prev_el_ticks = self.el_ticks
            self.last_tick_time = now

        self.az_deg = self.az_ticks / AZ_TICKS_PER_DEG
        self.el_deg = self.el_ticks / EL_TICKS_PER_DEG

    def set_az_duty(self, duty):
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        self.az_duty = duty
        self.pid_mode = False
        self._cmd(f"DUTY_AZ{duty:.3f}")

    def set_el_duty(self, duty):
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        self.el_duty = duty
        self.pid_mode = False
        self._cmd(f"DUTY_EL{duty:.3f}")

    def stop(self):
        self.az_duty = 0.0
        self.el_duty = 0.0
        self.pid_mode = False
        self._cmd("STOP")

    def zero(self):
        self._cmd("ZERO")
        self.az_ticks = 0
        self.el_ticks = 0
        self.prev_az_ticks = 0
        self.prev_el_ticks = 0
        self.az_deg = 0.0
        self.el_deg = 0.0

    def park(self):
        self.pid_mode = True
        self.az_duty = 0.0
        self.el_duty = 0.0
        self._cmd("PARK")

    def goto(self, az, el):
        self.pid_mode = True
        self.az_duty = 0.0
        self.el_duty = 0.0
        self._cmd(f"AZ{az:.1f} EL{el:.1f}")

    def state(self):
        return {
            "az_duty": round(self.az_duty * 100, 1),
            "el_duty": round(self.el_duty * 100, 1),
            "az_deg": round(self.az_deg, 1),
            "el_deg": round(self.el_deg, 1),
            "az_speed": round(self.az_speed, 1),
            "el_speed": round(self.el_speed, 1),
            "az_ticks": self.az_ticks,
            "el_ticks": self.el_ticks,
            "pid_mode": self.pid_mode,
            "firmware": self.firmware,
        }

    def close(self):
        self.stop()
        self.ser.close()


HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Motor Control</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { background: #111; color: #eee; font-family: 'SF Mono', 'Fira Code', monospace; padding: 20px; }
h1 { color: #0ff; font-size: 22px; margin-bottom: 4px; }
.header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; border-bottom: 1px solid #333; padding-bottom: 12px; }
.header-right { text-align: right; font-size: 13px; color: #888; }
.mode { font-weight: bold; font-size: 16px; }
.mode.raw { color: #fa0; }
.mode.pid { color: #0f0; }
.axes { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px; }
.axis { background: #1a1a1a; border: 1px solid #333; border-radius: 8px; padding: 16px; }
.axis h2 { color: #0f0; font-size: 16px; margin-bottom: 12px; }
.stat { display: flex; justify-content: space-between; margin-bottom: 6px; font-size: 14px; }
.stat .label { color: #888; }
.stat .value { color: #fff; font-weight: bold; }
.stat .value.moving { color: #0ff; }
.stat .value.warning { color: #f44; }

/* Duty bar */
.duty-bar-wrap { margin: 12px 0; }
.duty-bar { height: 28px; background: #222; border-radius: 4px; position: relative; overflow: hidden; border: 1px solid #444; }
.duty-bar .center { position: absolute; left: 50%; top: 0; bottom: 0; width: 2px; background: #666; }
.duty-bar .fill { position: absolute; top: 2px; bottom: 2px; background: #0a0; border-radius: 2px; transition: left 0.08s, width 0.08s; }
.duty-bar .fill.neg { background: #a00; }
.duty-label { text-align: center; font-size: 20px; font-weight: bold; margin-top: 4px; }

/* Buttons */
.controls { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; margin-bottom: 20px; }
.btn { background: #222; border: 1px solid #444; color: #eee; padding: 14px 8px; border-radius: 6px;
       font-family: inherit; font-size: 14px; cursor: pointer; text-align: center; user-select: none;
       transition: background 0.1s; }
.btn:hover { background: #333; }
.btn:active, .btn.active { background: #444; border-color: #0ff; }
.btn.stop { background: #400; border-color: #a00; color: #f44; font-weight: bold; font-size: 18px; }
.btn.stop:hover { background: #600; }
.btn.park { background: #140; border-color: #0a0; }

.duty-btns { display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; margin-bottom: 16px; }
.duty-btns h3 { grid-column: 1 / -1; color: #888; font-size: 13px; font-weight: normal; margin-bottom: 2px; }
.dbtn { background: #1a1a2a; border: 1px solid #335; color: #aaf; padding: 10px 4px; border-radius: 4px;
        font-family: inherit; font-size: 13px; cursor: pointer; text-align: center; }
.dbtn:hover { background: #2a2a4a; }
.dbtn:active { background: #3a3a5a; border-color: #0ff; }

.slider-row { display: flex; align-items: center; gap: 10px; margin: 8px 0; }
.slider-row label { color: #888; font-size: 13px; min-width: 25px; }
.slider-row input[type=range] { flex: 1; accent-color: #0f0; }
.slider-row .sval { min-width: 55px; text-align: right; font-size: 14px; font-weight: bold; }

.keys { background: #1a1a1a; border: 1px solid #333; border-radius: 8px; padding: 14px; font-size: 13px; color: #888; line-height: 1.8; }
.keys kbd { background: #333; color: #fff; padding: 2px 7px; border-radius: 3px; border: 1px solid #555; font-family: inherit; }
</style>
</head>
<body>

<div class="header">
  <div>
    <h1>SatNOGS Motor Control</h1>
  </div>
  <div class="header-right">
    <div id="fw">...</div>
    <div class="mode" id="mode">---</div>
  </div>
</div>

<div class="axes">
  <div class="axis">
    <h2>AZIMUTH</h2>
    <div class="duty-bar-wrap">
      <div class="duty-bar"><div class="center"></div><div class="fill" id="az-fill"></div></div>
      <div class="duty-label" id="az-duty-label">0.0%</div>
    </div>
    <div class="slider-row">
      <label>AZ</label>
      <input type="range" id="az-slider" min="-95" max="95" value="0" step="1">
      <span class="sval" id="az-slider-val">0%</span>
    </div>
    <div class="stat"><span class="label">Position</span><span class="value" id="az-pos">0.0°</span></div>
    <div class="stat"><span class="label">Speed</span><span class="value" id="az-spd">0.0 °/s</span></div>
    <div class="stat"><span class="label">Ticks</span><span class="value" id="az-ticks">0</span></div>
  </div>
  <div class="axis">
    <h2>ELEVATION</h2>
    <div class="duty-bar-wrap">
      <div class="duty-bar"><div class="center"></div><div class="fill" id="el-fill"></div></div>
      <div class="duty-label" id="el-duty-label">0.0%</div>
    </div>
    <div class="slider-row">
      <label>EL</label>
      <input type="range" id="el-slider" min="-95" max="95" value="0" step="1">
      <span class="sval" id="el-slider-val">0%</span>
    </div>
    <div class="stat"><span class="label">Position</span><span class="value" id="el-pos">0.0°</span></div>
    <div class="stat"><span class="label">Speed</span><span class="value" id="el-spd">0.0 °/s</span></div>
    <div class="stat"><span class="label">Ticks</span><span class="value" id="el-ticks">0</span></div>
  </div>
</div>

<div class="controls">
  <button class="btn stop" onclick="cmd('stop')" id="btn-stop">STOP</button>
  <button class="btn park" onclick="cmd('park')">PARK</button>
  <button class="btn" onclick="cmd('zero')">ZERO ENC</button>
  <button class="btn" onclick="doGoto()">GOTO</button>
</div>

<div class="duty-btns">
  <h3>AZ Quick Duty</h3>
  <button class="dbtn" onclick="cmd('az_duty', -50)">-50%</button>
  <button class="dbtn" onclick="cmd('az_duty', -25)">-25%</button>
  <button class="dbtn" onclick="cmd('az_duty', 0)">0%</button>
  <button class="dbtn" onclick="cmd('az_duty', 25)">+25%</button>
  <button class="dbtn" onclick="cmd('az_duty', 50)">+50%</button>
</div>
<div class="duty-btns">
  <h3>EL Quick Duty</h3>
  <button class="dbtn" onclick="cmd('el_duty', -50)">-50%</button>
  <button class="dbtn" onclick="cmd('el_duty', -25)">-25%</button>
  <button class="dbtn" onclick="cmd('el_duty', 0)">0%</button>
  <button class="dbtn" onclick="cmd('el_duty', 25)">+25%</button>
  <button class="dbtn" onclick="cmd('el_duty', 50)">+50%</button>
</div>

<div class="keys">
  <kbd>A</kbd>/<kbd>D</kbd> AZ -/+5% &nbsp;
  <kbd>Q</kbd>/<kbd>E</kbd> AZ -/+1% &nbsp;
  <kbd>W</kbd>/<kbd>S</kbd> EL +/-5% &nbsp;
  <kbd>R</kbd>/<kbd>F</kbd> EL +/-1% &nbsp;
  <kbd>Space</kbd> Stop &nbsp;
  <kbd>0</kbd> Zero &nbsp;
  <kbd>P</kbd> Park &nbsp;
  <kbd>1</kbd>-<kbd>9</kbd> AZ 10-90%
</div>

<script>
let state = {};
const STEP = 5, FINE = 1;

function cmd(action, value) {
  fetch('/cmd', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action, value: value || 0})
  });
}

function doGoto() {
  const az = prompt('AZ degrees:');
  if (az === null) return;
  const el = prompt('EL degrees:');
  if (el === null) return;
  fetch('/cmd', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({action: 'goto', az: parseFloat(az), el: parseFloat(el)})
  });
}

function updateBar(id, duty) {
  const fill = document.getElementById(id);
  const pct = Math.abs(duty) / 95 * 50;
  if (duty >= 0) {
    fill.style.left = '50%';
    fill.style.width = pct + '%';
    fill.className = 'fill';
  } else {
    fill.style.left = (50 - pct) + '%';
    fill.style.width = pct + '%';
    fill.className = 'fill neg';
  }
}

function poll() {
  fetch('/state').then(r => r.json()).then(s => {
    state = s;
    document.getElementById('fw').textContent = 'FW: ' + s.firmware;
    const modeEl = document.getElementById('mode');
    modeEl.textContent = s.pid_mode ? '[PID]' : '[RAW]';
    modeEl.className = 'mode ' + (s.pid_mode ? 'pid' : 'raw');

    updateBar('az-fill', s.az_duty);
    updateBar('el-fill', s.el_duty);
    document.getElementById('az-duty-label').textContent = s.az_duty.toFixed(1) + '%';
    document.getElementById('el-duty-label').textContent = s.el_duty.toFixed(1) + '%';

    const azPos = document.getElementById('az-pos');
    azPos.textContent = s.az_deg.toFixed(1) + '°';
    azPos.className = 'value' + (Math.abs(s.az_deg) > 300 ? ' warning' : '');

    const elPos = document.getElementById('el-pos');
    elPos.textContent = s.el_deg.toFixed(1) + '°';
    elPos.className = 'value' + (s.el_deg > 160 || s.el_deg < -10 ? ' warning' : '');

    const azSpd = document.getElementById('az-spd');
    azSpd.textContent = s.az_speed.toFixed(1) + ' °/s';
    azSpd.className = 'value' + (Math.abs(s.az_speed) > 0.5 ? ' moving' : '');

    const elSpd = document.getElementById('el-spd');
    elSpd.textContent = s.el_speed.toFixed(1) + ' °/s';
    elSpd.className = 'value' + (Math.abs(s.el_speed) > 0.5 ? ' moving' : '');

    document.getElementById('az-ticks').textContent = s.az_ticks;
    document.getElementById('el-ticks').textContent = s.el_ticks;
  }).catch(() => {});
}

// Sliders
document.getElementById('az-slider').addEventListener('input', function() {
  document.getElementById('az-slider-val').textContent = this.value + '%';
});
document.getElementById('az-slider').addEventListener('change', function() {
  cmd('az_duty', parseInt(this.value));
});
document.getElementById('el-slider').addEventListener('input', function() {
  document.getElementById('el-slider-val').textContent = this.value + '%';
});
document.getElementById('el-slider').addEventListener('change', function() {
  cmd('el_duty', parseInt(this.value));
});

// Keyboard
document.addEventListener('keydown', function(e) {
  if (e.target.tagName === 'INPUT') return;
  const k = e.key;
  if (k === ' ') { e.preventDefault(); cmd('stop'); }
  else if (k === 'd') cmd('az_duty', (state.az_duty || 0) + STEP);
  else if (k === 'a') cmd('az_duty', (state.az_duty || 0) - STEP);
  else if (k === 'e') cmd('az_duty', (state.az_duty || 0) + FINE);
  else if (k === 'q') cmd('az_duty', (state.az_duty || 0) - FINE);
  else if (k === 'w') cmd('el_duty', (state.el_duty || 0) + STEP);
  else if (k === 's') { e.preventDefault(); cmd('el_duty', (state.el_duty || 0) - STEP); }
  else if (k === 'r') cmd('el_duty', (state.el_duty || 0) + FINE);
  else if (k === 'f') cmd('el_duty', (state.el_duty || 0) - FINE);
  else if (k === 'p') cmd('park');
  else if (k === '0') cmd('zero');
  else if (k >= '1' && k <= '9') cmd('az_duty', parseInt(k) * 10);
});

setInterval(poll, 100);
poll();
</script>
</body>
</html>"""


class Handler(BaseHTTPRequestHandler):
    mc = None

    def log_message(self, format, *args):
        pass  # silence request logs

    def do_GET(self):
        if self.path == "/state":
            self.mc.poll()
            data = json.dumps(self.mc.state()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())

    def do_POST(self):
        if self.path == "/cmd":
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length))
            action = body.get("action", "")
            value = body.get("value", 0)

            if action == "stop":
                self.mc.stop()
            elif action == "park":
                self.mc.park()
            elif action == "zero":
                self.mc.zero()
            elif action == "az_duty":
                self.mc.set_az_duty(value / 100.0)
            elif action == "el_duty":
                self.mc.set_el_duty(value / 100.0)
            elif action == "goto":
                az = body.get("az", 0)
                el = body.get("el", 0)
                self.mc.goto(az, el)

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')


def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else auto_detect_serial()
    if port_name is None:
        print("No serial port found. Usage: python3 motor_control.py /dev/cu.usbmodemXXXX")
        sys.exit(1)

    print(f"Connecting to {port_name}...")
    mc = MotorController(port_name)
    print(f"Firmware: {mc.firmware}")

    Handler.mc = mc
    HTTPServer.allow_reuse_address = True
    server = HTTPServer(("0.0.0.0", 8080), Handler)
    print(f"\n  Dashboard: http://localhost:8080\n")
    print("  Press Ctrl+C to stop.\n")

    try:
        import webbrowser
        webbrowser.open("http://localhost:8080")
    except Exception:
        pass

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping motors...")
    finally:
        mc.close()
        server.server_close()
        print("Done.")


if __name__ == "__main__":
    main()
