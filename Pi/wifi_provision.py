#!/usr/bin/env python3
"""
WiFi Provisioning — AP-based captive portal for field WiFi setup.

On boot, if no known WiFi is available, starts a temporary access point
("SatNOGS-Setup") with a simple web page where the user enters their
phone hotspot SSID + password. Once connected, the AP shuts down.

Workflow:
    1. Pi boots → systemd runs wifi_provision.py
    2. No known WiFi? → start AP "SatNOGS-Setup" (open, no password)
    3. User connects phone → captive portal page
    4. User enters hotspot SSID + password → Pi saves & connects
    5. AP goes down, Pi is on the hotspot with internet
    6. Next boot: Pi finds saved WiFi → skips provisioning entirely

Saved networks persist in NetworkManager — survives reboots.
"""

import os
import sys
import json
import time
import signal
import subprocess
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

AP_SSID = "SatNOGS-Setup"
AP_IFACE = "wlan0"
AP_IP = "192.168.4.1"
AP_CON_NAME = "satnogs-ap"
PROVISION_PORT = 80
STATION_CONF = os.path.expanduser("~/station.conf")

# How long to wait for WiFi before starting AP (seconds)
WIFI_WAIT_TIMEOUT = 30

# Track if provisioning is done
_provisioned = False


def run(cmd, check=False):
    """Run a shell command, return (returncode, stdout)."""
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if check and r.returncode != 0:
        raise RuntimeError(f"Command failed: {cmd}\n{r.stderr}")
    return r.returncode, r.stdout.strip()


def is_wifi_connected():
    """Check if wlan0 has an IP address (connected to some WiFi)."""
    rc, out = run("nmcli -t -f DEVICE,STATE dev status")
    for line in out.splitlines():
        if line.startswith(f"{AP_IFACE}:connected"):
            return True
    return False


def get_saved_networks():
    """List saved WiFi SSIDs from NetworkManager."""
    rc, out = run("nmcli -t -f NAME,TYPE con show")
    networks = []
    for line in out.splitlines():
        parts = line.split(":")
        if len(parts) >= 2 and parts[1] == "802-11-wireless":
            name = parts[0]
            if name != AP_CON_NAME:
                networks.append(name)
    return networks


def wait_for_wifi(timeout=WIFI_WAIT_TIMEOUT):
    """Wait for WiFi connection. Returns True if connected."""
    print(f"Waiting {timeout}s for WiFi connection...")
    for i in range(timeout):
        if is_wifi_connected():
            print("WiFi connected!")
            return True
        time.sleep(1)
    print("No WiFi connection found.")
    return False


def start_ap():
    """Start the provisioning access point."""
    print(f"Starting AP: {AP_SSID}")

    # Remove old AP connection if it exists
    run(f"sudo nmcli con delete {AP_CON_NAME} 2>/dev/null")

    # Create AP connection
    run(f"sudo nmcli con add con-name {AP_CON_NAME} "
        f"ifname {AP_IFACE} type wifi ssid '{AP_SSID}' "
        f"802-11-wireless.mode ap 802-11-wireless.band bg "
        f"ipv4.method shared ipv4.addresses {AP_IP}/24 "
        f"connection.autoconnect no", check=True)

    # Start it
    run(f"sudo nmcli con up {AP_CON_NAME}", check=True)
    print(f"AP started: {AP_SSID} at {AP_IP}")


def stop_ap():
    """Stop and remove the provisioning AP."""
    print("Stopping AP...")
    run(f"sudo nmcli con down {AP_CON_NAME} 2>/dev/null")
    run(f"sudo nmcli con delete {AP_CON_NAME} 2>/dev/null")


def add_wifi_network(ssid, password):
    """Add a WiFi network to NetworkManager and try to connect."""
    print(f"Adding WiFi network: {ssid}")

    # Delete if already exists (to update password)
    run(f"sudo nmcli con delete '{ssid}' 2>/dev/null")

    # Add the connection
    rc, out = run(
        f"sudo nmcli con add con-name '{ssid}' ifname {AP_IFACE} "
        f"type wifi ssid '{ssid}' "
        f"wifi-sec.key-mgmt wpa-psk wifi-sec.psk '{password}' "
        f"connection.autoconnect yes connection.autoconnect-priority 10"
    )
    if rc != 0:
        return False, f"Failed to add network: {out}"

    # Stop AP first so wlan0 is free
    stop_ap()
    time.sleep(2)

    # Try to connect
    rc, out = run(f"sudo nmcli con up '{ssid}'")
    if rc != 0:
        return False, f"Failed to connect: {out}"

    # Verify connection
    time.sleep(3)
    if is_wifi_connected():
        # Get the IP
        _, ip_out = run(f"hostname -I")
        return True, ip_out.split()[0] if ip_out else "unknown"

    return False, "Connected but no IP"


PORTAL_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SatNOGS WiFi Setup</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:-apple-system,system-ui,sans-serif;background:#0a0e1a;color:#f7f7f2;
       display:flex;align-items:center;justify-content:center;min-height:100vh;padding:20px}
  .card{background:rgba(33,36,68,.6);border:1px solid rgba(255,255,255,.08);
        border-radius:16px;padding:32px;max-width:400px;width:100%}
  h1{font-size:1.4em;margin-bottom:8px}
  .sub{color:#94a3b8;font-size:.85em;margin-bottom:24px}
  label{display:block;color:#94a3b8;font-size:.75em;text-transform:uppercase;
        letter-spacing:.08em;margin-bottom:4px;margin-top:16px}
  input{width:100%;padding:10px 12px;border-radius:8px;border:1px solid rgba(255,255,255,.12);
        background:rgba(255,255,255,.06);color:#f7f7f2;font-size:1em;outline:none}
  input:focus{border-color:#38bdf8}
  button{width:100%;margin-top:24px;padding:12px;border:none;border-radius:10px;
         background:#38bdf8;color:#0a0e1a;font-weight:700;font-size:1em;cursor:pointer}
  button:active{background:#0ea5e9}
  .status{margin-top:16px;padding:12px;border-radius:8px;font-size:.9em;display:none}
  .ok{background:rgba(74,222,128,.15);color:#4ade80;display:block}
  .err{background:rgba(239,68,68,.15);color:#ef4444;display:block}
  .saved{margin-top:20px;padding-top:16px;border-top:1px solid rgba(255,255,255,.08)}
  .saved h3{font-size:.85em;color:#94a3b8;margin-bottom:8px}
  .saved ul{list-style:none;padding:0}
  .saved li{font-size:.85em;color:#f7f7f2;padding:4px 0}
  .saved li::before{content:"\\2022 ";color:#38bdf8}
</style>
</head>
<body>
<div class="card">
  <h1>SatNOGS WiFi Setup</h1>
  <p class="sub">Connect this station to your phone hotspot or local WiFi.</p>
  <form id="wf" onsubmit="return doConnect()">
    <label for="ssid">WiFi Network (SSID)</label>
    <input id="ssid" name="ssid" required placeholder="e.g. My iPhone">
    <label for="psk">Password</label>
    <input id="psk" name="password" type="password" required placeholder="WiFi password">
    <button type="submit" id="btn">Connect</button>
  </form>
  <div class="status" id="status"></div>
  <div class="saved" id="saved-div" style="display:none">
    <h3>Saved Networks</h3>
    <ul id="saved-list"></ul>
  </div>
</div>
<script>
function doConnect(){
  var btn=document.getElementById('btn');
  var st=document.getElementById('status');
  btn.disabled=true; btn.textContent='Connecting...';
  st.className='status'; st.style.display='none';
  fetch('/api/connect',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:document.getElementById('ssid').value,
                         password:document.getElementById('psk').value})
  }).then(r=>r.json()).then(d=>{
    if(d.ok){
      st.className='status ok';
      st.innerHTML='Connected! Station IP: <b>'+d.ip+'</b><br>'+
        'Now turn on your hotspot, connect to it, and open:<br>'+
        '<b>https://'+d.ip+':5000</b>';
    } else {
      st.className='status err';
      st.textContent='Failed: '+d.error;
      btn.disabled=false; btn.textContent='Connect';
    }
    st.style.display='block';
  }).catch(e=>{
    st.className='status err';
    st.textContent='Error: '+e;
    st.style.display='block';
    btn.disabled=false; btn.textContent='Connect';
  });
  return false;
}
fetch('/api/networks').then(r=>r.json()).then(d=>{
  if(d.networks&&d.networks.length){
    var ul=document.getElementById('saved-list');
    d.networks.forEach(n=>{var li=document.createElement('li');li.textContent=n;ul.appendChild(li);});
    document.getElementById('saved-div').style.display='block';
  }
});
</script>
</body>
</html>"""


class ProvisionHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/api/networks":
            networks = get_saved_networks()
            body = json.dumps({"networks": networks}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            # Serve portal page for any path (captive portal behavior)
            body = PORTAL_HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    def do_POST(self):
        global _provisioned
        if self.path == "/api/connect":
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length))
            ssid = data.get("ssid", "").strip()
            password = data.get("password", "").strip()

            if not ssid or not password:
                body = json.dumps({"ok": False, "error": "SSID and password required"}).encode()
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            ok, result = add_wifi_network(ssid, password)
            body = json.dumps({"ok": ok, "ip": result if ok else None,
                               "error": None if ok else result}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

            if ok:
                _provisioned = True
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


def run_portal():
    """Run the captive portal HTTP server."""
    server = HTTPServer(("0.0.0.0", PROVISION_PORT), ProvisionHandler)
    print(f"Captive portal running at http://{AP_IP}")
    server.serve_forever()


def main():
    global _provisioned

    # If already connected to WiFi, exit immediately
    if wait_for_wifi(timeout=WIFI_WAIT_TIMEOUT):
        print("Already connected to WiFi — provisioning not needed.")
        sys.exit(0)

    # No WiFi — start AP and portal
    try:
        start_ap()
    except Exception as e:
        print(f"Failed to start AP: {e}")
        sys.exit(1)

    # Run portal in background thread
    portal_thread = threading.Thread(target=run_portal, daemon=True)
    portal_thread.start()

    print(f"Connect your phone to '{AP_SSID}' and open any webpage.")
    print("Waiting for provisioning...")

    # Wait until provisioned or killed
    def on_signal(sig, frame):
        stop_ap()
        sys.exit(0)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    while not _provisioned:
        time.sleep(1)

    print("Provisioning complete! WiFi connected.")
    # Give the response time to reach the client
    time.sleep(3)
    stop_ap()


if __name__ == "__main__":
    main()
