#!/usr/bin/env python3
"""
WiFi Provisioning — AP-based captive portal for field WiFi setup.

On boot, if no known WiFi is available, starts a temporary access point
("SatNOGS-Setup") with a simple web page where the user enters their
phone hotspot SSID + password. Once connected, the AP shuts down.

Workflow:
    1. Pi boots → systemd runs wifi_provision.py
    2. No known WiFi? → start AP "SatNOGS-Setup" (open, no password)
    3. User connects phone → captive portal page with WiFi scan list
    4. User enters hotspot SSID + password → Pi saves & connects
    5. AP goes down, Pi is on the hotspot with internet
    6. Next boot: Pi finds saved WiFi → skips provisioning entirely

Buzzer feedback (via RF HAT Pico on /dev/serial0):
    - AP started: READY beep
    - Credentials saved + connecting: AOS beep
    - WiFi connected: WIFI_OK jingle
    - Connection failed: ERROR tone → AP restarts for retry

Saved networks persist in NetworkManager — survives reboots.
"""

import os
import sys
import json
import time
import signal
import subprocess
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler, ThreadingHTTPServer

AP_SSID = "SatNOGS (pw:satnogs123)"
AP_IFACE = "wlan0"
AP_IP = "192.168.4.1"
AP_CON_NAME = "satnogs-ap"
PROVISION_PORT = 80
DNSMASQ_SHARED_CONF = "/etc/NetworkManager/dnsmasq-shared.d/captive.conf"

# How long to wait for WiFi before starting AP (seconds)
WIFI_WAIT_TIMEOUT = 20

# How long to wait for STA connection before falling back to AP
STA_CONNECT_TIMEOUT = 20  # nmcli --wait seconds
STA_FALLBACK_DELAY = 5    # seconds before restarting AP on failure

# Track if provisioning is done
_provisioned = False


def run(cmd, check=False):
    """Run a shell command, return (returncode, stdout)."""
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if check and r.returncode != 0:
        raise RuntimeError(f"Command failed: {cmd}\n{r.stderr}")
    return r.returncode, r.stdout.strip()


def _try_buzzer(pattern):
    """Send a single buzzer command to RF HAT. Best-effort, never blocks long."""
    try:
        from buzzer_util import beep
        beep(pattern)
    except Exception:
        pass


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


def scan_wifi_networks():
    """Scan for visible WiFi networks. Returns list of {ssid, signal, security}."""
    # Trigger a fresh scan (may take 2-3s)
    run("sudo nmcli dev wifi rescan 2>/dev/null")
    time.sleep(2)
    rc, out = run("nmcli -t -f SSID,SIGNAL,SECURITY dev wifi list")
    seen = set()
    networks = []
    for line in out.splitlines():
        parts = line.split(":")
        if len(parts) >= 3:
            ssid = parts[0].strip()
            if not ssid or ssid in seen:
                continue
            seen.add(ssid)
            try:
                sig = int(parts[1])
            except ValueError:
                sig = 0
            security = parts[2].strip()
            networks.append({"ssid": ssid, "signal": sig, "security": security})
    # Sort by signal strength descending
    networks.sort(key=lambda n: n["signal"], reverse=True)
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


def start_captive_redirect():
    """Configure NM's dnsmasq to redirect all DNS to AP IP (wildcard).
    This is the cleanest approach — no separate dnsmasq, no port conflicts.
    NM's shared mode reads drop-in configs from dnsmasq-shared.d/."""
    os.makedirs(os.path.dirname(DNSMASQ_SHARED_CONF), exist_ok=True)
    with open(DNSMASQ_SHARED_CONF, "w") as f:
        f.write(f"address=/#/{AP_IP}\n")
    print("DNS wildcard captive redirect configured")


def stop_captive_redirect():
    """Remove DNS wildcard config."""
    try:
        os.remove(DNSMASQ_SHARED_CONF)
    except FileNotFoundError:
        pass
    print("DNS wildcard captive redirect removed")


def start_ap():
    """Start the provisioning access point + captive portal DNS."""
    print(f"Starting AP: {AP_SSID}")

    # Remove old AP connection if it exists
    run(f"sudo nmcli con delete {AP_CON_NAME} 2>/dev/null")

    # Write DNS wildcard BEFORE starting AP (dnsmasq reads config on startup)
    start_captive_redirect()

    # Create AP connection — WPA2 with simple password (iOS rejects open APs)
    run(f"sudo nmcli con add con-name {AP_CON_NAME} "
        f"ifname {AP_IFACE} type wifi ssid '{AP_SSID}' "
        f"802-11-wireless.mode ap 802-11-wireless.band bg "
        f"802-11-wireless.channel 6 "
        f"wifi-sec.key-mgmt wpa-psk wifi-sec.psk 'satnogs123' "
        f"ipv4.method shared ipv4.addresses {AP_IP}/24 "
        f"connection.autoconnect no", check=True)

    # Start it (NM's dnsmasq picks up the wildcard config we wrote above)
    run(f"sudo nmcli con up {AP_CON_NAME}", check=True)
    print(f"AP started: {AP_SSID} at {AP_IP}")

    # Buzzer: AP ready
    _try_buzzer(0x01)  # READY


def stop_ap():
    """Stop and remove the provisioning AP + redirect rules."""
    print("Stopping AP...")
    stop_captive_redirect()
    run(f"sudo nmcli con down {AP_CON_NAME} 2>/dev/null")
    run(f"sudo nmcli con delete {AP_CON_NAME} 2>/dev/null")


def add_wifi_network(ssid, password):
    """Add a WiFi network to NetworkManager. Returns immediately with status.
    Actual connection happens in background thread after HTTP response is sent."""
    print(f"Adding WiFi network: {ssid}")

    # Delete existing connection with same name (update password)
    subprocess.run(["sudo", "nmcli", "con", "delete", ssid], capture_output=True)

    # Use 'nmcli dev wifi connect' which auto-negotiates WPA2/WPA3
    # This creates a connection profile AND attempts to connect in one step.
    # We just save the profile here — actual connection happens in _do_handoff()
    # after the AP is torn down.
    # Try WPA-PSK first (most compatible), fall back to SAE (WPA3) if needed.
    for key_mgmt in ["wpa-psk", "sae"]:
        subprocess.run(["sudo", "nmcli", "con", "delete", ssid], capture_output=True)
        rc = subprocess.run([
            "sudo", "nmcli", "con", "add",
            "con-name", ssid, "ifname", AP_IFACE,
            "type", "wifi", "ssid", ssid,
            "wifi-sec.key-mgmt", key_mgmt, "wifi-sec.psk", password,
            "connection.autoconnect", "yes",
            "connection.autoconnect-priority", "10",
            "connection.autoconnect-retries", "0",
        ], capture_output=True, text=True)
        if rc.returncode == 0:
            print(f"Network added with {key_mgmt}")
            return True, ssid

    _try_buzzer(0x05)  # ERROR
    return False, f"Failed to add network"


def _do_handoff(ssid):
    """Background thread: tear down AP and connect STA. Called after HTTP response sent."""
    print(f"Handoff: AP down, connecting to {ssid}")
    _try_buzzer(0x02)  # AOS beep

    # Tear down AP (frees wlan0 for STA)
    stop_ap()
    time.sleep(1)

    # Connect with hard timeout
    rc = subprocess.run(
        ["sudo", "nmcli", "--wait", str(STA_CONNECT_TIMEOUT), "con", "up", ssid],
        capture_output=True, text=True
    )

    global _provisioned
    if rc.returncode == 0 and is_wifi_connected():
        _, ip_out = run("hostname -I")
        ip = ip_out.split()[0] if ip_out else "unknown"
        print(f"Connected to {ssid} at {ip}")
        _try_buzzer(0x06)  # WIFI_OK
        _provisioned = True  # NOW it's safe to signal main thread
        return

    # Connection failed — delete bad credentials and restart AP for retry
    print(f"Failed to connect to {ssid} — restarting AP in {STA_FALLBACK_DELAY}s")
    subprocess.run(["sudo", "nmcli", "con", "delete", ssid], capture_output=True)
    _try_buzzer(0x05)  # ERROR
    time.sleep(STA_FALLBACK_DELAY)
    try:
        start_ap()
        print("AP restarted for retry.")
    except Exception as e:
        print(f"Failed to restart AP: {e}")


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
        border-radius:16px;padding:32px;max-width:420px;width:100%}
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
  .btn-sm{width:auto;margin-top:8px;padding:8px 16px;font-size:.85em;border-radius:8px}
  .status{margin-top:16px;padding:12px;border-radius:8px;font-size:.9em;display:none}
  .ok{background:rgba(74,222,128,.15);color:#4ade80;display:block}
  .err{background:rgba(239,68,68,.15);color:#ef4444;display:block}
  .scan{margin-top:16px}
  .scan h3{font-size:.85em;color:#94a3b8;margin-bottom:8px;display:flex;
           align-items:center;justify-content:space-between}
  .net-list{list-style:none;padding:0;max-height:200px;overflow-y:auto}
  .net-list li{font-size:.85em;color:#f7f7f2;padding:8px 10px;border-radius:6px;
               cursor:pointer;display:flex;align-items:center;justify-content:space-between;
               border:1px solid transparent}
  .net-list li:hover{background:rgba(56,189,248,.1);border-color:rgba(56,189,248,.3)}
  .net-name{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
  .net-meta{font-size:.75em;color:#64748b;margin-left:8px;white-space:nowrap}
  .sig-bars{display:inline-block;width:16px;height:12px;position:relative;vertical-align:middle}
  .sig-bars span{display:inline-block;width:2px;margin-right:1px;background:#64748b;
                 vertical-align:bottom;border-radius:1px}
  .sig-bars.s1 span:nth-child(1){background:#4ade80}
  .sig-bars.s2 span:nth-child(1),.sig-bars.s2 span:nth-child(2){background:#4ade80}
  .sig-bars.s3 span:nth-child(1),.sig-bars.s3 span:nth-child(2),
  .sig-bars.s3 span:nth-child(3){background:#4ade80}
  .sig-bars.s4 span{background:#4ade80}
  .lock{font-size:.7em;margin-left:4px}
  .info{margin-top:20px;padding:12px;border-radius:8px;background:rgba(56,189,248,.08);
        border:1px solid rgba(56,189,248,.15);font-size:.8em;color:#94a3b8;line-height:1.5}
  .info strong{color:#f7f7f2}
  .saved{margin-top:20px;padding-top:16px;border-top:1px solid rgba(255,255,255,.08)}
  .saved h3{font-size:.85em;color:#94a3b8;margin-bottom:8px}
  .saved ul{list-style:none;padding:0}
  .saved li{font-size:.85em;color:#f7f7f2;padding:4px 0}
  .saved li::before{content:"\\2022 ";color:#38bdf8}
  .scanning{color:#94a3b8;font-size:.8em;padding:8px 0}
</style>
</head>
<body>
<div class="card">
  <h1>SatNOGS WiFi Setup</h1>
  <p class="sub">Connect this station to your phone hotspot or local WiFi.</p>

  <div class="scan" id="scan-div">
    <h3>Nearby Networks <button class="btn-sm" onclick="doScan()" id="scan-btn">Scan</button></h3>
    <div id="scan-status" class="scanning">Scanning...</div>
    <ul class="net-list" id="net-list" style="display:none"></ul>
  </div>

  <form id="wf" onsubmit="return doConnect()">
    <label for="ssid">WiFi Network (SSID)</label>
    <input id="ssid" name="ssid" required placeholder="Select above or type manually">
    <label for="psk">Password</label>
    <input id="psk" name="password" type="password" required placeholder="WiFi password">
    <button type="submit" id="btn">Connect</button>
  </form>
  <div class="status" id="status"></div>

  <div class="info">
    <strong>Phone hotspot tip:</strong><br>
    After connecting, enable your phone's hotspot. The station will auto-connect on every boot.<br><br>
    <strong>iPhone:</strong> Settings &rarr; Personal Hotspot &rarr; Allow Others to Join<br>
    &nbsp;&nbsp;Enable <em>Maximize Compatibility</em> (forces WPA2, required for Pi)<br>
    &nbsp;&nbsp;Rename iPhone to avoid special characters (Settings &rarr; General &rarr; About &rarr; Name)<br>
    <strong>Android:</strong> Settings &rarr; Hotspot &amp; tethering &rarr; Wi-Fi hotspot
  </div>

  <div class="saved" id="saved-div" style="display:none">
    <h3>Saved Networks</h3>
    <ul id="saved-list"></ul>
  </div>
</div>
<script>
function sigClass(s){if(s>=75)return 's4';if(s>=50)return 's3';if(s>=25)return 's2';return 's1';}
function doScan(){
  var ss=document.getElementById('scan-status');
  var nl=document.getElementById('net-list');
  var sb=document.getElementById('scan-btn');
  ss.textContent='Scanning...'; ss.style.display='block'; nl.style.display='none';
  sb.disabled=true;
  fetch('/api/scan').then(r=>r.json()).then(d=>{
    ss.style.display='none'; nl.innerHTML=''; nl.style.display='block';
    sb.disabled=false;
    if(!d.networks||!d.networks.length){ss.textContent='No networks found.';ss.style.display='block';return;}
    d.networks.forEach(n=>{
      var li=document.createElement('li');
      li.innerHTML='<span class="net-name">'+n.ssid+'</span>'+
        '<span class="net-meta">'+
        '<span class="sig-bars '+sigClass(n.signal)+'">'+
        '<span style="height:3px"></span><span style="height:6px"></span>'+
        '<span style="height:9px"></span><span style="height:12px"></span></span>'+
        (n.security?' <span class="lock">&#128274;</span>':'')+
        ' '+n.signal+'%</span>';
      li.onclick=function(){document.getElementById('ssid').value=n.ssid;
        document.getElementById('psk').focus();};
      nl.appendChild(li);
    });
  }).catch(e=>{ss.textContent='Scan failed: '+e;ss.style.display='block';sb.disabled=false;});
}
function doConnect(){
  var btn=document.getElementById('btn');
  var st=document.getElementById('status');
  btn.disabled=true; btn.textContent='Connecting...';
  st.className='status'; st.style.display='none';
  var ssid=document.getElementById('ssid').value;
  fetch('/api/connect',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:ssid,password:document.getElementById('psk').value})
  }).then(r=>r.json()).then(d=>{
    if(d.ok){
      st.className='status ok';
      st.innerHTML='<strong>Credentials saved!</strong><br><br>'+
        'Connecting to <strong>'+ssid+'</strong>...<br>'+
        'This network will disconnect in a few seconds.<br><br>'+
        '1. Reconnect your phone to <strong>'+ssid+'</strong><br>'+
        '2. Open <strong>https://satnogs.local:5000</strong>';
      btn.textContent='Connecting...';
    } else {
      st.className='status err';
      st.textContent='Failed: '+d.error+'. AP will restart for retry.';
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
// Auto-scan on load
doScan();
// Load saved networks
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


# Captive portal detection paths — redirect these so OS shows portal popup
CAPTIVE_PORTAL_PATHS = {
    "/generate_204",                # Android
    "/gen_204",                     # Android alt
    "/connecttest.txt",             # Windows
    "/ncsi.txt",                    # Windows alt
    "/hotspot-detect.html",         # iOS / macOS
    "/library/test/success.html",   # iOS alt
    "/canonical.html",              # Firefox
    "/success.txt",                 # Firefox alt
}


class ProvisionHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?")[0]

        if path in CAPTIVE_PORTAL_PATHS:
            # 302 redirect triggers iOS CNA popup + Android notification reliably
            self.send_response(302)
            self.send_header("Location", f"http://{AP_IP}/")
            self.send_header("Content-Length", "0")
            self.end_headers()

        elif path == "/api/networks":
            networks = get_saved_networks()
            body = json.dumps({"networks": networks}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif path == "/api/scan":
            networks = scan_wifi_networks()
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
            if ok:
                # Credentials saved — tell user to reconnect, then handoff
                body = json.dumps({
                    "ok": True,
                    "ssid": ssid,
                    "message": f"Credentials saved. Connecting to {ssid}... "
                               f"Reconnect your phone to {ssid} and open "
                               f"https://satnogs.local:5000"
                }).encode()
            else:
                body = json.dumps({"ok": False, "error": result}).encode()

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            self.wfile.flush()

            if ok:
                # Don't set _provisioned yet — wait for _do_handoff to confirm connection
                # Wait 2s for response to reach phone, then handoff in background
                threading.Thread(target=lambda: (time.sleep(2), _do_handoff(ssid)),
                                 daemon=True).start()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


def run_portal():
    """Run the captive portal HTTP server."""
    server = ThreadingHTTPServer(("0.0.0.0", PROVISION_PORT), ProvisionHandler)
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
