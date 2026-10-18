#!/usr/bin/env python3
"""RD-03D radar on a CH341 (UART mode) -> web server with live 3D view + JSON API.

Hardware:
  Put the CH341 board in UART/TTL mode (it then shows up as 1a86:7523 and /dev/ttyUSB0).
  Set its logic-voltage jumper to 3.3 V.
    RD-03D VCC -> 5 V      RD-03D GND -> GND
    RD-03D TX  -> CH341 RXD
    RD-03D RX  -> CH341 TXD

Setup (Ubuntu):
  sudo apt remove brltty              # it can grab CH341 serial ports
  sudo usermod -aG dialout $USER      # then log out and back in
  pip install pyserial

Run:
  python3 rd03d_ch341.py                       # auto-detects /dev/ttyUSB* or /dev/ttyACM*
  python3 rd03d_ch341.py --serial /dev/ttyUSB0 --web-port 8000
  python3 rd03d_ch341.py --raw                 # print raw bytes to debug baud/wiring

Open http://localhost:8000/   (JSON: /api/targets, CORS enabled)
"""
import argparse
import glob
import json
import math
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

HDR = bytes([0xAA, 0xFF, 0x03, 0x00])   # frame: HDR | 3 x 8 bytes | 55 CC  (30 bytes)
START = time.monotonic()
lock = threading.Lock()
state = {"targets": [], "frames": 0, "last": None}


def decode_signed(raw):
    """Radar encodes signed values as: bit15 set = positive, low 15 bits = magnitude."""
    v = raw & 0x7FFF
    return v if raw & 0x8000 else -v


def parse(frame):
    targets = []
    for i in range(3):
        xr, yr, sr, rr = struct.unpack_from("<HHHH", frame, 4 + i * 8)
        targets.append({
            "active": (xr | yr | sr | rr) != 0,
            "x": decode_signed(xr),      # mm, + = right of sensor
            "y": decode_signed(yr),      # mm, forward
            "speed": decode_signed(sr),  # cm/s (verify against datasheet)
            "res": rr,                   # distance resolution, mm
        })
    with lock:
        state["targets"] = targets
        state["frames"] += 1
        state["last"] = time.monotonic()


def feed(buf):
    while True:
        i = buf.find(HDR)
        if i < 0:
            del buf[:-3]                 # keep a possible partial header
            return
        if i > 0:
            del buf[:i]
        if len(buf) < 30:
            return
        if buf[28] == 0x55 and buf[29] == 0xCC:
            parse(bytes(buf[:30]))
            del buf[:30]
        else:
            del buf[:1]


def find_port():
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No serial port found. Is the CH341 in UART mode (lsusb shows 1a86:7523)? "
                 "Try: sudo apt remove brltty, then replug.")
    return ports[0]


def reader(port, baud):
    while True:
        try:
            with serial.Serial(port, baud, timeout=0.1) as ser:
                print(f"Reading {port} @ {baud} baud")
                buf = bytearray()
                while True:
                    data = ser.read(256)
                    if data:
                        buf += data
                        feed(buf)
        except (serial.SerialException, OSError) as e:
            print(f"Serial error: {e}. Retrying in 2 s...")
            time.sleep(2)


def snapshot():
    with lock:
        last = state["last"]
        age = -1 if last is None else int((time.monotonic() - last) * 1000)
        if last is not None and age > 600:
            targets = []                 # radar stopped sending: show nothing
        else:
            targets = state["targets"]
        out = []
        for i, t in enumerate(targets, 1):
            if not t["active"]:
                continue
            x, y = t["x"], t["y"]
            out.append({
                "id": i, "x": x, "y": y, "speed": t["speed"],
                "dist": round(math.hypot(x, y)),
                "angle": round(math.degrees(math.atan2(x, y)), 1),
                "res": t["res"],
            })
        return {
            "uptime_ms": int((time.monotonic() - START) * 1000),
            "frames": state["frames"],
            "age_ms": age,
            "targets": out,
        }


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            body, ctype = PAGE.encode("utf-8"), "text/html; charset=utf-8"
        elif path == "/api/targets":
            body, ctype = json.dumps(snapshot()).encode("utf-8"), "application/json"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass


def raw_dump(port, baud):
    with serial.Serial(port, baud, timeout=1) as ser:
        data = ser.read(90)
    print(f"{len(data)} bytes from {port} @ {baud}:")
    print(data.hex(" "))
    print("Expect frames starting with: aa ff 03 00 ... 55 cc")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", help="serial device (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=256000)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--web-port", type=int, default=8000)
    ap.add_argument("--raw", action="store_true", help="print raw bytes and exit")
    args = ap.parse_args()

    port = args.serial or find_port()
    if args.raw:
        raw_dump(port, args.baud)
        return

    threading.Thread(target=reader, args=(port, args.baud), daemon=True).start()
    srv = ThreadingHTTPServer((args.host, args.web_port), Handler)
    print(f"Web UI: http://localhost:{args.web_port}/   API: /api/targets   (Ctrl+C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RD-03D radar 3D</title>
<style>
:root{--bg:#fff;--fg:#1f1f1e;--mut:#6b6a65;--line:#d3d1c7;--card:#f6f5f1;--fan:rgba(55,138,221,.10);--edge:rgba(55,138,221,.7)}
@media(prefers-color-scheme:dark){:root{--bg:#1c1c1b;--fg:#ecebe6;--mut:#9c9a92;--line:#444441;--card:#262624;--fan:rgba(55,138,221,.16);--edge:rgba(133,183,235,.8)}}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
h1{font-size:20px;font-weight:500;margin:0 0 4px}
#st{color:var(--mut);margin-bottom:12px}
canvas{width:100%;max-width:760px;height:auto;background:var(--card);border:1px solid var(--line);border-radius:12px;display:block;touch-action:none;cursor:grab}
.bar{display:flex;gap:8px;align-items:center;margin-top:8px;color:var(--mut);font-size:13px}
button{background:var(--card);color:var(--fg);border:1px solid var(--line);border-radius:8px;padding:4px 10px;font:inherit;cursor:pointer}
table{border-collapse:collapse;margin-top:12px;width:100%;max-width:760px}
th,td{padding:6px 10px;border-bottom:1px solid var(--line);text-align:right}
th:first-child,td:first-child{text-align:left}
th{color:var(--mut);font-weight:500}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
</style></head><body>
<h1>RD-03D radar (3D)</h1>
<div id="st">connecting...</div>
<canvas id="c" width="760" height="480"></canvas>
<div class="bar"><button id="rv">Reset view</button><button id="tv">Top view</button><span>drag to rotate, wheel to zoom</span></div>
<table><thead><tr><th>Target</th><th>X (m)</th><th>Y (m)</th><th>Dist (m)</th><th>Angle</th><th>Speed (cm/s)</th></tr></thead><tbody id="tb"></tbody></table>
<script>
const c=document.getElementById('c'),g=c.getContext('2d');
const W=c.width,H=c.height,COL=['#378ADD','#1D9E75','#D85A30'],RNG=8,PERSON=1.7;
let yaw=0,pitch=0.95,dist=11,FOV=560,K={};
const css=v=>getComputedStyle(document.documentElement).getPropertyValue(v).trim();
function P(x,y,z){
  y-=3.5;
  const cx=x*Math.cos(yaw)-y*Math.sin(yaw),cy=x*Math.sin(yaw)+y*Math.cos(yaw);
  const dy=cy*Math.cos(pitch)-z*Math.sin(pitch),up=cy*Math.sin(pitch)+z*Math.cos(pitch);
  const d=dist+dy,f=FOV/d;
  return [W/2+cx*f,H*0.62-up*f,d,f];
}
function poly(pts,fill,stroke,lw){
  g.beginPath();pts.forEach((p,i)=>i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1]));
  if(fill){g.fillStyle=fill;g.fill();}
  if(stroke){g.strokeStyle=stroke;g.lineWidth=lw||1;g.stroke();}
}
function arc(r){const a=[];for(let d=-60;d<=60;d+=4){const t=d*Math.PI/180;a.push(P(r*Math.sin(t),r*Math.cos(t),0));}return a;}
function floor(){
  const wedge=[P(0,0,0),...arc(RNG)];
  poly(wedge,K.fan,null);
  g.font='12px system-ui';
  for(let r=2;r<=RNG;r+=2){poly(arc(r),null,K.line,1);const l=P(0,r,0);g.fillStyle=K.mut;g.fillText(r+' m',l[0]+4,l[1]-3);}
  for(const d of [-60,-30,0,30,60]){const t=d*Math.PI/180;poly([P(0,0,0),P(RNG*Math.sin(t),RNG*Math.cos(t),0)],null,K.line,1);}
  poly(wedge,null,K.edge,1.5);
}
function sensor(){
  poly([P(0,0,0),P(0,0,0.4)],null,K.fg,2);
  poly([P(-.18,0,.4),P(.18,0,.4),P(.18,0,.62),P(-.18,0,.62)],K.fg,null);
  const l=P(0,0,0.62);g.fillStyle=K.mut;g.font='12px system-ui';g.fillText('RD-03D',l[0]+10,l[1]);
}
const T={};
function update(d){
  const now=performance.now();
  d.targets.forEach(t=>{
    let o=T[t.id];if(!o)o=T[t.id]={x:t.x/1000,y:t.y/1000,trail:[]};
    o.tx=t.x/1000;o.ty=t.y/1000;o.dist=t.dist;o.last=now;
    o.trail.push([o.tx,o.ty]);if(o.trail.length>30)o.trail.shift();
  });
}
function frame(){
  K={fan:css('--fan'),line:css('--line'),mut:css('--mut'),fg:css('--fg'),edge:css('--edge')};
  g.clearRect(0,0,W,H);
  floor();sensor();
  const now=performance.now(),list=[];
  for(const id in T){
    const o=T[id];
    if(now-o.last>600){delete T[id];continue;}
    o.x+=(o.tx-o.x)*0.25;o.y+=(o.ty-o.y)*0.25;
    const col=COL[id-1];
    o.trail.forEach((q,i,a)=>{const p=P(q[0],q[1],0);g.globalAlpha=(i+1)/a.length*0.5;g.fillStyle=col;g.beginPath();g.arc(p[0],p[1],p[3]*0.06,0,7);g.fill();});
    g.globalAlpha=1;
    list.push([id,o]);
  }
  list.sort((a,b)=>P(b[1].x,b[1].y,0)[2]-P(a[1].x,a[1].y,0)[2]);
  for(const [id,o] of list){
    const col=COL[id-1],gp=P(o.x,o.y,0),tp=P(o.x,o.y,PERSON);
    poly([P(0,0,0),gp],null,col+'88',1);
    const ring=[];for(let k=0;k<=24;k++){const t=k/24*2*Math.PI;ring.push(P(o.x+0.35*Math.cos(t),o.y+0.35*Math.sin(t),0));}
    poly(ring,col+'44',col,1.5);
    g.lineCap='round';g.strokeStyle=col;g.globalAlpha=0.85;g.lineWidth=gp[3]*0.5;
    g.beginPath();g.moveTo(gp[0],gp[1]);g.lineTo(tp[0],tp[1]);g.stroke();
    g.globalAlpha=1;g.lineCap='butt';
    g.fillStyle=K.fg;g.font='13px system-ui';g.fillText('T'+id+'  '+(o.dist/1000).toFixed(1)+' m',tp[0]+gp[3]*0.35,tp[1]);
  }
  requestAnimationFrame(frame);
}
let drag=null;
c.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];c.setPointerCapture(e.pointerId);c.style.cursor='grabbing';});
c.addEventListener('pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*0.008;pitch=Math.max(0.1,Math.min(1.55,pitch+(e.clientY-drag[1])*0.006));drag=[e.clientX,e.clientY];});
const end=()=>{drag=null;c.style.cursor='grab';};
c.addEventListener('pointerup',end);c.addEventListener('pointercancel',end);
c.addEventListener('wheel',e=>{e.preventDefault();dist=Math.max(5,Math.min(25,dist*(1+e.deltaY*0.001)));},{passive:false});
document.getElementById('rv').onclick=()=>{yaw=0;pitch=0.95;dist=11;};
document.getElementById('tv').onclick=()=>{yaw=0;pitch=1.55;dist=11;};
async function poll(){
  try{
    const d=await (await fetch('/api/targets')).json();
    document.getElementById('st').textContent=d.frames===0?'no frames from radar yet - check wiring/baud':
      (d.age_ms>1000?'radar stalled ('+d.age_ms+' ms since last frame)':d.targets.length+' target(s) | '+d.frames+' frames');
    update(d);
    document.getElementById('tb').innerHTML=d.targets.map(t=>
      '<tr><td><span class="dot" style="background:'+COL[t.id-1]+'"></span>T'+t.id+'</td><td>'+(t.x/1000).toFixed(2)+'</td><td>'+(t.y/1000).toFixed(2)+'</td><td>'+(t.dist/1000).toFixed(2)+'</td><td>'+t.angle.toFixed(0)+'&deg;</td><td>'+t.speed+'</td></tr>').join('')
      ||'<tr><td colspan="6" style="text-align:center;color:var(--mut)">no moving targets</td></tr>';
  }catch(e){document.getElementById('st').textContent='ESP8266 not reachable';}
  setTimeout(poll,150);
}
frame();poll();
</script></body></html>"""

if __name__ == "__main__":
    main()
