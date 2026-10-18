/*
  RD-03D (24 GHz multi-target radar) -> ESP8266 -> web server

  Wiring (NodeMCU / D1 mini):
    RD-03D VCC -> VIN (5 V)
    RD-03D GND -> GND
    RD-03D TX  -> GPIO13 / D7  (ESP RX after Serial.swap())
    RD-03D RX  -> GPIO15 / D8  (ESP TX after Serial.swap())

  After Serial.swap() the USB serial is NOT available.
  Debug prints go out Serial1 (TX only, GPIO2 / D4) at 115200.

  Endpoints:
    http://radar.local/            live 3D radar view (or use the IP printed on Serial1)
    http://radar.local/api/targets JSON, CORS enabled (use from Streamlit, Python, etc.)

  Board: "NodeMCU 1.0 (ESP-12E)" or "LOLIN(WEMOS) D1 R2 & mini"
*/
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* HOSTNAME  = "radar";

const uint32_t RADAR_BAUD = 256000;

struct Target {
  bool active;
  int  x;        // mm, + = right of the sensor
  int  y;        // mm, forward
  int  speed;    // cm/s, sign = direction (verify against datasheet)
  unsigned res;  // distance resolution, mm
};

Target   targets[3];
uint8_t  buf[30];
uint8_t  idx = 0;
uint32_t lastFrameMs = 0;
uint32_t frames = 0;

ESP8266WebServer server(80);

// Radar encodes signed values as: bit15 set = positive, clear = negative, low 15 bits = magnitude
static int decodeSigned(uint16_t raw) {
  int v = raw & 0x7FFF;
  return (raw & 0x8000) ? v : -v;
}

void parseFrame() {
  for (int i = 0; i < 3; i++) {
    const uint8_t* p = buf + 4 + i * 8;
    uint16_t xr = p[0] | (p[1] << 8);
    uint16_t yr = p[2] | (p[3] << 8);
    uint16_t sr = p[4] | (p[5] << 8);
    uint16_t rr = p[6] | (p[7] << 8);
    targets[i].active = (xr | yr | sr | rr) != 0;
    targets[i].x = decodeSigned(xr);
    targets[i].y = decodeSigned(yr);
    targets[i].speed = decodeSigned(sr);
    targets[i].res = rr;
  }
  lastFrameMs = millis();
  frames++;
}

// Frame: AA FF 03 00 | 3 x 8 bytes | 55 CC  (30 bytes)
void feed(uint8_t b) {
  static const uint8_t HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
  if (idx < 4) {
    if (b == HDR[idx]) {
      buf[idx++] = b;
    } else if (b == HDR[0]) {
      buf[0] = b;
      idx = 1;
    } else {
      idx = 0;
    }
    return;
  }
  buf[idx++] = b;
  if (idx == 30) {
    if (buf[28] == 0x55 && buf[29] == 0xCC) parseFrame();
    idx = 0;
  }
}

void handleApi() {
  String j = "{\"uptime_ms\":" + String(millis());
  j += ",\"frames\":" + String(frames);
  j += ",\"age_ms\":" + (frames ? String(millis() - lastFrameMs) : String(-1));
  j += ",\"targets\":[";
  bool first = true;
  for (int i = 0; i < 3; i++) {
    if (!targets[i].active) continue;
    float x = targets[i].x, y = targets[i].y;
    if (!first) j += ",";
    first = false;
    j += "{\"id\":" + String(i + 1);
    j += ",\"x\":" + String(targets[i].x);
    j += ",\"y\":" + String(targets[i].y);
    j += ",\"speed\":" + String(targets[i].speed);
    j += ",\"dist\":" + String(sqrtf(x * x + y * y), 0);
    j += ",\"angle\":" + String(degrees(atan2f(x, y)), 1);
    j += ",\"res\":" + String(targets[i].res) + "}";
  }
  j += "]}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

const char PAGE[] PROGMEM = R"rawliteral(<!doctype html>
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
</script></body></html>)rawliteral";

void setup() {
  Serial1.begin(115200);              // debug on GPIO2 (D4)
  Serial.setRxBufferSize(512);
  Serial.begin(RADAR_BAUD);
  Serial.swap();                      // UART0 -> GPIO13 (RX) / GPIO15 (TX)

  WiFi.mode(WIFI_STA);
  WiFi.hostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial1.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial1.print(".");
  }
  Serial1.println();
  Serial1.print("IP: ");
  Serial1.println(WiFi.localIP());

  if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api/targets", handleApi);
  server.begin();
}

void loop() {
  while (Serial.available()) feed(Serial.read());
  server.handleClient();
  MDNS.update();
}
