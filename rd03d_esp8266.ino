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
    http://radar.local/            live radar scope (or use the IP printed on Serial1)
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
<title>RD-03D radar</title>
<style>
:root{--bg:#fff;--fg:#1f1f1e;--mut:#6b6a65;--line:#d3d1c7;--card:#f6f5f1}
@media(prefers-color-scheme:dark){:root{--bg:#1c1c1b;--fg:#ecebe6;--mut:#9c9a92;--line:#444441;--card:#262624}}
body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
h1{font-size:20px;font-weight:500;margin:0 0 4px}
#st{color:var(--mut);margin-bottom:12px}
canvas{width:100%;max-width:720px;background:var(--card);border:1px solid var(--line);border-radius:12px;display:block}
table{border-collapse:collapse;margin-top:12px;width:100%;max-width:720px}
th,td{padding:6px 10px;border-bottom:1px solid var(--line);text-align:right}
th:first-child,td:first-child{text-align:left}
th{color:var(--mut);font-weight:500}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}
</style></head><body>
<h1>RD-03D radar</h1>
<div id="st">connecting...</div>
<canvas id="c" width="720" height="440"></canvas>
<table><thead><tr><th>Target</th><th>X (m)</th><th>Y (m)</th><th>Dist (m)</th><th>Angle</th><th>Speed (cm/s)</th></tr></thead><tbody id="tb"></tbody></table>
<script>
const c=document.getElementById('c'),g=c.getContext('2d');
const W=c.width,H=c.height,CX=W/2,CY=H-20,RANGE=8000,S=(H-40)/RANGE;
const COL=['#378ADD','#1D9E75','#D85A30'];
const trails={};
const css=v=>getComputedStyle(document.documentElement).getPropertyValue(v);
function grid(){
  g.clearRect(0,0,W,H);g.strokeStyle=css('--line');g.fillStyle=css('--mut');g.font='12px system-ui';g.lineWidth=1;
  for(let r=2000;r<=RANGE;r+=2000){g.beginPath();g.arc(CX,CY,r*S,Math.PI*7/6,Math.PI*11/6);g.stroke();g.fillText((r/1000)+' m',CX+6,CY-r*S+14);}
  for(const a of [-60,0,60]){const t=a*Math.PI/180;g.beginPath();g.moveTo(CX,CY);g.lineTo(CX+Math.sin(t)*RANGE*S,CY-Math.cos(t)*RANGE*S);g.stroke();}
  g.fillStyle=css('--fg');g.fillRect(CX-14,CY,28,6);
}
function draw(d){
  grid();
  const ids=d.targets.map(t=>t.id);
  for(const k in trails) if(!ids.includes(+k)) delete trails[k];
  for(const t of d.targets){
    const p=[CX+t.x*S,CY-t.y*S];
    (trails[t.id]=trails[t.id]||[]).push(p);
    if(trails[t.id].length>20) trails[t.id].shift();
    const col=COL[t.id-1];
    trails[t.id].forEach((q,i,a)=>{g.globalAlpha=(i+1)/a.length*0.5;g.fillStyle=col;g.beginPath();g.arc(q[0],q[1],3,0,7);g.fill();});
    g.globalAlpha=1;g.fillStyle=col;g.beginPath();g.arc(p[0],p[1],8,0,7);g.fill();
    g.fillStyle=css('--fg');g.font='13px system-ui';g.fillText('T'+t.id,p[0]+12,p[1]+4);
  }
  document.getElementById('tb').innerHTML=d.targets.map(t=>
    '<tr><td><span class="dot" style="background:'+COL[t.id-1]+'"></span>T'+t.id+'</td><td>'+(t.x/1000).toFixed(2)+'</td><td>'+(t.y/1000).toFixed(2)+'</td><td>'+(t.dist/1000).toFixed(2)+'</td><td>'+t.angle.toFixed(0)+'&deg;</td><td>'+t.speed+'</td></tr>').join('')
    ||'<tr><td colspan="6" style="text-align:center;color:var(--mut)">no moving targets</td></tr>';
}
async function poll(){
  try{
    const d=await (await fetch('/api/targets')).json();
    const st=document.getElementById('st');
    st.textContent=d.frames===0?'no frames from radar yet - check wiring/baud':
      (d.age_ms>1000?'radar stalled ('+d.age_ms+' ms since last frame)':d.targets.length+' target(s) | '+d.frames+' frames');
    draw(d);
  }catch(e){document.getElementById('st').textContent='ESP8266 not reachable';}
  setTimeout(poll,150);
}
grid();poll();
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
