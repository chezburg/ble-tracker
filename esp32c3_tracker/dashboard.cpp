/*
 * webserver.cpp
 *
 * Serves the single-page dashboard over HTTP (port 80) and pushes
 * live JSON position updates via WebSocket (port 81).
 *
 * Libraries: ESPAsyncWebServer, AsyncTCP, ArduinoJson
 */

#include "dashboard.h"
#include "ble_scanner.h"
#include "positioning.h"
#include "altimeter.h"
#include "storage.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

extern TrackerConfig activeCfg;
Dashboard dashboard;

static WebServer httpServer(WEB_SERVER_PORT);

// ─── Embedded dashboard HTML ────────────────────────────────────
// Stored in flash (PROGMEM) to save RAM
static const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>BLE Tracker</title><style>
:root{--bg:#0f1117;--surface:#1a1d27;--border:#2a2d3e;--accent:#4f8ef7;--green:#22c55e;--yellow:#f59e0b;--red:#ef4444;--text:#e2e8f0;--muted:#64748b}
*{box-sizing:border-box;margin:0;padding:0}body{background:var(--bg);color:var(--text);font-family:system-ui,sans-serif;min-height:100vh}
header{padding:12px 20px;border-bottom:1px solid var(--border);display:flex;align-items:center;gap:12px}
header h1{font-size:1rem;font-weight:600}.dot{width:10px;height:10px;border-radius:50%;background:var(--red);transition:background .3s}
.dot.connected{background:var(--green)}.layout{display:grid;grid-template-columns:1fr 300px;gap:12px;padding:12px;height:calc(100vh - 50px)}
.card{background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:12px}
#mapCard{display:flex;flex-direction:column}#mapCanvas{flex:1;border-radius:6px;background:#0a0c14;cursor:crosshair}
.sidebar{display:flex;flex-direction:column;gap:10px;overflow-y:auto}
.card h2{font-size:.75rem;text-transform:uppercase;letter-spacing:.05em;color:var(--muted);margin-bottom:8px}
.stat-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px}.stat{background:var(--bg);border-radius:6px;padding:8px}
.stat label{font-size:.65rem;color:var(--muted);display:block}.stat value{font-size:1.1rem;font-weight:700;color:var(--accent)}
.base-row{display:flex;align-items:center;gap:8px;padding:6px 0;border-bottom:1px solid var(--border)}
.base-id{width:24px;height:24px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-weight:700;font-size:.8rem}
.b0{background:#4f8ef744;color:#4f8ef7}.b1{background:#f59e0b44;color:#f59e0b}.b2{background:#22c55e44;color:#22c55e}
.base-info .name{font-size:.75rem;font-weight:600}.base-info .vals{font-size:.7rem;color:var(--muted)}
.signal-bar{height:3px;border-radius:2px;background:var(--border);margin-top:4px}.signal-bar .fill{height:100%;border-radius:2px;transition:width .4s}
.floor-badge{display:inline-flex;align-items:center;gap:6px;background:var(--accent);color:#fff;padding:4px 12px;border-radius:15px;font-weight:700;font-size:1rem}
input,select{width:100%;background:var(--bg);border:1px solid var(--border);color:var(--text);padding:6px;border-radius:4px;font-size:.8rem;margin-bottom:6px}
button{width:100%;padding:8px;background:var(--accent);color:#fff;border:none;border-radius:4px;font-weight:600;cursor:pointer;margin-bottom:4px}
button:hover{opacity:.85}button.danger{background:var(--red)}.trail-toggle{display:flex;align-items:center;gap:6px;font-size:.75rem;cursor:pointer;margin-bottom:4px}
</style></head><body>
<header><div class="dot" id="wsDot"></div><h1>🛰 BLE Tracker Dashboard</h1></header>
<div class="layout">
<div class="card" id="mapCard"><h2>Map</h2><canvas id="mapCanvas"></canvas></div>
<div class="sidebar">
<div class="card"><h2>Position</h2><div class="stat-grid">
<div class="stat"><label>X(m)</label><value id="posX">—</value></div>
<div class="stat"><label>Y(m)</label><value id="posY">—</value></div>
<div class="stat"><label>Alt(m)</label><value id="altM">—</value></div>
<div class="stat"><label>Acc(m)</label><value id="acc">—</value></div>
</div><div style="text-align:center;margin-top:8px"><span class="floor-badge">Floor <span id="floorNum">?</span></span></div></div>
<div class="card"><h2>Bases</h2><div id="baseList"></div></div>
<div class="card"><h2>Config</h2>
<input type="number" id="cfgN" step="0.1" placeholder="n (Path Loss)"><button onclick="sendConfig()">Apply n</button>
<input type="text" id="cfgBase" placeholder="id,x,y"><button onclick="sendBaseCoord()">Set Base</button>
<input type="number" id="cfgFloor" placeholder="Floor #"><button onclick="regFloor()">Reg Floor</button>
<input type="number" id="cfgPing" placeholder="Ping (s)"><button onclick="sendPingInterval()">Set Ping</button>
<button class="danger" onclick="clearLog()">Clear Log</button></div>
<div class="card"><h2>View</h2><label class="trail-toggle"><input type="checkbox" id="showTrail" checked> Trail</label>
<label class="trail-toggle"><input type="checkbox" id="autoScale" checked> Auto-scale</label></div>
</div></div>
<script>
const TRAIL_MAX=200;let trail=[],bases=[{},{},{}],fix=null,autoScale=true,mapScale=50,mapOffX=0,mapOffY=0,pollTimer=null;
function connect(ms=500){resizeCanvas();if(pollTimer)clearInterval(pollTimer);pollTimer=setInterval(pollUpdate,ms)}
function pollUpdate(){fetch('/update').then(r=>r.json()).then(d=>{document.getElementById('wsDot').classList.add('connected');
if(d.position)handlePosition(d.position);if(d.bases)handleBases(d.bases);drawMap()}).catch(()=>document.getElementById('wsDot').classList.remove('connected'))}
function handlePosition(d){fix=d;document.getElementById('posX').textContent=d.x.toFixed(2);document.getElementById('posY').textContent=d.y.toFixed(2);
document.getElementById('altM').textContent=d.altM.toFixed(2);document.getElementById('acc').textContent=d.accuracy>=0?d.accuracy.toFixed(2):(d.stable?'—':'WAIT');
document.getElementById('floorNum').textContent=d.floor;if(d.valid&&document.getElementById('showTrail').checked){trail.push({x:d.x,y:d.y});if(trail.length>TRAIL_MAX)trail.shift()}}
function handleBases(b){bases=b;const l=document.getElementById('baseList');if(!l)return;l.innerHTML='';const c=['#4f8ef7','#f59e0b','#22c55e'];
b.forEach((s,i)=>{const q=s.valid?Math.max(0,Math.min(100,(s.rssi+100)*2)):0;
l.innerHTML+=`<div class="base-row"><div class="base-id b${i}">${i}</div><div class="base-info"><div class="name">Base ${i}${s.valid?'':' (stale)'}</div>
<div class="vals">${s.rssi??'—'}dBm | ${s.dist?s.dist.toFixed(2)+'m':'—'}</div><div class="signal-bar"><div class="fill" style="width:${q}%;background:${c[i]}"></div></div></div></div>`})}
function sendConfig(){const n=parseFloat(document.getElementById('cfgN').value);if(!isNaN(n))fetch('/config',{method:'POST',body:JSON.stringify({pathLossN:n})})}
function sendBaseCoord(){const p=document.getElementById('cfgBase').value.split(',');if(p.length>=3)fetch('/config',{method:'POST',body:JSON.stringify({base:{id:+p[0],x:+p[1],y:+p[2]}})})}
function regFloor(){const f=parseInt(document.getElementById('cfgFloor').value);fetch('/config',{method:'POST',body:JSON.stringify({registerFloor:f})})}
function sendPingInterval(){const s=parseInt(document.getElementById('cfgPing').value);if(!isNaN(s))fetch('/config',{method:'POST',body:JSON.stringify({pingIntervalS:s})}).then(r=>{if(r.ok)connect(s*1000)})}
function clearLog(){if(confirm('Clear?'))fetch('/clearlog',{method:'POST'}).then(()=>{trail=[];fix=null;drawMap()})}
const canvas=document.getElementById('mapCanvas'),ctx=canvas.getContext('2d');
function resizeCanvas(){canvas.width=canvas.offsetWidth;canvas.height=canvas.offsetHeight}
window.addEventListener('resize',()=>{resizeCanvas();drawMap()});
function worldToCanvas(x,y){return[canvas.width/2+(x-mapOffX)*mapScale,canvas.height/2-(y-mapOffY)*mapScale]}
function drawMap(){if(canvas.width===0)resizeCanvas();ctx.clearRect(0,0,canvas.width,canvas.height);ctx.strokeStyle='#1e2130';ctx.lineWidth=1;
for(let x=(canvas.width/2%mapScale);x<canvas.width;x+=mapScale){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,canvas.height);ctx.stroke()}
for(let y=(canvas.height/2%mapScale);y<canvas.height;y+=mapScale){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(canvas.width,y);ctx.stroke()}
if(document.getElementById('autoScale').checked&&bases.length){const xs=bases.filter(b=>b.x!=null).map(b=>b.x),ys=bases.filter(b=>b.y!=null).map(b=>b.y);
if(fix&&fix.valid){xs.push(fix.x);ys.push(fix.y)}if(xs.length>0){const minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
const r=Math.max(maxX-minX,maxY-minY,3)*1.4;mapScale=Math.min(canvas.width,canvas.height)/r;mapOffX=(minX+maxX)/2;mapOffY=(minY+maxY)/2}}
if(trail.length>1){ctx.beginPath();ctx.strokeStyle='rgba(79,142,247,0.35)';ctx.lineWidth=2;let[tx,ty]=worldToCanvas(trail[0].x,trail[0].y);ctx.moveTo(tx,ty);
for(let i=1;i<trail.length;i++){[tx,ty]=worldToCanvas(trail[i].x,trail[i].y);ctx.lineTo(tx,ty)}ctx.stroke()}
const bc=['#4f8ef7','#f59e0b','#22c55e'];bases.forEach((b,i)=>{if(b.x==null)return;const[cx,cy]=worldToCanvas(b.x,b.y);
if(b.dist&&b.valid){ctx.beginPath();ctx.arc(cx,cy,b.dist*mapScale,0,Math.PI*2);ctx.strokeStyle=bc[i]+'44';ctx.lineWidth=1.5;ctx.stroke()}
ctx.beginPath();ctx.arc(cx,cy,8,0,Math.PI*2);ctx.fillStyle=bc[i];ctx.fill();ctx.fillStyle='#fff';ctx.font='bold 10px sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(i,cx,cy)});
if(fix&&fix.valid){const[px,py]=worldToCanvas(fix.x,fix.y);if(fix.accuracy>0){ctx.beginPath();ctx.arc(px,py,fix.accuracy*mapScale,0,Math.PI*2);
ctx.fillStyle='rgba(79,142,247,0.08)';ctx.strokeStyle='rgba(79,142,247,0.3)';ctx.lineWidth=1;ctx.fill();ctx.stroke()}
ctx.beginPath();ctx.arc(px,py,10,0,Math.PI*2);ctx.fillStyle='#4f8ef7';ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke()}}
requestAnimationFrame(drawMap);fetch('/state').then(r=>r.json()).then(d=>{if(d.bases)handleBases(d.bases);
if(d.pingIntervalS){document.getElementById('cfgPing').value=d.pingIntervalS;connect(d.pingIntervalS*1000)}else connect(500)})}
</script></body></html>
)rawhtml";

// ────────────────────────────────────────────────────────────────
void Dashboard::begin(const char* ssid, const char* pass) {
  Serial.printf("[WIFI] Connecting to %s", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500); Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("FAIL");
    return;
  }

  setupRoutes();
  httpServer.begin();
  Serial.printf("[HTTP] Server on port %d\n", WEB_SERVER_PORT);
}

// ────────────────────────────────────────────────────────────────
void Dashboard::setupRoutes() {
  // Dashboard
  httpServer.on("/", HTTP_GET, []() {
    httpServer.send_P(200, "text/html", DASHBOARD_HTML);
  });

  // Snapshot of current state (for initial page load)
  httpServer.on("/state", HTTP_GET, []() {
    DynamicJsonDocument doc(1280); // Heap allocation to prevent stack overflow
    JsonArray jBases = doc.createNestedArray("bases");
    for (int i = 0; i < NUM_BASES; i++) {
      JsonObject jb = jBases.createNestedObject();
      const BaseReading& b = bleScanner.base(i);
      jb["id"]    = i;
      jb["valid"] = b.valid;
      jb["rssi"]  = b.rssiFiltered;
      jb["dist"]  = b.distanceM;
      jb["x"]     = positioning.getBaseCoord(i).x;
      jb["y"]     = positioning.getBaseCoord(i).y;
    }
    doc["pingIntervalS"] = activeCfg.pingIntervalS;
    String out; serializeJson(doc, out);
    httpServer.send(200, "application/json", out);
  });

  // /update endpoint for live dashboard polling
  httpServer.on("/update", HTTP_GET, []() {
    DynamicJsonDocument doc(2048);
    
    // Position
    const PositionFix& fix = positioning.latest();
    JsonObject pos = doc.createNestedObject("position");
    pos["x"]        = fix.valid ? fix.x        : 0;
    pos["y"]        = fix.valid ? fix.y        : 0;
    pos["altM"]     = altimeter.getAltitudeM();
    pos["floor"]    = altimeter.getFloor();
    pos["accuracy"] = fix.valid ? fix.accuracy : -1;
    pos["valid"]    = fix.valid;
    pos["stable"]   = positioning.isStable();

    // Bases
    JsonArray arr = doc.createNestedArray("bases");
    for (int i = 0; i < NUM_BASES; i++) {
      const BaseReading& b = bleScanner.base(i);
      JsonObject jb = arr.createNestedObject();
      jb["id"]    = i;
      jb["valid"] = b.valid;
      jb["rssi"]  = (int)b.rssiFiltered;
      jb["dist"]  = b.distanceM;
      jb["x"]     = positioning.getBaseCoord(i).x;
      jb["y"]     = positioning.getBaseCoord(i).y;
    }

    String out; serializeJson(doc, out);
    httpServer.send(200, "application/json", out);
  });

  // Clear log
  httpServer.on("/clearlog", HTTP_POST, []() {
    storage.clearLog();
    positioning.resetFix();
    httpServer.send(200, "text/plain", "OK");
  });

  // Config updates from dashboard
  httpServer.on("/config", HTTP_POST, []() {
    if (!httpServer.hasArg("plain")) {
      httpServer.send(400, "text/plain", "Body missing");
      return;
    }
    String data = httpServer.arg("plain");
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, data)) {
      httpServer.send(400, "text/plain", "JSON invalid");
      return;
    }

    // Update the live global config
    bool changed = false;

    if (doc.containsKey("pathLossN")) {
      activeCfg.pathLossN = doc["pathLossN"];
      bleScanner.setPathLossN(activeCfg.pathLossN);
      changed = true;
    }
    if (doc.containsKey("base")) {
      uint8_t id = doc["base"]["id"];
      activeCfg.baseCoords[id].x = doc["base"]["x"];
      activeCfg.baseCoords[id].y = doc["base"]["y"];
      positioning.setBaseCoord(id, activeCfg.baseCoords[id].x, activeCfg.baseCoords[id].y);
      changed = true;
    }
    if (doc.containsKey("registerFloor")) {
      uint8_t f = doc["registerFloor"];
      float h = altimeter.getAltitudeM();
      activeCfg.floorHeights[f] = h;
      activeCfg.floorCount = max((int)activeCfg.floorCount, (int)f + 1);
      positioning.setFloorHeight(f, h);
      positioning.setFloorCount(activeCfg.floorCount);
      changed = true;
    }
    if (doc.containsKey("pingIntervalS")) {
      activeCfg.pingIntervalS = doc["pingIntervalS"];
      positioning.clearAccumulator();
      changed = true;
    }

    if (changed) {
      storage.saveConfig(activeCfg);
    }
    httpServer.send(200, "text/plain", "OK");
  });
}

// ────────────────────────────────────────────────────────────────
void Dashboard::pushUpdate() {
  const PositionFix& fix = positioning.latest();
  if (!fix.valid) return;

  WiFiClient client;
  HTTPClient http;

  if (http.begin(client, DEFAULT_SERVER_URL)) {
    http.addHeader("Content-Type", "application/json");

    JsonDocument doc;
    doc["id"] = TRACKER_ID; // Include the unique Tracker ID
    doc["x"] = fix.x;
    doc["y"] = fix.y;
    doc["floor"] = altimeter.getFloor(); // Use calculated floor

    String json;
    serializeJson(doc, json);

    int code = http.POST(json);
    if (code > 0) {
      Serial.printf("POST:%d\n", code);
    } else {
      Serial.printf("ERR:%s\n", http.errorToString(code).c_str());
    }
    http.end();
  }
}

// ────────────────────────────────────────────────────────────────
void Dashboard::update() {
  httpServer.handleClient();

  // Push to central server at defined interval
  if (WiFi.status() == WL_CONNECTED && (millis() - _lastPush > (activeCfg.pingIntervalS * 1000))) {
    _lastPush = millis();
    pushUpdate();
  }
}
