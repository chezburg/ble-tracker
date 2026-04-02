/*
 * webserver.cpp
 *
 * Serves the single-page dashboard over HTTP (port 80) and pushes
 * live JSON position updates via WebSocket (port 81).
 *
 * Libraries: ESPAsyncWebServer, AsyncTCP, ArduinoJson
 */

#include "webserver.h"
#include "ble_scanner.h"
#include "positioning.h"
#include "altimeter.h"
#include "storage.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>

WebServer webServer;

static AsyncWebServer  httpServer(WEB_SERVER_PORT);
static AsyncWebSocket  ws("/ws");

// ─── Embedded dashboard HTML ────────────────────────────────────
// Stored in flash (PROGMEM) to save RAM
static const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BLE Tracker Dashboard</title>
<style>
  :root {
    --bg: #0f1117; --surface: #1a1d27; --border: #2a2d3e;
    --accent: #4f8ef7; --green: #22c55e; --yellow: #f59e0b;
    --red: #ef4444; --text: #e2e8f0; --muted: #64748b;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: 'Segoe UI', system-ui, sans-serif; min-height: 100vh; }
  header { padding: 16px 24px; border-bottom: 1px solid var(--border); display: flex; align-items: center; gap: 12px; }
  header h1 { font-size: 1.1rem; font-weight: 600; }
  .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--red); transition: background .3s; }
  .dot.connected { background: var(--green); }
  .layout { display: grid; grid-template-columns: 1fr 320px; gap: 16px; padding: 16px; height: calc(100vh - 57px); }
  .card { background: var(--surface); border: 1px solid var(--border); border-radius: 12px; padding: 16px; }
  #mapCard { display: flex; flex-direction: column; }
  #mapCard h2 { font-size: .85rem; text-transform: uppercase; letter-spacing: .08em; color: var(--muted); margin-bottom: 12px; }
  #mapCanvas { flex: 1; border-radius: 8px; background: #0a0c14; cursor: crosshair; }
  .sidebar { display: flex; flex-direction: column; gap: 12px; overflow-y: auto; }
  .card h2 { font-size: .85rem; text-transform: uppercase; letter-spacing: .08em; color: var(--muted); margin-bottom: 12px; }
  .stat-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .stat { background: var(--bg); border-radius: 8px; padding: 10px; }
  .stat label { font-size: .7rem; color: var(--muted); display: block; margin-bottom: 2px; }
  .stat value { font-size: 1.3rem; font-weight: 700; color: var(--accent); }
  .base-row { display: flex; align-items: center; gap: 10px; padding: 8px 0; border-bottom: 1px solid var(--border); }
  .base-row:last-child { border-bottom: none; }
  .base-id { width: 28px; height: 28px; border-radius: 50%; display: flex; align-items: center; justify-content: center; font-weight: 700; font-size: .85rem; }
  .b0 { background: #4f8ef744; color: #4f8ef7; }
  .b1 { background: #f59e0b44; color: #f59e0b; }
  .b2 { background: #22c55e44; color: #22c55e; }
  .base-info { flex: 1; }
  .base-info .name { font-size: .8rem; font-weight: 600; }
  .base-info .vals { font-size: .75rem; color: var(--muted); }
  .signal-bar { height: 4px; border-radius: 2px; background: var(--border); margin-top: 4px; }
  .signal-bar .fill { height: 100%; border-radius: 2px; transition: width .4s; }
  .floor-badge { display: inline-flex; align-items: center; gap: 6px; background: var(--accent); color: #fff; padding: 6px 14px; border-radius: 20px; font-weight: 700; font-size: 1.1rem; }
  input, select { width: 100%; background: var(--bg); border: 1px solid var(--border); color: var(--text); padding: 7px 10px; border-radius: 6px; font-size: .85rem; margin-bottom: 8px; }
  button { width: 100%; padding: 9px; background: var(--accent); color: #fff; border: none; border-radius: 6px; font-weight: 600; cursor: pointer; margin-bottom: 6px; }
  button:hover { opacity: .85; }
  button.danger { background: var(--red); }
  .trail-toggle { display: flex; align-items: center; gap: 8px; font-size: .8rem; cursor: pointer; margin-bottom: 8px; }
  .trail-toggle input { width: auto; margin: 0; }
</style>
</head>
<body>
<header>
  <div class="dot" id="wsDot"></div>
  <h1>🛰 BLE Tracker — Live Dashboard</h1>
</header>

<div class="layout">
  <!-- Map -->
  <div class="card" id="mapCard">
    <h2>Position Map (XY Plane)</h2>
    <canvas id="mapCanvas"></canvas>
  </div>

  <!-- Sidebar -->
  <div class="sidebar">

    <!-- Position -->
    <div class="card">
      <h2>Position</h2>
      <div class="stat-grid">
        <div class="stat"><label>X (m)</label><value id="posX">—</value></div>
        <div class="stat"><label>Y (m)</label><value id="posY">—</value></div>
        <div class="stat"><label>Altitude (m)</label><value id="altM">—</value></div>
        <div class="stat"><label>Accuracy (m)</label><value id="acc">—</value></div>
      </div>
      <br>
      <div style="text-align:center">
        <span class="floor-badge">Floor <span id="floorNum">?</span></span>
      </div>
    </div>

    <!-- Base stations -->
    <div class="card">
      <h2>Base Stations</h2>
      <div id="baseList"></div>
    </div>

    <!-- Config -->
    <div class="card">
      <h2>Configuration</h2>
      <label style="font-size:.75rem;color:var(--muted)">Path Loss Exponent n</label>
      <input type="number" id="cfgN" step="0.1" min="1.5" max="5" placeholder="2.5">
      <button onclick="sendConfig()">Apply Path Loss</button>

      <label style="font-size:.75rem;color:var(--muted);display:block;margin-top:8px">Base Coords (id,x,y)</label>
      <input type="text" id="cfgBase" placeholder="0,0,0">
      <button onclick="sendBaseCoord()">Set Base Coord</button>

      <label style="font-size:.75rem;color:var(--muted);display:block;margin-top:8px">Register Floor (floor #)</label>
      <input type="number" id="cfgFloor" min="0" max="19" placeholder="0">
      <button onclick="regFloor()">Register Current Alt as Floor</button>

      <button class="danger" style="margin-top:8px" onclick="clearLog()">Clear Position Log</button>
    </div>

    <!-- Trail toggle -->
    <div class="card">
      <h2>Display</h2>
      <label class="trail-toggle">
        <input type="checkbox" id="showTrail" checked> Show position trail
      </label>
      <label class="trail-toggle">
        <input type="checkbox" id="autoScale" checked> Auto-scale map
      </label>
    </div>

  </div><!-- sidebar -->
</div><!-- layout -->

<script>
const TRAIL_MAX = 200;
let ws, trail = [], bases = [{},{},{}], fix = null;
let autoScale = true;
let mapScale = 50, mapOffX = 0, mapOffY = 0;

// ── WebSocket ─────────────────────────────────────────────────
function connect() {
  ws = new WebSocket('ws://' + location.hostname + ':81/ws');
  ws.onopen  = () => { document.getElementById('wsDot').classList.add('connected'); };
  ws.onclose = () => { document.getElementById('wsDot').classList.remove('connected'); setTimeout(connect, 2000); };
  ws.onmessage = e => {
    const d = JSON.parse(e.data);
    if (d.type === 'position') handlePosition(d);
    if (d.type === 'bases')    handleBases(d.bases);
  };
}

function handlePosition(d) {
  fix = d;
  document.getElementById('posX').textContent    = d.x.toFixed(2);
  document.getElementById('posY').textContent    = d.y.toFixed(2);
  document.getElementById('altM').textContent    = d.altM.toFixed(2);
  document.getElementById('acc').textContent     = d.accuracy >= 0 ? d.accuracy.toFixed(2) : '—';
  document.getElementById('floorNum').textContent= d.floor;

  if (document.getElementById('showTrail').checked) {
    trail.push({x: d.x, y: d.y});
    if (trail.length > TRAIL_MAX) trail.shift();
  }
  drawMap();
}

function handleBases(b) {
  bases = b;
  const list = document.getElementById('baseList');
  list.innerHTML = '';
  const colors = ['#4f8ef7','#f59e0b','#22c55e'];
  b.forEach((base, i) => {
    const quality = base.valid ? Math.max(0, Math.min(100, (base.rssi + 100) * 2)) : 0;
    list.innerHTML += `
      <div class="base-row">
        <div class="base-id b${i}">${i}</div>
        <div class="base-info">
          <div class="name">Base ${i} ${base.valid ? '' : '⚠ stale'}</div>
          <div class="vals">RSSI: ${base.rssi ?? '—'} dBm &nbsp;|&nbsp; Dist: ${base.dist ? base.dist.toFixed(2)+'m' : '—'}</div>
          <div class="signal-bar"><div class="fill" style="width:${quality}%;background:${colors[i]}"></div></div>
        </div>
      </div>`;
  });
}

// ── Canvas map ───────────────────────────────────────────────
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
  canvas.width  = canvas.offsetWidth;
  canvas.height = canvas.offsetHeight;
  drawMap();
}
window.addEventListener('resize', resizeCanvas);

function worldToCanvas(x, y) {
  return [
    canvas.width/2  + (x - mapOffX) * mapScale,
    canvas.height/2 - (y - mapOffY) * mapScale
  ];
}

function drawMap() {
  resizeCanvas();
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Grid
  ctx.strokeStyle = '#1e2130'; ctx.lineWidth = 1;
  const step = mapScale;
  for (let x = (canvas.width/2 % step); x < canvas.width; x += step) {
    ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,canvas.height); ctx.stroke();
  }
  for (let y = (canvas.height/2 % step); y < canvas.height; y += step) {
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(canvas.width,y); ctx.stroke();
  }

  // Auto-scale: fit all bases
  if (document.getElementById('autoScale').checked && bases.length) {
    const xs = bases.map(b => b.x ?? 0);
    const ys = bases.map(b => b.y ?? 0);
    if (fix) { xs.push(fix.x); ys.push(fix.y); }
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const range = Math.max(maxX - minX, maxY - minY, 2) * 1.4;
    mapScale = Math.min(canvas.width, canvas.height) / range;
    mapOffX = (minX + maxX) / 2;
    mapOffY = (minY + maxY) / 2;
  }

  // Trail
  if (trail.length > 1) {
    ctx.beginPath();
    ctx.strokeStyle = 'rgba(79,142,247,0.35)'; ctx.lineWidth = 2;
    let [tx, ty] = worldToCanvas(trail[0].x, trail[0].y);
    ctx.moveTo(tx, ty);
    for (let i = 1; i < trail.length; i++) {
      [tx, ty] = worldToCanvas(trail[i].x, trail[i].y);
      ctx.lineTo(tx, ty);
    }
    ctx.stroke();
  }

  // Base stations
  const bColors = ['#4f8ef7','#f59e0b','#22c55e'];
  bases.forEach((base, i) => {
    if (base.x == null) return;
    const [cx, cy] = worldToCanvas(base.x, base.y);

    // Distance circle
    if (base.dist && base.valid) {
      ctx.beginPath();
      ctx.arc(cx, cy, base.dist * mapScale, 0, Math.PI*2);
      ctx.strokeStyle = bColors[i] + '44'; ctx.lineWidth = 1.5;
      ctx.stroke();
    }

    // Base marker
    ctx.beginPath(); ctx.arc(cx, cy, 8, 0, Math.PI*2);
    ctx.fillStyle = bColors[i]; ctx.fill();
    ctx.fillStyle = '#fff'; ctx.font = 'bold 10px sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(i, cx, cy);

    // Label
    ctx.fillStyle = bColors[i]; ctx.font = '11px sans-serif';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`Base ${i}`, cx, cy - 10);
  });

  // Tracker position
  if (fix && fix.valid) {
    const [px, py] = worldToCanvas(fix.x, fix.y);

    // Accuracy circle
    if (fix.accuracy > 0) {
      ctx.beginPath();
      ctx.arc(px, py, fix.accuracy * mapScale, 0, Math.PI*2);
      ctx.fillStyle = 'rgba(79,142,247,0.08)';
      ctx.strokeStyle = 'rgba(79,142,247,0.3)'; ctx.lineWidth = 1;
      ctx.fill(); ctx.stroke();
    }

    // Tracker dot
    ctx.beginPath(); ctx.arc(px, py, 12, 0, Math.PI*2);
    ctx.fillStyle = '#4f8ef7'; ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();

    // Crosshair
    ctx.strokeStyle = 'rgba(255,255,255,0.4)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(px-18,py); ctx.lineTo(px+18,py); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(px,py-18); ctx.lineTo(px,py+18); ctx.stroke();

    ctx.fillStyle = '#fff'; ctx.font = 'bold 11px sans-serif';
    ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    ctx.fillText(`(${fix.x.toFixed(1)}, ${fix.y.toFixed(1)})`, px, py - 15);
  }
}

// ── Config helpers ───────────────────────────────────────────
function sendConfig() {
  const n = parseFloat(document.getElementById('cfgN').value);
  if (isNaN(n)) return;
  fetch('/config', { method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({pathLossN: n}) });
}

function sendBaseCoord() {
  const parts = document.getElementById('cfgBase').value.split(',');
  if (parts.length < 3) return;
  fetch('/config', { method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({base: {id: +parts[0], x: +parts[1], y: +parts[2]}}) });
}

function regFloor() {
  const f = parseInt(document.getElementById('cfgFloor').value);
  fetch('/config', { method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({registerFloor: f}) });
}

function clearLog() {
  if (confirm('Clear all logged positions?'))
    fetch('/clearlog', {method:'POST'});
}

// ── Init ─────────────────────────────────────────────────────
connect();
requestAnimationFrame(drawMap);

// Poll base coords from server on load
fetch('/state').then(r=>r.json()).then(d => {
  if (d.bases) handleBases(d.bases);
});
</script>
</body>
</html>
)rawhtml";

// ────────────────────────────────────────────────────────────────
void WebServer::begin(const char* ssid, const char* pass) {
  Serial.printf("[WIFI] Connecting to %s", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500); Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WIFI] Failed to connect. Check credentials.");
    return;
  }

  setupRoutes();

  ws.onEvent([](AsyncWebSocket* srv, AsyncWebSocketClient* client,
                AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT)
      Serial.printf("[WS] Client #%u connected\n", client->id());
  });

  httpServer.addHandler(&ws);
  httpServer.begin();
  Serial.printf("[HTTP] Server on port %d\n", WEB_SERVER_PORT);
}

// ────────────────────────────────────────────────────────────────
void WebServer::setupRoutes() {
  // Dashboard
  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", DASHBOARD_HTML);
  });

  // Snapshot of current state (for initial page load)
  httpServer.on("/state", HTTP_GET, [](AsyncWebServerRequest* req) {
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
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  // Download full position log as JSON
  httpServer.on("/log", HTTP_GET, [](AsyncWebServerRequest* req) {
    // Stream response directly to avoid building a 24KB String in RAM
    AsyncResponseStream *response = req->beginResponseStream("application/json");
    response->print("[");
    for (int i = 0; i < storage.logCount(); i++) {
      if (i > 0) response->print(",");
      LogEntry e = storage.getLog(i);
      char buf[128];
      snprintf(buf, sizeof(buf),
               "{\"ts\":%lu,\"x\":%.3f,\"y\":%.3f,\"alt\":%.3f,\"floor\":%d,\"acc\":%.3f}",
               (unsigned long)e.ts, e.x, e.y, e.altM, e.floor, e.accuracy);
      response->print(buf);
    }
    response->print("]");
    req->send(response);
  });

  // Clear log
  httpServer.on("/clearlog", HTTP_POST, [](AsyncWebServerRequest* req) {
    storage.clearLog();
    req->send(200, "text/plain", "OK");
  });

  // Config updates from dashboard
  httpServer.on("/config", HTTP_POST,
    [](AsyncWebServerRequest* req) { req->send(200, "text/plain", "OK"); },
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
      StaticJsonDocument<256> doc;
      if (deserializeJson(doc, data, len)) return;

      TrackerConfig cfg;
      storage.loadConfig(cfg);
      bool changed = false;

      if (doc.containsKey("pathLossN")) {
        cfg.pathLossN = doc["pathLossN"];
        bleScanner.setPathLossN(cfg.pathLossN);
        changed = true;
      }
      if (doc.containsKey("base")) {
        uint8_t id = doc["base"]["id"];
        cfg.baseCoords[id].x = doc["base"]["x"];
        cfg.baseCoords[id].y = doc["base"]["y"];
        positioning.setBaseCoord(id, cfg.baseCoords[id].x, cfg.baseCoords[id].y);
        changed = true;
      }
      if (doc.containsKey("registerFloor")) {
        uint8_t f = doc["registerFloor"];
        float h = 0.0f; // altimeter disabled
        cfg.floorHeights[f] = h;
        cfg.floorCount = max((int)cfg.floorCount, (int)f + 1);
        positioning.setFloorHeight(f, h);
        positioning.setFloorCount(cfg.floorCount);
        changed = true;
        Serial.printf("[WEB] Floor %d registered at %.3f m\n", f, h);
      }

      if (changed) storage.saveConfig(cfg);
    }
  );
}

// ────────────────────────────────────────────────────────────────
void WebServer::update() {
  uint32_t now = millis();
  if (now - _lastPush < WS_PUSH_INTERVAL_MS) return;
  _lastPush = now;

  if (ws.count() == 0) return;

  // Position update
  {
    const PositionFix& fix = positioning.latest();
    DynamicJsonDocument doc(256); // Heap allocation
    doc["type"]     = "position";
    doc["x"]        = fix.valid ? fix.x        : 0;
    doc["y"]        = fix.valid ? fix.y        : 0;
    doc["altM"]     = altimeter.getAltitudeM();
    doc["floor"]    = altimeter.getFloor();
    doc["accuracy"] = fix.valid ? fix.accuracy : -1;
    doc["valid"]    = fix.valid;
    String out; serializeJson(doc, out);
    ws.textAll(out);
  }

  // Base station data
  {
    DynamicJsonDocument doc(640); // Heap allocation
    doc["type"] = "bases";
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
    ws.textAll(out);
  }
}
