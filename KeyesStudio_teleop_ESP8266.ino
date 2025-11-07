// ESP8266 AP-only Teleop Server (no STA)
// SSID: Gallery-Robot, Pass: 12345678, IP: 192.168.4.1
// Sends only f/b/l/r/s to the Mega.
// Teleop starts OFF
// Emergency is now a latch similar to teleop.

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <pgmspace.h>

const char* AP_SSID = "Gallery-Robot";
const char* AP_PASS = "12345678";
const uint8_t AP_CHANNEL = 1;

ESP8266WebServer server(80);

// start teleop OFF
bool teleopEnabled = false;
// new: emergency latch OFF
bool estopEnabled  = false;

const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Teleop</title>
<style>
html,body{height:100%;margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif}
.wrap{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:18px;height:100%;padding:18px}
.grid{display:grid;grid-template-columns:90px 90px 90px;grid-template-rows:90px 90px 90px;gap:12px;touch-action:none}
button{border:0;border-radius:12px;font-size:18px;font-weight:700;box-shadow:0 6px 14px rgba(0,0,0,.18);padding:10px;cursor:pointer}
button:active{transform:scale(.98)}
.fwd{grid-column:2;grid-row:1}.left{grid-column:1;grid-row:2}.stop{grid-column:2;grid-row:2;background:#d32f2f;color:white}
.right{grid-column:3;grid-row:2}.back{grid-column:2;grid-row:3}
#toggleBtn.on{background:#4caf50;color:white}
#toggleBtn.off{background:#e53935;color:white}
#estopBtn.on{background:#b71c1c;color:white}
#estopBtn.off{background:#757575;color:white}
.disabled{opacity:0.4;pointer-events:none}
.top-row{display:flex;gap:12px;align-items:center;justify-content:center}
</style></head><body>
<div class="wrap">
  <h2>Wall Gallery Robot Teleop</h2>

  <div class="top-row">
    <button id="toggleBtn" class="off" onclick="toggleTeleop()">Teleop: OFF</button>
    <button id="estopBtn" class="off" onclick="toggleEstop()">EMERGENCY: OFF</button>
  </div>

  <div class="grid disabled" id="btnGrid">
    <button class="fwd"  ontouchstart="go('f')" ontouchend="go('s')" onmousedown="go('f')" onmouseup="go('s')">↑</button>
    <button class="left" ontouchstart="go('l')" ontouchend="go('s')" onmousedown="go('l')" onmouseup="go('s')">←</button>
    <button class="stop" onclick="go('s')">■</button>
    <button class="right" ontouchstart="go('r')" ontouchend="go('s')" onmousedown="go('r')" onmouseup="go('s')">→</button>
    <button class="back" ontouchstart="go('b')" ontouchend="go('s')" onmousedown="go('b')" onmouseup="go('s')">↓</button>
  </div>
</div>
<script>
let teleopEnabled = false;
let estopEnabled  = false;

function updateUI() {
  const tbtn = document.getElementById('toggleBtn');
  const ebtn = document.getElementById('estopBtn');
  const grid = document.getElementById('btnGrid');

  // emergency has highest priority
  if (estopEnabled) {
    ebtn.textContent = "EMERGENCY: ON";
    ebtn.classList.remove('off'); ebtn.classList.add('on');

    // teleop must look OFF
    teleopEnabled = false;
    tbtn.textContent = "Teleop: OFF";
    tbtn.classList.remove('on'); tbtn.classList.add('off');

    grid.classList.add('disabled');
    return;
  } else {
    ebtn.textContent = "EMERGENCY: OFF";
    ebtn.classList.remove('on'); ebtn.classList.add('off');
  }

  // normal teleop UI when not in emergency
  if (teleopEnabled) {
    tbtn.textContent = "Teleop: ON";
    tbtn.classList.remove('off'); tbtn.classList.add('on');
    grid.classList.remove('disabled');
  } else {
    tbtn.textContent = "Teleop: OFF";
    tbtn.classList.remove('on'); tbtn.classList.add('off');
    grid.classList.add('disabled');
  }
}

async function toggleTeleop() {
  // if emergency is on, ignore teleop clicks
  if (estopEnabled) return;
  try {
    const resp = await fetch('/toggle', {cache:'no-store'});
    const text = await resp.text();
    teleopEnabled = (text.trim() === "ON");
    updateUI();
  } catch(e) { console.log(e); }
}

async function toggleEstop() {
  try {
    const resp = await fetch('/estop_toggle', {cache:'no-store'});
    const text = await resp.text(); // "ON" or "OFF"
    estopEnabled = (text.trim() === "ON");
    // when estop ON, also force teleop off in UI
    if (estopEnabled) {
      teleopEnabled = false;
    }
    updateUI();
  } catch(e) { console.log(e); }
}

async function go(dir){
  if (!teleopEnabled || estopEnabled) return;
  try{ await fetch(`/cmd?dir=${dir}`, {cache:'no-store'});}catch(e){console.log(e)}
}

updateUI();
</script></body></html>
)HTML";

void sendIndex(){
  server.setContentLength(strlen_P(index_html));
  server.send(200, "text/html");
  WiFiClient c = server.client();
  for (const char* p=index_html;;++p){
    char ch = pgm_read_byte(p);
    if(!ch) break;
    c.write(ch);
  }
}

void setup(){
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);         // to Mega
  delay(200);
  Serial.println("\n[ESP] Booting (AP-only)...");

  // tell Mega at startup: teleop is OFF
  Serial.write('A'); Serial.write('\n');

  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(50);
  WiFi.mode(WIFI_AP);

  IPAddress ip(192,168,4,1), gw(192,168,4,1), mask(255,255,255,0);
  bool cfgOk = WiFi.softAPConfig(ip, gw, mask);
  Serial.printf("[ESP] softAPConfig -> %s\n", cfgOk ? "OK" : "FAIL");

  bool apOk = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4);
  Serial.printf("[ESP] softAP(\"%s\", ch=%d) -> %s\n", AP_SSID, AP_CHANNEL, apOk ? "OK" : "FAIL");
  Serial.printf("[ESP] AP IP: %s (browse http://192.168.4.1)\n", WiFi.softAPIP().toString().c_str());

  WiFi.printDiag(Serial);

  // routes
  server.on("/", [](){
    Serial.println("[HTTP] GET /");
    sendIndex();
  });

  // toggle teleop (only valid when not in estop)
  server.on("/toggle", [](){
    // if estop is active, ignore
    if (estopEnabled) {
      server.send(200, "text/plain", "OFF");
      return;
    }
    teleopEnabled = !teleopEnabled;
    Serial.printf("[HTTP] /toggle -> teleop = %s\n", teleopEnabled ? "ON" : "OFF");

    // tell Mega
    if (teleopEnabled) {
      Serial.write('M'); Serial.write('\n');  // teleop ON
    } else {
      Serial.write('A'); Serial.write('\n');  // teleop OFF
    }

    server.send(200, "text/plain", teleopEnabled ? "ON" : "OFF");
  });

  // new: emergency toggle
  server.on("/estop_toggle", [](){
    estopEnabled = !estopEnabled;
    if (estopEnabled) {
      // EMERGENCY ON: tell Mega to STOP and stay stopped
      Serial.println("[HTTP] /estop_toggle -> EMERGENCY ON");
      Serial.write('E'); Serial.write('\n');  // <- latch E-stop on the Mega
      // do NOT send 'A' here
      teleopEnabled = false;
    } else {
      // EMERGENCY OFF: now we can tell the Mega we're back to auto/safe
      Serial.println("[HTTP] /estop_toggle -> EMERGENCY OFF");
      Serial.write('A'); Serial.write('\n');  // <- only on release
    }
    server.send(200, "text/plain", estopEnabled ? "ON" : "OFF");
  });

  // movement commands
  server.on("/cmd", [](){
    char c = server.hasArg("dir") ? server.arg("dir")[0] : 's';
    if (!teleopEnabled || estopEnabled) {
      Serial.printf("[HTTP] /cmd IGNORED (teleop=%d, estop=%d) dir=%c\n",
                    teleopEnabled, estopEnabled, c);
      server.send(200, "text/plain", "IGNORED");
      return;
    }
    Serial.printf("[HTTP] /cmd dir=%c  -> UART '%c\\n'\n", c, c);
    Serial.write(c); Serial.write('\n');
    server.send(200, "text/plain", "OK");
  });

  server.onNotFound([](){
    Serial.printf("[HTTP] 404 %s\n", server.uri().c_str());
    server.send(404, "text/plain", "404");
  });

  server.begin();
  Serial.println("[HTTP] Server started.");
  digitalWrite(LED_BUILTIN, LOW); delay(150); digitalWrite(LED_BUILTIN, HIGH);
}

void loop(){
  server.handleClient();

  // show AP client count every 2s
  static uint32_t t0=0; static uint8_t last=255;
  if (millis()-t0 > 2000){
    t0 = millis();
    uint8_t n = wifi_softap_get_station_num();
    if (n != last){
      Serial.printf("[ESP] AP clients: %u\n", n);
      last = n;
    }
  }
}



