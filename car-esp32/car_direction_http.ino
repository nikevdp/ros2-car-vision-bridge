
#include <WiFi.h>
#include <WebServer.h>

// ===== WIFI CONFIG =====
const char* ssid = "wifiName";
const char* pass = "wifiPass";

WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
  <!doctype html>
<html lang="es">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 Car Control</title>
  <style>
    body { font-family: system-ui, sans-serif; padding: 20px; max-width: 520px; margin: 0 auto; }
    h1 { font-size: 20px; }
    .grid { display: grid; gap: 12px; grid-template-columns: repeat(3, 1fr); }
    button { padding: 16px; font-size: 16px; border-radius: 12px; border: 1px solid #ccc; }
    button:active { transform: scale(0.98); }
    .full { grid-column: 1 / -1; }
    .status { margin-top: 14px; font-size: 14px; opacity: .8; }
  </style>
</head>

<body>
  <h1>Control del auto (LAN)</h1>

  <div class="grid">
    <div></div>
    <button id="btnF">↑ Adelante</button>
    <div></div>

    <button id="btnL">← Izq</button>
    <button id="btnS">■ Stop</button>
    <button id="btnR">Der →</button>

    <div></div>
    <button id="btnB">↓ Atrás</button>
    <div></div>

  </div>

  <div class="status" id="status">Estado: listo</div>

  <script>
    const statusEl = document.getElementById("status");

    async function sendCmd(code) {
      try {
        statusEl.textContent = `Estado: enviando ${code}...`;
        const r = await fetch(`/cmd?m=${encodeURIComponent(code)}`, { cache: "no-store" });
        const t = await r.text();
        statusEl.textContent = `Estado: ${t}`;
      } catch (e) {
        statusEl.textContent = `Estado: error (${e})`;
      }
    }

    // Clicks (simple mode)
    document.getElementById("btnF").onclick = () => sendCmd("F");
    document.getElementById("btnB").onclick = () => sendCmd("B");
    document.getElementById("btnL").onclick = () => sendCmd("L");
    document.getElementById("btnR").onclick = () => sendCmd("R");
    document.getElementById("btnS").onclick = () => sendCmd("S");


    // (Optional) hold to move and release = stop
    function holdButton(btnId, code) {
      const el = document.getElementById(btnId);
      el.addEventListener("pointerdown", () => sendCmd(code));
      el.addEventListener("pointerup",   () => sendCmd("S"));
      el.addEventListener("pointercancel", () => sendCmd("S"));
      el.addEventListener("pointerleave",  () => sendCmd("S"));
    }
    holdButton("btnF", "F");
    holdButton("btnB", "B");
    holdButton("btnL", "L");
    holdButton("btnR", "R");
  </script>
</body>
</html>
)rawliteral";

// GPIO pins to ULN2003 IN1..IN4
static const int IN1 = 25;
static const int IN2 = 26;
static const int IN3 = 27;
static const int IN4 = 14;
static const int ENA = 32;
static const int ENB = 33;
static const int PWM_FREQ = 20000;      // 20 kHz (quiet)
static const int PWM_RES  = 8;          // 8 bits => 0..255
volatile char last_cmd = 'S';        // 'F','B','L','R','S'
volatile uint32_t last_ms = 0;

void registerRoutes() {
  server.on("/status", HTTP_GET, []() {
    uint32_t age_ms = millis() - last_ms;

    // Build "F 1234\n" (last_cmd + age_ms)
    String body;
    body.reserve(20);
    body += last_cmd;
    body += ' ';
    body += String(age_ms);
    body += '\n';
    server.send(200, "text/plain; charset=utf-8", body);
  });
}

void setupPwm() {
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  // Start with motors off
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

void engineB(uint8_t V3, uint8_t V4){
  digitalWrite(IN3, V3);
  digitalWrite(IN4, V4);
}

void engineA(uint8_t V1, uint8_t V2){
  digitalWrite(IN1, V1);
  digitalWrite(IN2, V2);
}

void speedControl(uint8_t speed){
  ledcWrite(ENA, speed);
  ledcWrite(ENB, speed);
}

void highSpeed(){
  speedControl(255);
}

void halfSpeed(){
  speedControl(230);
}

void slowSpeed(){
  speedControl(200);
}

void forward(){
  engineB(LOW, HIGH);
  engineA(HIGH, LOW);
  highSpeed();
}

void backward(){
  engineB(HIGH, LOW);
  engineA(LOW, HIGH);
  halfSpeed();
}

void left(){
  engineB(LOW, HIGH);
  engineA(LOW, HIGH);
  slowSpeed();
}

void right(){
  engineB(HIGH, LOW);
  engineA(HIGH, LOW);
  slowSpeed();
}

void stopCar() {
  engineB(LOW, LOW);
  engineA(LOW, LOW);
  speedControl(0);
}

void setup() {
  Serial.begin(115200);

  Serial.println("=== TEST ESP32 ===");
  Serial.println("Inicializando...");
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  setupPwm();
  stopCar();
  registerRoutes();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Serial.print("Conectando WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  // Main page
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html; charset=utf-8", INDEX_HTML);
  });

  // Command endpoint: /cmd?m=F
  server.on("/cmd", HTTP_GET, []() {
    if (!server.hasArg("m")) {
      server.send(400, "text/plain; charset=utf-8", "ERR: missing m");
      return;
    }
    String m = server.arg("m");
    String resp = handleMoveCode(m);
    server.send(200, "text/plain; charset=utf-8", resp);
  });

  server.begin();
  Serial.println("HTTP server listo.");
}

void sendData(char cmd) {
  last_cmd = cmd;
  last_ms  = millis();
}

String handleMoveCode(const String& m) {
  if (m == "F") { sendData('F'); forward();  return "OK: forward"; }
  if (m == "B") { sendData('B'); backward(); return "OK: backward"; }
  if (m == "L") { sendData('L'); left();     return "OK: left"; }
  if (m == "R") { sendData('R'); right();    return "OK: right"; }
  if (m == "S") { sendData('S'); stopCar();  return "OK: stop"; }
  return "ERR: unknown cmd";
}

void loop() {
  server.handleClient();
}
