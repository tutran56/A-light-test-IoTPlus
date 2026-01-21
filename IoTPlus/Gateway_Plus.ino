#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <PubSubClient.h>

// ================== WiFi hotspot ==================
const char* WIFI_SSID = "Trần Tú";
const char* WIFI_PASS = "tu12345678";

// ================== MQTT broker (MacBook) ==================
const char* MQTT_HOST = "172.20.10.3";
const int   MQTT_PORT = 1883;

// Topics
static const char* TOPIC_CMD       = "lab4/cmd";
static const char* TOPIC_TELEMETRY = "lab4/telemetry";  // publish telemetry tại đây

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ================== Telemetry from Node ==================
typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint32_t ts_ms;
  float    temp_c;
  uint8_t  alarm_on;
  uint8_t  backup_on;
} TelemetryMsg;

// ================== Control to Node ==================
enum CmdType : uint8_t {
  CMD_NONE        = 0,
  CMD_RESET_ALARM = 1,
  CMD_BACKUP_ON   = 2,
  CMD_BACKUP_OFF  = 3
};

typedef struct __attribute__((packed)) {
  uint32_t cmd_seq;
  uint8_t  cmd_type;
} ControlMsg;

// ================== Ring Buffer for Store & Forward ==================
static const int BUF_CAP   = 240; // ~20 phút nếu 1 mẫu/5s (240 mẫu). Tuỳ bạn.
static const int MAX_BATCH = 50;  // flush tối đa 50 mẫu/lần

TelemetryMsg buf[BUF_CAP];
int head = 0, tail = 0, count = 0;

void bufPush(const TelemetryMsg& t) {
  if (count == BUF_CAP) {
    // drop oldest
    tail = (tail + 1) % BUF_CAP;
    count--;
  }
  buf[head] = t;
  head = (head + 1) % BUF_CAP;
  count++;
}

bool bufPop(TelemetryMsg& out) {
  if (count == 0) return false;
  out = buf[tail];
  tail = (tail + 1) % BUF_CAP;
  count--;
  return true;
}

// ================== State ==================
uint8_t nodeMac[6] = {0};
bool nodeKnown = false;
uint32_t cmdSeq = 0;

uint8_t homeCh = 0;

static const unsigned long WIFI_RETRY_MS  = 5000;
static const unsigned long MQTT_RETRY_MS  = 3000;
static const unsigned long FLUSH_EVERY_MS = 1000;

// ================== Helpers ==================
void printMac(const uint8_t* mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

void updateHomeChannel() {
  uint8_t ch; wifi_second_chan_t sch;
  esp_wifi_get_channel(&ch, &sch);
  homeCh = ch;
}

void applyEspNowChannel() {
  if (homeCh == 0) return;
  esp_wifi_set_channel(homeCh, WIFI_SECOND_CHAN_NONE);
}

void addNodePeerIfNeeded(const uint8_t* mac) {
  if (nodeKnown) return;

  memcpy(nodeMac, mac, 6);

  updateHomeChannel();
  applyEspNowChannel();

  // remove old peer record (safe if not exist)
  esp_now_del_peer(nodeMac);

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, nodeMac, 6);
  p.channel = homeCh;
  p.encrypt = false;

  esp_err_t r = esp_now_add_peer(&p);
  if (r == ESP_OK) {
    nodeKnown = true;
    Serial.print("NODE peer added (ch=");
    Serial.print(homeCh);
    Serial.print("): ");
    printMac(nodeMac);
    Serial.println();
  } else {
    Serial.print("Add node peer FAILED, err=");
    Serial.println((int)r);
  }
}

void sendCmdToNode(CmdType cmd) {
  if (!nodeKnown) {
    Serial.println("No node MAC yet -> cannot send cmd");
    return;
  }

  updateHomeChannel();
  applyEspNowChannel();

  ControlMsg c = {};
  c.cmd_seq  = cmdSeq++;
  c.cmd_type = (uint8_t)cmd;

  esp_err_t r = esp_now_send(nodeMac, (uint8_t*)&c, sizeof(c));

  Serial.print("Forward CMD to node: ");
  Serial.print((int)cmd);
  Serial.print(" | ch=");
  Serial.print(homeCh);
  Serial.print(" | esp_now_send=");
  Serial.println(r == ESP_OK ? "ESP_OK" : "ERR");
}

// ================== MQTT publish (telemetry) ==================
String makeJsonOne(const TelemetryMsg& t) {
  String s;
  s.reserve(128);
  s += "{";
  s += "\"seq\":" + String(t.seq) + ",";
  s += "\"ts_ms\":" + String(t.ts_ms) + ",";
  s += "\"temp\":" + String(t.temp_c, 2) + ",";
  s += "\"alarm\":" + String(t.alarm_on) + ",";
  s += "\"backup\":" + String(t.backup_on);
  s += "}";
  return s;
}

String makeJsonBatch(TelemetryMsg* arr, int n) {
  String s;
  s.reserve(64 + n * 80);
  s += "{\"samples\":[";
  for (int i = 0; i < n; i++) {
    s += makeJsonOne(arr[i]);
    if (i != n - 1) s += ",";
  }
  s += "]}";
  return s;
}

bool publishBatch(TelemetryMsg* arr, int n) {
  if (!mqtt.connected()) return false;
  String payload = makeJsonBatch(arr, n);
  // QoS 0 (PubSubClient): publish returns bool
  return mqtt.publish(TOPIC_TELEMETRY, payload.c_str());
}

void tryFlush() {
  static unsigned long lastTry = 0;
  if (!mqtt.connected()) return;
  if (count == 0) return;
  if (millis() - lastTry < FLUSH_EVERY_MS) return;
  lastTry = millis();

  TelemetryMsg batch[MAX_BATCH];
  int n = 0;

  while (n < MAX_BATCH && count > 0) {
    TelemetryMsg t;
    if (!bufPop(t)) break;
    batch[n++] = t;
  }

  bool ok = publishBatch(batch, n);
  if (ok) {
    Serial.printf("FLUSH OK: sent %d samples, remaining buf=%d\n", n, count);
  } else {
    // publish fail -> push back theo thứ tự
    for (int i = 0; i < n; i++) bufPush(batch[i]);
    Serial.printf("FLUSH FAIL: keep %d samples, buf=%d\n", n, count);
  }
}

// ================== ESP-NOW receive ==================
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != (int)sizeof(TelemetryMsg)) return;

  TelemetryMsg t;
  memcpy(&t, data, sizeof(t));

  addNodePeerIfNeeded(info->src_addr);

  // Store first (Store & Forward)
  bufPush(t);

  // Log + buffer size
  Serial.printf("RX seq=%u temp=%.2f alarm=%u backup=%u | buf=%d\n",
                t.seq, t.temp_c, t.alarm_on, t.backup_on, count);
}

// ================== MQTT callback ==================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.print("MQTT msg: ");
  Serial.println(msg);

  if (msg == "RESET_ALARM") {
    sendCmdToNode(CMD_RESET_ALARM);
  } else if (msg == "TURN_ON_BACKUP_COOLER") {
    sendCmdToNode(CMD_BACKUP_ON);
  } else if (msg == "TURN_OFF_BACKUP_COOLER") {
    sendCmdToNode(CMD_BACKUP_OFF);
  } else {
    Serial.println("Unknown command");
  }
}

// ================== WiFi / MQTT maintenance ==================
void ensureWiFi() {
  static unsigned long lastTry = 0;

  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastTry < WIFI_RETRY_MS) return;
  lastTry = millis();

  Serial.println("WiFi: begin()");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void ensureMQTT() {
  static unsigned long lastTry = 0;

  if (mqtt.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  if (millis() - lastTry < MQTT_RETRY_MS) return;
  lastTry = millis();

  String cid = "gateway-" + WiFi.macAddress();

  Serial.print("MQTT: connecting to ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  if (mqtt.connect(cid.c_str())) {
    Serial.println("MQTT connected");
    mqtt.subscribe(TOPIC_CMD);
    Serial.print("Subscribed: ");
    Serial.println(TOPIC_CMD);

    updateHomeChannel();
    Serial.print("GATEWAY CH: ");
    Serial.println(homeCh);

    applyEspNowChannel();
    Serial.print("ESP-NOW home channel set to: ");
    Serial.println(homeCh);
  } else {
    Serial.print("MQTT connect fail, rc=");
    Serial.println(mqtt.state());
  }
}

// ================== Setup / Loop ==================
// ================== Setup / Loop ==================
void setup() {
  Serial.begin(115200);
  delay(300);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);

  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.print("GATEWAY MAC: ");
  Serial.println(WiFi.macAddress());

  // Start WiFi (for MQTT)
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // ESP-NOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
  Serial.println("ESP-NOW receiver ready.");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  // === QUAN TRỌNG: Dòng này giúp fix lỗi FLUSH FAIL ===
  mqtt.setBufferSize(4096); // Nới rộng bộ đệm để chứa được gói tin to
}

void loop() {
  ensureWiFi();
  ensureMQTT();

  if (mqtt.connected()) {
    mqtt.loop();
    tryFlush();   // Store & Forward flush
  }

  delay(10);
}
