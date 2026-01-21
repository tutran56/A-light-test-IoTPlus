#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ADC_PIN       34
#define LED_ALARM     25
#define LED_BACKUP    26

uint8_t GATEWAY_MAC[] = { 0x88, 0x57, 0x21, 0xBC, 0x56, 0x8C };
static const uint8_t ESPNOW_CH = 6;

const float ALARM_TEMP  = 8.0;
const float BACKUP_TEMP = 10.0;
const unsigned long SAMPLE_MS = 5000;

typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint32_t ts_ms;
  float    temp_c;
  uint8_t  alarm_on;
  uint8_t  backup_on;
} TelemetryMsg;

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

uint32_t seq = 0;
unsigned long lastSend = 0;
float tempEMA = 5.0;
const float alpha = 0.2;

unsigned long alarmSuppressUntil = 0;
int backupForced = -1;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(ControlMsg)) return;

  ControlMsg c;
  memcpy(&c, data, sizeof(c));

  Serial.printf("CMD: %d (Seq=%u)\n", c.cmd_type, c.cmd_seq);

  if (c.cmd_type == CMD_RESET_ALARM) {
    alarmSuppressUntil = millis() + 10000;
  } else if (c.cmd_type == CMD_BACKUP_ON) {
    backupForced = 1;
  } else if (c.cmd_type == CMD_BACKUP_OFF) {
    backupForced = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_ALARM, OUTPUT);
  pinMode(LED_BACKUP, OUTPUT);
  digitalWrite(LED_ALARM, LOW);
  digitalWrite(LED_BACKUP, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    ESP.restart();
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, GATEWAY_MAC, 6);
  peerInfo.channel = ESPNOW_CH;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Add Peer Failed");
  } else {
    Serial.println("Node Ready");
  }
}

void loop() {
  unsigned long now = millis();

  int adc = analogRead(ADC_PIN);
  float tempRaw = mapFloat(adc, 0, 4095, 0.0, 15.0);
  
  tempEMA = alpha * tempRaw + (1.0 - alpha) * tempEMA;

  bool alarmCondition = (tempEMA > ALARM_TEMP);
  bool alarmState = alarmCondition;
  
  if (now < alarmSuppressUntil) {
    alarmState = false; 
  }

  bool backupCondition = (tempEMA > BACKUP_TEMP);
  bool backupState = backupCondition;

  if (backupForced == 1) backupState = true;
  if (backupForced == 0) backupState = false;

  digitalWrite(LED_ALARM, alarmState ? HIGH : LOW);
  digitalWrite(LED_BACKUP, backupState ? HIGH : LOW);

  if (now - lastSend >= SAMPLE_MS) {
    lastSend = now;

    TelemetryMsg t;
    t.seq = seq++;
    t.ts_ms = now;
    t.temp_c = tempEMA;
    t.alarm_on = alarmState ? 1 : 0;
    t.backup_on = backupState ? 1 : 0;

    esp_err_t result = esp_now_send(GATEWAY_MAC, (uint8_t*)&t, sizeof(t));

    Serial.printf("Seq=%u Temp=%.2f Alarm=%d Backup=%d -> %s\n", 
                  t.seq, t.temp_c, t.alarm_on, t.backup_on, 
                  result == ESP_OK ? "OK" : "ERR");
  }
  
  delay(20);
}