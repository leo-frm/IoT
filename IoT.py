/*
 * ============================================================
 *  Tap-in / Tap-out  ->  ThingsBoard CE 4.3.1 (via MQTT)
 *  Hardware: ESP32-C3 Super Mini + leitor NFC PN532 (HSU/UART)
 * ============================================================
 *  Mantem a leitura PN532 do sketch original; acrescenta WiFi,
 *  NTP, publicacao MQTT no ThingsBoard, e SINAL VISUAL no LED
 *  onboard (pisca 2x quando WiFi+NTP+MQTT estao prontos).
 *
 *  Bibliotecas a instalar (Library Manager):
 *    - Adafruit PN532
 *    - PubSubClient (Nick O'Leary)
 *
 *  Use ESTE arquivo nos dois ESPs, mudando apenas:
 *    - TB_TOKEN  (token do catraca-tapin / catraca-tapout)
 *    - TAP_TYPE  ("in" num ESP, "out" no outro)
 * ============================================================
 */

#include <PN532_HSU.h>
#include <PN532.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <sys/time.h>

// ===================== AJUSTE AQUI =========================
const char* WIFI_SSID = "GD03";
const char* WIFI_PASS = "Gd#03!@8";

const char* TB_HOST   = "10.10.115.172";
const int   TB_PORT   = 1883;
const char* TB_TOKEN  = "tCeOBf0iozgs31tySH0Q";

// "in" neste ESP da entrada; troque para "out" no outro ESP:
const char* TAP_TYPE  = "in";

const char* NTP_SERVER = "pool.ntp.org";

// LED onboard do ESP32-C3 Super Mini (active-LOW: LOW = aceso, HIGH = apagado)
#define LED_PIN 8
// ===========================================================

HardwareSerial mySerial(1);
PN532_HSU pn532hsu(mySerial);
PN532 nfc(pn532hsu);

WiFiClient   espClient;
PubSubClient mqtt(espClient);

// Debounce: evita reenviar o mesmo cartao varias vezes em sequencia
String lastCard = "";
unsigned long lastReadMs = 0;
const unsigned long DEBOUNCE_MS = 3000;

// Sinalizacao de "pronto" (acontece UMA vez quando tudo conecta)
bool readySignaled = false;

void ledReady() {
  digitalWrite(LED_PIN, LOW);  // LED aceso (active-LOW) — fica assim ate reset
}

void setupWifi() {
  Serial.printf("Conectando WiFi '%s'", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
}

void setupTime() {
  configTime(0, 0, NTP_SERVER);
  Serial.print("Sincronizando hora via NTP");
  struct tm t;
  while (!getLocalTime(&t)) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" OK");
}

uint64_t epochMillis() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000ULL);
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.print("Conectando MQTT...");
    String clientId = "esp32-tap-" + String(TAP_TYPE);
    if (mqtt.connect(clientId.c_str(), TB_TOKEN, "")) {
      Serial.println(" conectado!");
    } else {
      Serial.printf(" falhou (rc=%d). Nova tentativa em 5s\n", mqtt.state());
      delay(5000);
    }
  }
}

String uidToString(uint8_t* uid, uint8_t len) {
  String s = "";
  for (uint8_t i = 0; i < len; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
  }
  s.toUpperCase();
  return s;
}

void publishTap(const String& cardId) {
  uint64_t ts = epochMillis();
  char payload[160];
  snprintf(payload, sizeof(payload),
           "{\"ts\":%llu,\"values\":{\"cardId\":\"%s\",\"tapType\":\"%s\"}}",
           (unsigned long long)ts, cardId.c_str(), TAP_TYPE);

  if (mqtt.publish("v1/devices/me/telemetry", payload)) {
    Serial.printf("Enviado: %s\n", payload);
  } else {
    Serial.println("Falha ao publicar (rc=" + String(mqtt.state()) + ")");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando...");

  // LED apagado de cara
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // active-LOW: HIGH = apagado

  // --- Inicializacao PN532 ---
  mySerial.begin(115200, SERIAL_8N1, 20, 21);
  Serial.println("Serial1 OK");

  nfc.begin();
  Serial.println("NFC begin OK");

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 nao encontrado! Verifique ligacao e DIP switch.");
    while (1);
  }
  Serial.print("Chip PN5");
  Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware: ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.');
  Serial.println((versiondata >> 8) & 0xFF, DEC);

  nfc.setPassiveActivationRetries(0xFF);
  nfc.SAMConfig();
  // ----------------------------

  setupWifi();
  setupTime();
  mqtt.setServer(TB_HOST, TB_PORT);

  Serial.printf("Pronto. Modo TAP-%s. Aguardando cartao...\n", TAP_TYPE);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) setupWifi();
  if (!mqtt.connected())             mqttReconnect();
  mqtt.loop();
  digitalWrite(LED_PIN, mqtt.connected() ? LOW : HIGH);

  // Primeira vez que tudo (WiFi + NTP + MQTT) esta verde: pisca 2x
  if (!readySignaled) {
    Serial.println(">>> Sistema pronto pra uso (WiFi+NTP+MQTT OK)");
    ledReady();
    readySignaled = true;
  }

  uint8_t uid[7] = {0};
  uint8_t uidLength;

  bool success = nfc.readPassiveTargetID(
      PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    String cardId = uidToString(uid, uidLength);
    unsigned long now = millis();

    if (cardId == lastCard && (now - lastReadMs) < DEBOUNCE_MS) {
      delay(500);
      return;
    }
    lastCard = cardId;
    lastReadMs = now;

    Serial.print("Cartao detectado: ");
    Serial.println(cardId);

    publishTap(cardId);

    delay(1000);
  }
}
