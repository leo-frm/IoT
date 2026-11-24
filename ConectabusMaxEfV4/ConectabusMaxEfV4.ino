/*
 * ============================================================
 *  Tap-in / Tap-out -> ThingsBoard (via MQTT)
 *  Hardware: ESP32-C3 Super Mini + PN532 (HSU) + IRQ no GPIO 2
 *
 *  *** VERSAO 4 - MAXIMA EFICIENCIA + SERVIDOR PELO NOME ***
 *  Baseada na ConectabusMaxEfV31. Mudancas:
 *
 *   [A] SERVIDOR PELO NOME (nao mais IP fixo)
 *       TB_HOST agora e' o NOME da maquina do ThingsBoard. O ESP
 *       resolve o nome por DNS normal e, se nao achar, por mDNS
 *       (nome.local). O IP encontrado fica guardado na RTC memory
 *       (sobrevive ao deep sleep) -> nao gasta radio procurando o
 *       nome a cada acordada. So procura de novo quando a conexao
 *       MQTT falha (sinal de que o IP do servidor mudou).
 *
 *   [B] DEEP SLEEP LIGADO (DEBUG_KEEP_AWAKE = 0)
 *       Na V31 estava em 1 -> nunca dormia, sem economia nenhuma.
 *
 *   [C] NTP CORRIGIDO
 *       Na V31, no resync o getLocalTime() retornava OK na hora
 *       (o relogio ja era valido) e o WiFi desligava antes do NTP
 *       responder -> o resync nunca acontecia de verdade. Agora
 *       espera o status real do SNTP, com timeout.
 *
 *   [D] Nome do proprio ESP na rede: conectabus-tapin / -tapout
 *
 *   [E] BUZZER no GPIO 5: apita a cada cartao lido com sucesso.
 *       Pino fica travado em LOW durante o deep sleep (nao apita
 *       nem gasta corrente dormindo).
 *
 *   [F] SINAIS DE LED + BUZZER (iguais ao TesteHardware):
 *    - Ao ligar: 1 bip + 1 piscada longa  -> LED e buzzer OK
 *    - PN532 encontrado: 2 bips curtos
 *    - PN532 NAO encontrado: 3 bips graves e tenta de novo a cada 3s
 *      (depois de PN532_FAST_RETRIES tentativas, passa a cada 30s
 *      para nao gastar a bateria)
 *    - Cartao lido: bip + piscada
 *    Os sinais de "ligou" e "PN532 encontrado" so tocam ao ligar ou
 *    apertar RESET, nao a cada vez que acorda do deep sleep.
 *
 *  Mantido da V31: InAutoPoll (campo RF em duty-cycle), envio em
 *  lote, NTP so quando precisa, wake por IRQ do PN532, 80 MHz.
 *
 *  Pre-requisito no servidor (Linux) para o nome.local funcionar:
 *      sudo apt install avahi-daemon
 *  Teste de outro PC da mesma rede:  ping NOME.local
 *
 *  Bibliotecas: PN532 (Elechouse/Seeed: PN532 + PN532_HSU),
 *               PubSubClient. ESPmDNS ja vem no core ESP32.
 *  Fiacao adicional: PN532 IRQ -> ESP GPIO 2
 * ============================================================
 */

#include <Arduino.h>
#include <PN532_HSU.h>
#include <PN532.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <time.h>
#include <sys/time.h>
#include <esp_sntp.h>
#include <esp_sleep.h>
#include <esp_bt.h>
#include <driver/gpio.h>

// ===================== AJUSTE AQUI =========================
const char* WIFI_SSID  = "SEU_WIFI";
const char* WIFI_PASS  = "SUA_SENHA";

// [A] NOME da maquina do ThingsBoard (resultado do comando `hostname`
// nela). Pode escrever com ou sem ".local". Um IP aqui tambem funciona.
const char* TB_HOST    = "servidor-tb";
// Opcional: IP usado SO se o nome nao puder ser resolvido. "" = nenhum.
const char* TB_FALLBACK_IP = "";
const int   TB_PORT    = 1883;
const char* TB_TOKEN   = "SEU_TOKEN_THINGSBOARD";
const char* TAP_TYPE   = "in";            // "in" ou "out"
const char* NTP_SERVER = "pool.ntp.org";

#define LED_PIN   8     // LED onboard (active LOW)
#define IRQ_PIN   2     // PN532 IRQ -> aqui
#define PN532_RX  20
#define PN532_TX  21

// ---- [E] Buzzer --------------------------------------------
#define BUZZER_PIN       5
// 0 = buzzer ATIVO (apita sozinho com 3.3V - o mais comum, 2 pinos + / -)
// 1 = buzzer PASSIVO (precisa de frequencia; se o ativo nao apitar, teste 1)
#define BUZZER_PASSIVE   1
#define BUZZER_FREQ_HZ   2700   // so p/ passivo
#define BUZZER_MS        100    // duracao do bip
#define BUZZER_LOW_HZ    800    // tom grave do sinal de erro (so p/ passivo)
#define PN532_FAST_RETRIES 10   // [F] tentativas a cada 3s antes de passar a 30s

#define MAX_BUFFERED_TAPS         64
#define CONNECT_TOTAL_TIMEOUT_MS  30000   // hard reset apos isso
#define MDNS_QUERY_TIMEOUT_MS     2500    // tempo maximo procurando nome.local

// ---- Duty-cycle do campo RF (InAutoPoll) ------------------
// Periodo de polling = AUTOPOLL_PERIOD * 150ms.
//   0x01 = 150ms (mais responsivo, gasta mais)
//   0x02 = 300ms (equilibrio recomendado)  <-- default
//   0x03 = 450ms (economiza mais, ate ~0.45s pra detectar)
#define AUTOPOLL_PERIOD           0x02
#define AUTOPOLL_TYPE             0x00    // 0x00 = ISO14443A 106kbps (Mifare/Bilhete Unico)
// Se o InAutoPoll nao funcionar no seu modulo/fiacao, coloque 1
// aqui pra cair no modo antigo (varredura continua, sem economia RF):
#define USE_CONTINUOUS_FALLBACK   0

// ---- Politica de envio em lote ----------------------------
#define FLUSH_THRESHOLD           10      // sobe WiFi quando juntar N taps
#define MAX_HOLD_MS               120000  // ...ou quando o tap mais antigo passar disso (2 min)
#define IDLE_HEARTBEAT_S          300     // buffer vazio: acorda so a cada 5 min (fallback)

// ---- NTP ---------------------------------------------------
#define NTP_RESYNC_H              6       // re-sincroniza hora a cada 6h (aproveita WiFi ja ligado)

// ---- DEBUG -------------------------------------------------
// 1 = NAO entra em deep sleep (mantem a USB viva p/ ver o Serial Monitor).
//     Use so para testar. Em producao deixe 0, senao nao ha economia.
//     Para ver o Serial: Tools > USB CDC On Boot = Enabled.
#define DEBUG_KEEP_AWAKE          0

// ---- Opcionais (economia extra) ---------------------------
// IP estatico do PROPRIO ESP: pula o DHCP (~0.5-2s de radio a menos).
// Nao tem relacao com o IP do servidor (esse e' resolvido pelo nome).
#define USE_STATIC_IP             0
#if USE_STATIC_IP
IPAddress STA_IP (10,10,115,200);
IPAddress STA_GW (10,10,115,1);
IPAddress STA_MASK(255,255,255,0);
IPAddress STA_DNS(10,10,115,1);
#endif

// ESP32-C3 Super Mini: antena ruim -> NAO conecta em potencia maxima.
// 8.5 dBm e' o valor que ja funciona nessa placa.
#define WIFI_TX_POWER             WIFI_POWER_8_5dBm

// Telemetria de bateria (divisor R1=R2=100k -> leitura x2).
// Defina o GPIO do ADC usado; 0 = desabilitado.
#define BATT_ADC_PIN              0       // ex: 3 (GPIO3 = ADC1). 0 desliga.
#define BATT_DIVIDER              2.0f    // (R1+R2) / R2
// ===========================================================

// Entrada do buffer (na RTC memory)
typedef struct {
  uint64_t ts_ms;
  char     cardId[16];
} TapEntry;

// Estado persistente entre deep sleeps (RTC memory)
RTC_DATA_ATTR bool     firstBoot       = true;
RTC_DATA_ATTR uint32_t bootCount       = 0;
RTC_DATA_ATTR uint32_t tapsTotalSent   = 0;
RTC_DATA_ATTR uint32_t bufferCount     = 0;
RTC_DATA_ATTR uint64_t lastNtpSyncMs   = 0;
RTC_DATA_ATTR uint32_t tbIpCache       = 0;   // [A] IP do servidor ja resolvido (0 = nao sei)
RTC_DATA_ATTR uint32_t pn532FailCount  = 0;   // [F] falhas seguidas do PN532
RTC_DATA_ATTR TapEntry tapBuffer[MAX_BUFFERED_TAPS];

HardwareSerial mySerial(1);
PN532_HSU pn532hsu(mySerial);
PN532 nfc(pn532hsu);

WiFiClient   espClient;
PubSubClient mqtt(espClient);

bool mdnsStarted = false;

// ============= HELPERS ===============

inline void ledOn()  { digitalWrite(LED_PIN, LOW);  }
inline void ledOff() { digitalWrite(LED_PIN, HIGH); }

void blink(int ms_on, int times = 1, int ms_off = 150) {
  for (int i = 0; i < times; i++) {
    ledOn();  delay(ms_on);
    ledOff(); if (i < times - 1) delay(ms_off);
  }
}

uint64_t epochMillis() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL;
}

bool timeIsValid() {
  // Depois do NTP o epoch fica > ~2020. Antes disso, invalido.
  return epochMillis() > 1600000000000ULL;
}

String uidToString(uint8_t *uid, uint8_t len) {
  String s = "";
  for (uint8_t i = 0; i < len; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
  }
  s.toUpperCase();
  return s;
}

// [E] Bip (freq so vale para buzzer passivo)
void beep(int ms = BUZZER_MS, int freq = BUZZER_FREQ_HZ) {
#if BUZZER_PASSIVE
  tone(BUZZER_PIN, freq);
  delay(ms);
  noTone(BUZZER_PIN);
#else
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
#endif
}

// ---- [F] Sinais ------------------------------------------
// Ao ligar: 1 bip + 1 piscada longa -> LED e buzzer OK
void signalPowerOn() {
  ledOn();
  beep(200);
  delay(300);
  ledOff();
}

// PN532 encontrado: 2 bips curtos
void signalPN532Found() {
  beep(60); delay(80); beep(60);
}

// PN532 NAO encontrado: 3 bips graves
void signalPN532Missing() {
  for (int i = 0; i < 3; i++) { beep(150, BUZZER_LOW_HZ); delay(120); }
}

// Cartao lido: bip + piscada
void signalCardRead() {
  ledOn();
  beep(100);
  delay(100);
  ledOff();
}

// [E] Prende o buzzer em LOW antes do deep sleep (sem isso o pino
// pode flutuar e o buzzer apitar/consumir enquanto o ESP dorme).
void buzzerHoldForSleep() {
  digitalWrite(BUZZER_PIN, LOW);
  gpio_hold_en((gpio_num_t)BUZZER_PIN);
  gpio_deep_sleep_hold_en();
}

String deviceHostname() {
  return "conectabus-tap" + String(TAP_TYPE);   // ex: conectabus-tapin
}

// ============= BUFFER ===============

void bufferAddTap(const String &cardId, uint64_t ts) {
  if (bufferCount >= MAX_BUFFERED_TAPS) {
    Serial.println("Buffer cheio - descartando o mais antigo");
    memmove(&tapBuffer[0], &tapBuffer[1],
            (MAX_BUFFERED_TAPS - 1) * sizeof(TapEntry));
    bufferCount = MAX_BUFFERED_TAPS - 1;
  }

  TapEntry *e = &tapBuffer[bufferCount];
  e->ts_ms = ts;
  strncpy(e->cardId, cardId.c_str(), sizeof(e->cardId) - 1);
  e->cardId[sizeof(e->cardId) - 1] = '\0';
  bufferCount++;

  Serial.printf("Buffer: %u tap(s) armazenado(s)\n", bufferCount);
}

// Decide se ja vale a pena gastar uma sessao WiFi.
bool shouldFlushNow() {
  if (bufferCount == 0) return false;
  if (bufferCount >= FLUSH_THRESHOLD) return true;
  if (timeIsValid()) {
    uint64_t now = epochMillis();
    if (now >= tapBuffer[0].ts_ms &&
        (now - tapBuffer[0].ts_ms) >= (uint64_t)MAX_HOLD_MS) return true;
  }
  return false;
}

// ============= PN532 ===============

bool initPN532() {
  mySerial.begin(115200, SERIAL_8N1, PN532_RX, PN532_TX);
  delay(50);
  while (mySerial.available()) mySerial.read();  // drena lixo

  nfc.begin();
  uint32_t v = nfc.getFirmwareVersion();
  if (!v) {
    Serial.println("PN532 nao encontrado!");
    return false;
  }
  Serial.printf("PN532 OK (fw 0x%08x)\n", v);
  nfc.SAMConfig();
  return true;
}

// Tenta ler o cartao agora (apos wake - cartao ainda presente)
bool readCardNow(String &cardIdOut) {
  uint8_t uid[7] = {0};
  uint8_t uidLen = sizeof(uid);
  nfc.setPassiveActivationRetries(0x05);  // retries curtos
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 800)) {
    cardIdOut = uidToString(uid, uidLen);
    return true;
  }
  return false;
}

// Frame de informacao normal do PN532:
//   00 00 FF LEN LCS <TFI+data...> DCS 00
void sendPN532Frame(const uint8_t *data, uint8_t len) {
  uint8_t hdr[5];
  hdr[0] = 0x00; hdr[1] = 0x00; hdr[2] = 0xFF;
  hdr[3] = len;                       // LEN
  hdr[4] = (uint8_t)(0x100 - len);    // LCS
  mySerial.write(hdr, 5);

  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) { mySerial.write(data[i]); sum += data[i]; }

  uint8_t dcs = (uint8_t)(0x100 - (sum & 0xFF));
  mySerial.write(dcs);
  mySerial.write((uint8_t)0x00);      // postamble
  mySerial.flush();
}

/*
 * Arma o PN532 em AUTO-POLL: pulsa o campo RF a cada
 * (AUTOPOLL_PERIOD * 150ms); entre os pulsos a antena fica desligada.
 * PollNr=0xFF => varre ate achar cartao; ao achar, baixa a IRQ -> ESP acorda.
 * InAutoPoll: TFI=D4, CMD=0x60, PollNr, Period, Type1
 */
void armPN532AutoPoll() {
  uint8_t wake[] = {0x55, 0x55, 0x00, 0x00, 0x00};
  mySerial.write(wake, sizeof(wake));
  delay(15);
  while (mySerial.available()) mySerial.read();

  uint8_t cmd[] = { 0xD4, 0x60, 0xFF, AUTOPOLL_PERIOD, AUTOPOLL_TYPE };
  sendPN532Frame(cmd, sizeof(cmd));
  delay(30);

  // Drena o ACK e garante IRQ desasserida (HIGH) antes de dormir.
  uint32_t t0 = millis();
  while (millis() - t0 < 60) {
    while (mySerial.available()) mySerial.read();
    if (digitalRead(IRQ_PIN) == HIGH) break;
    delay(2);
  }
}

// Modo antigo (fallback): varredura continua, campo ~100% ligado.
void armPN532ContinuousScan() {
  uint8_t wake[] = {0x55, 0x55, 0x00, 0x00, 0x00};
  mySerial.write(wake, sizeof(wake));
  delay(15);
  nfc.setPassiveActivationRetries(0xFF);
  delay(20);
  while (mySerial.available()) mySerial.read();
  uint8_t cmd[] = { 0xD4, 0x4A, 0x01, 0x00 };  // InListPassiveTarget
  sendPN532Frame(cmd, sizeof(cmd));
  delay(30);
  while (mySerial.available()) mySerial.read();
}

void armPN532() {
#if USE_CONTINUOUS_FALLBACK
  armPN532ContinuousScan();
#else
  armPN532AutoPoll();
#endif
}

// ============= NETWORK ===============

bool connectWifi(uint32_t timeout_ms) {
  Serial.printf("WiFi -> %s", WIFI_SSID);
  WiFi.setHostname(deviceHostname().c_str());       // [D] antes do mode/begin
  WiFi.mode(WIFI_STA);
#if USE_STATIC_IP
  WiFi.config(STA_IP, STA_GW, STA_MASK, STA_DNS);
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setTxPower(WIFI_TX_POWER);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > timeout_ms) {
      Serial.println(" TIMEOUT");
      return false;
    }
    delay(200);
    Serial.print(".");
  }
  Serial.printf(" OK %s\n", WiFi.localIP().toString().c_str());
  return true;
}

// [C] Espera a resposta REAL do NTP (nao so "relogio valido").
bool syncTime(uint32_t timeout_ms) {
  sntp_set_sync_status(SNTP_SYNC_STATUS_RESET);
  configTime(0, 0, NTP_SERVER);
  uint32_t t0 = millis();
  while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) {
    if (millis() - t0 > timeout_ms) {
      Serial.println("NTP sem resposta (mantendo hora do RTC)");
      return false;
    }
    delay(100);
  }
  lastNtpSyncMs = epochMillis();
  Serial.println("NTP OK");
  return true;
}

// So sincroniza se a hora nao e valida OU passou NTP_RESYNC_H.
void maybeSyncTime() {
  bool need = !timeIsValid();
  if (!need && lastNtpSyncMs > 0) {
    uint64_t age = epochMillis() - lastNtpSyncMs;
    if (age >= (uint64_t)NTP_RESYNC_H * 3600ULL * 1000ULL) need = true;
  }
  if (need) syncTime(3000);
}

// [A] Resolve TB_HOST -> IP. Ordem: IP literal, DNS normal, mDNS (.local),
// TB_FALLBACK_IP. Guarda o resultado em tbIpCache (RTC memory).
bool resolveTbHost() {
  IPAddress ip;

  // 1) Ja e' um IP escrito direto?
  if (ip.fromString(TB_HOST)) {
    tbIpCache = (uint32_t)ip;
    return true;
  }

  // Separa o nome base (sem ".local") para o mDNS
  char base[64];
  strncpy(base, TB_HOST, sizeof(base) - 1);
  base[sizeof(base) - 1] = '\0';
  size_t n = strlen(base);
  bool hasLocal = (n > 6 && strcasecmp(base + n - 6, ".local") == 0);
  if (hasLocal) base[n - 6] = '\0';

  // 2) DNS normal (roteador conhece o nome). Pula se for ".local".
  if (!hasLocal && WiFi.hostByName(TB_HOST, ip) == 1 && ip != IPAddress(0, 0, 0, 0)) {
    tbIpCache = (uint32_t)ip;
    Serial.printf("Servidor '%s' -> %s (DNS)\n", TB_HOST, ip.toString().c_str());
    return true;
  }

  // 3) mDNS (nome.local)
  if (!mdnsStarted) mdnsStarted = MDNS.begin(deviceHostname().c_str());
  if (mdnsStarted) {
    ip = MDNS.queryHost(base, MDNS_QUERY_TIMEOUT_MS);
    if (ip != IPAddress(0, 0, 0, 0)) {
      tbIpCache = (uint32_t)ip;
      Serial.printf("Servidor '%s.local' -> %s (mDNS)\n", base, ip.toString().c_str());
      return true;
    }
  }

  // 4) IP reserva (opcional)
  if (TB_FALLBACK_IP[0] != '\0' && ip.fromString(TB_FALLBACK_IP)) {
    tbIpCache = (uint32_t)ip;
    Serial.printf("Nome nao resolvido - usando IP reserva %s\n", TB_FALLBACK_IP);
    return true;
  }

  Serial.printf("Nao consegui resolver o nome '%s'\n", TB_HOST);
  return false;
}

// Conecta no MQTT usando o IP em cache. Se a conexao TCP falhar
// (servidor nao esta naquele IP), esquece o cache e resolve o nome de novo.
bool connectMQTT(uint32_t timeout_ms) {
  mqtt.setKeepAlive(60);
  String clientId = "esp32-tap-" + String(TAP_TYPE);
  uint32_t t0 = millis();

  while (true) {
    if (tbIpCache == 0 && !resolveTbHost()) {
      if (millis() - t0 > timeout_ms) {
        Serial.println("MQTT TIMEOUT (servidor nao encontrado pelo nome)");
        return false;
      }
      delay(1000);
      continue;
    }

    mqtt.setServer(IPAddress(tbIpCache), TB_PORT);
    if (mqtt.connect(clientId.c_str(), TB_TOKEN, "")) {
      Serial.printf("MQTT OK (%s)\n", IPAddress(tbIpCache).toString().c_str());
      return true;
    }

    int rc = mqtt.state();
    // -2 = TCP nao conectou, -4 = sem resposta: provavelmente o IP mudou.
    // (4/5 = token recusado: nao adianta resolver o nome de novo)
    if (rc == MQTT_CONNECT_FAILED || rc == MQTT_CONNECTION_TIMEOUT) {
      Serial.printf("MQTT falhou em %s (rc=%d) - vou procurar o nome de novo\n",
                    IPAddress(tbIpCache).toString().c_str(), rc);
      tbIpCache = 0;
    } else {
      Serial.printf("MQTT recusado (rc=%d) - confira o TB_TOKEN\n", rc);
    }

    if (millis() - t0 > timeout_ms) {
      Serial.printf("MQTT TIMEOUT (rc=%d)\n", rc);
      return false;
    }
    delay(500);
  }
}

bool publishOne(const TapEntry *e) {
  char payload[200];
  snprintf(payload, sizeof(payload),
    "{\"ts\":%llu,\"values\":{\"cardId\":\"%s\",\"tapType\":\"%s\"}}",
    (unsigned long long)e->ts_ms, e->cardId, TAP_TYPE);
  bool ok = mqtt.publish("v1/devices/me/telemetry", payload);
  Serial.printf("%s: %s\n", ok ? "Sent" : "FAIL", payload);
  mqtt.loop();
  delay(50);
  return ok;
}

// Telemetria de bateria (opcional) - aproveita WiFi/MQTT ja ligados.
void publishBattery() {
#if BATT_ADC_PIN
  analogReadResolution(12);
  uint32_t acc = 0;
  for (int i = 0; i < 8; i++) { acc += analogReadMilliVolts(BATT_ADC_PIN); delay(2); }
  float vbat = (acc / 8.0f) * BATT_DIVIDER / 1000.0f;
  char payload[96];
  snprintf(payload, sizeof(payload),
    "{\"values\":{\"vbat\":%.3f,\"boots\":%u}}", vbat, bootCount);
  mqtt.publish("v1/devices/me/telemetry", payload);
  mqtt.loop();
  Serial.printf("Bateria: %.3fV\n", vbat);
#endif
}

// Drena o buffer publicando UM por UM. Se falhar no meio, mantem o resto.
void flushBuffer() {
  Serial.printf("Drenando buffer (%u tap[s])\n", bufferCount);
  uint32_t sent = 0;
  for (uint32_t i = 0; i < bufferCount; i++) {
    if (publishOne(&tapBuffer[i])) sent++;
    else break;
  }
  if (sent == bufferCount) {
    bufferCount = 0;
  } else if (sent > 0) {
    memmove(&tapBuffer[0], &tapBuffer[sent],
            (bufferCount - sent) * sizeof(TapEntry));
    bufferCount -= sent;
  }
  tapsTotalSent += sent;
  Serial.printf("Enviei %u tap(s) | buffer agora=%u\n", sent, bufferCount);
}

void networkOff() {
  if (mqtt.connected()) mqtt.disconnect();
  if (mdnsStarted) { MDNS.end(); mdnsStarted = false; }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// ============= SESSAO DE ENVIO ===============

// Sobe WiFi+MQTT UMA vez e drena o buffer inteiro.
void doFlushSession() {
  uint32_t cycleStart = millis();
  uint32_t budget = CONNECT_TOTAL_TIMEOUT_MS;

  if (!connectWifi(budget / 2)) {
    Serial.println("Sem WiFi - taps ficam no buffer");
    networkOff();
    ledOff();
    return;
  }
  blink(120);           // 1 PISCADA: WiFi OK
  maybeSyncTime();

  uint32_t elapsed = millis() - cycleStart;
  uint32_t remaining = (elapsed >= budget) ? 1000 : budget - elapsed;
  if (!connectMQTT(remaining)) {
    Serial.println("Sem MQTT - taps ficam no buffer");
    networkOff();
    ledOff();
    if (millis() - cycleStart > CONNECT_TOTAL_TIMEOUT_MS + 5000) {
      Serial.println("HARD RESET (travado)");
      delay(100);
      ESP.restart();
    }
    return;
  }
  blink(120, 2);        // 2 PISCADAS: MQTT OK

  ledOn();
  flushBuffer();
  publishBattery();
  ledOff();

  networkOff();
}

// ============= TAP CYCLE ===============

void doTapCycle(bool fromCard) {
  if (fromCard) {
    String cardId;
    if (readCardNow(cardId)) {
      Serial.printf("Card: %s\n", cardId.c_str());
      bufferAddTap(cardId, epochMillis());
      signalCardRead();   // [F] bip + piscada
    } else {
      Serial.println("Acordei mas nao consegui ler cartao");
    }
  }

  if (shouldFlushNow()) {
    doFlushSession();
  } else {
    Serial.printf("Buffer=%u abaixo do limite - adiando WiFi\n", bufferCount);
  }
}

// ============= SLEEP ===============

uint64_t computeTimerSleepS() {
  if (bufferCount == 0) return IDLE_HEARTBEAT_S;
  if (!timeIsValid())   return IDLE_HEARTBEAT_S;
  uint64_t now = epochMillis();
  uint64_t age = (now >= tapBuffer[0].ts_ms) ? (now - tapBuffer[0].ts_ms) : 0;
  if (age >= (uint64_t)MAX_HOLD_MS) return 1;
  uint64_t remain_ms = (uint64_t)MAX_HOLD_MS - age;
  uint64_t s = (remain_ms + 999) / 1000;
  if (s < 1) s = 1;
  if (s > IDLE_HEARTBEAT_S) s = IDLE_HEARTBEAT_S;
  return s;
}

void armAndSleep() {
  armPN532();

#if DEBUG_KEEP_AWAKE
  Serial.println("\n[DEBUG] KEEP_AWAKE ligado: NAO vou dormir.");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[DEBUG] Testando WiFi...");
    if (connectWifi(15000)) {
      Serial.printf("[DEBUG] >>> WiFi CONECTADO! IP=%s\n",
                    WiFi.localIP().toString().c_str());
      if (connectMQTT(15000)) Serial.println("[DEBUG] >>> MQTT CONECTADO!");
      else Serial.println("[DEBUG] !!! MQTT FALHOU (confira TB_HOST/avahi/porta/token)");
    } else {
      Serial.println("[DEBUG] !!! WiFi FALHOU (confira SSID/senha/alcance)");
    }
  }
  Serial.println("[DEBUG] Status a cada 3s. (DEBUG_KEEP_AWAKE = 0 no uso real)");
  return;
#endif

  uint64_t timerS = computeTimerSleepS();
  Serial.printf("Deep sleep. Boots=%u Sent=%u Buffer=%u TimerWake=%llus\n",
                bootCount, tapsTotalSent, bufferCount,
                (unsigned long long)timerS);
  Serial.flush();

  buzzerHoldForSleep();   // [E]

  // Wake on IRQ baixando (PN532 IRQ active LOW) - detectou cartao
  esp_deep_sleep_enable_gpio_wakeup(BIT(IRQ_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_sleep_enable_timer_wakeup(timerS * 1000000ULL);
  esp_deep_sleep_start();
}

// ============= MAIN ===============

void setup() {
  Serial.begin(115200);
  delay(300);
  bootCount++;

  pinMode(LED_PIN, OUTPUT);
  ledOff();
  pinMode(IRQ_PIN, INPUT_PULLUP);

  gpio_hold_dis((gpio_num_t)BUZZER_PIN);   // [E] solta o pino travado no sleep
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  setCpuFrequencyMhz(80);
  btStop();

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  bool fromCard = (cause == ESP_SLEEP_WAKEUP_GPIO);
  bool coldBoot = (cause == ESP_SLEEP_WAKEUP_UNDEFINED);   // ligou ou RESET

  if (coldBoot) {
    pn532FailCount = 0;
    signalPowerOn();                  // [F] 1 bip + piscada longa
  }

  if (!initPN532()) {
    pn532FailCount++;
    signalPN532Missing();             // [F] 3 bips graves
    uint64_t retryS = (pn532FailCount <= PN532_FAST_RETRIES) ? 3 : 30;
    Serial.printf("PN532 nao respondeu (%u) - tenta de novo em %llus\n",
                  pn532FailCount, (unsigned long long)retryS);
    buzzerHoldForSleep();   // [E]
    esp_sleep_enable_timer_wakeup(retryS * 1000000ULL);
    esp_deep_sleep_start();
  }

  if (coldBoot || pn532FailCount > 0) signalPN532Found();   // [F] 2 bips curtos
  pn532FailCount = 0;

  Serial.printf("\n=== Boot #%u cause=%d ===\n", bootCount, cause);

  if (firstBoot) {
    Serial.println("PRIMEIRO BOOT - validando rede");
    uint32_t t0 = millis();
    bool wifiOK = connectWifi(15000);
    if (wifiOK) blink(120);

    bool mqttOK = false;
    if (wifiOK) {
      syncTime(5000);                // hora uma vez (RTC mantem no sleep)
      mqttOK = connectMQTT(15000);   // [A] ja resolve e guarda o IP do servidor
      if (mqttOK) blink(120, 2);
    }

    if (wifiOK && mqttOK) {
      Serial.println("Validacao OK");
      ledOn(); delay(600); ledOff();
      firstBoot = false;
    } else {
      Serial.println("Validacao falhou - sera tentada no proximo wake");
      blink(40, 5, 80);
    }

    if (millis() - t0 > CONNECT_TOTAL_TIMEOUT_MS + 10000) {
      Serial.println("HARD RESET (travado no primeiro boot)");
      delay(100);
      ESP.restart();
    }
    networkOff();
  }
  else {
    doTapCycle(fromCard);
  }

  armAndSleep();
}

void loop() {
  // Em producao nunca executa: setup() termina em deep sleep.
#if DEBUG_KEEP_AWAKE
  if (mqtt.connected()) mqtt.loop();

  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    Serial.printf("[DEBUG] WiFi=%s MQTT=%s IP=%s TB=%s | buffer=%u sent=%u ts=%llu\n",
                  (WiFi.status() == WL_CONNECTED) ? "CONECTADO" : "off",
                  mqtt.connected() ? "CONECTADO" : "off",
                  WiFi.localIP().toString().c_str(),
                  IPAddress(tbIpCache).toString().c_str(),
                  bufferCount, tapsTotalSent,
                  (unsigned long long)epochMillis());
  }

  String c;
  if (readCardNow(c)) {
    Serial.printf("[DEBUG] Cartao lido: %s -> bufferizando e publicando\n", c.c_str());
    bufferAddTap(c, epochMillis());
    signalCardRead();   // [F]
    if (!mqtt.connected()) {
      Serial.println("[DEBUG] MQTT caiu, reconectando...");
      if (WiFi.status() != WL_CONNECTED) connectWifi(15000);
      connectMQTT(10000);
    }
    if (mqtt.connected()) flushBuffer();
    else Serial.println("[DEBUG] Sem MQTT - tap ficou no buffer");
    delay(1500);   // debounce simples
  }
#endif
}
