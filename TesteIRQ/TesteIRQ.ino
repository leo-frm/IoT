/*
 * ============================================================
 *  TESTE DO IRQ DO PN532 (HSU) - ESP32-C3 Super Mini
 *  Sem WiFi e sem deep sleep. Serve para descobrir se o pino IRQ
 *  do PN532 cai (LOW) quando um cartao e' detectado em modo HSU.
 *
 *  Ciclo:
 *   1. Envia InAutoPoll (ou InListPassiveTarget, ver USE_AUTOPOLL)
 *      com a mesma sendPN532Frame da V4. Le SO o ACK (6 bytes).
 *   2. Monitora o GPIO 2 (IRQ):
 *        - amostra a cada SAMPLE_MS e mostra no Serial
 *        - interrupcao em CHANGE registra TODA borda com timestamp
 *          em microssegundos (pega pulsos curtos que a amostragem perderia)
 *   3. Guarda (sem interpretar como a biblioteca) os bytes que chegam
 *      pela serial do PN532.
 *   4. Relatorio quando:
 *        - IRQ cai            -> bip agudo + piscada
 *        - bytes chegam SEM o IRQ cair -> 2 bips graves
 *        - CYCLE_TIMEOUT_MS sem nada   -> so o relatorio
 *      e re-arma o comando depois de REARM_DELAY_MS.
 *
 *  Arduino IDE: "ESP32C3 Dev Module", USB CDC On Boot = Enabled,
 *  Serial Monitor em 115200.
 * ============================================================
 */

#include <PN532_HSU.h>
#include <PN532.h>
#include <driver/gpio.h>

// ===================== AJUSTE AQUI =========================
#define LED_PIN     8     // LED onboard (active LOW)
#define BUZZER_PIN  5     // buzzer piezo
#define IRQ_PIN     2     // IRQ do PN532
#define PN532_RX    20    // ESP RX  <- TX do PN532
#define PN532_TX    21    // ESP TX  -> RX do PN532

#define BUZZER_PASSIVE  1
#define BUZZER_FREQ_HZ  2700
#define BUZZER_LOW_HZ   800

// 1 = InAutoPoll (0x60), como a V4
// 0 = InListPassiveTarget (0x4A, 0x01, 0x00), como o USE_CONTINUOUS_FALLBACK
#define USE_AUTOPOLL      1
#define AUTOPOLL_PERIOD   0x02    // 300 ms
#define AUTOPOLL_TYPE     0x00    // ISO14443A 106 kbps

#define IRQ_INTERNAL_PULLUP 1     // 1 = INPUT_PULLUP no GPIO 2 (igual a V4)

#define SAMPLE_MS          50     // amostragem do IRQ
#define PRINT_EVERY_SAMPLE 1      // 1 = imprime o nivel a cada amostra; 0 = so mudancas
#define CYCLE_TIMEOUT_MS   8000   // sem nada nesse tempo: relatorio + re-arma
#define POST_IRQ_WAIT_MS   60     // depois do IRQ cair, espera os bytes chegarem
#define QUIET_MS           150    // bytes sem IRQ: considera a resposta completa apos esse silencio
#define REARM_DELAY_MS     1500   // pausa antes de re-armar (tempo de tirar o cartao)
// ===========================================================

HardwareSerial mySerial(1);
PN532_HSU pn532hsu(mySerial);
PN532 nfc(pn532hsu);

// ---------- Estado ----------
enum Phase : uint8_t { PH_IDLE = 0, PH_ACK = 1, PH_RESP = 2, PH_WAIT = 3 };
volatile uint8_t phase = PH_IDLE;
const char* phaseName(uint8_t p) {
  switch (p) {
    case PH_ACK:  return "ACK";
    case PH_RESP: return "RESP";
    case PH_WAIT: return "PAUSA";
    default:      return "IDLE";
  }
}

uint32_t armedAtMs = 0;
uint32_t rearmAtMs = 0;
uint32_t cycleNo   = 0;

// ---------- Log de bordas do IRQ (preenchido na interrupcao) ----------
#define EDGE_LOG 64
volatile uint32_t edgeUs[EDGE_LOG];
volatile uint8_t  edgeLvl[EDGE_LOG];
volatile uint8_t  edgePh[EDGE_LOG];
volatile uint8_t  edgeHead = 0, edgeTail = 0;
volatile uint32_t edgeDropped = 0;

volatile bool     irqFellInResp = false;   // IRQ caiu na fase de resposta
volatile uint32_t irqFellUs     = 0;

void IRAM_ATTR irqIsr() {
  uint8_t lvl = gpio_get_level((gpio_num_t)IRQ_PIN);
  uint32_t t  = micros();
  uint8_t h = edgeHead;
  uint8_t n = (h + 1) % EDGE_LOG;
  if (n != edgeTail) {
    edgeUs[h] = t; edgeLvl[h] = lvl; edgePh[h] = phase;
    edgeHead = n;
  } else {
    edgeDropped++;
  }
  if (lvl == 0 && phase == PH_RESP && !irqFellInResp) {
    irqFellInResp = true;
    irqFellUs = t;
  }
}

// ---------- Bytes recebidos do PN532 ----------
uint8_t  rx[256];
int      rxLen = 0;
uint32_t firstByteMs = 0, lastByteMs = 0;

void pumpRx() {
  while (mySerial.available()) {
    int b = mySerial.read();
    if (rxLen == 0) firstByteMs = millis();
    lastByteMs = millis();
    if (rxLen < (int)sizeof(rx)) rx[rxLen++] = (uint8_t)b;
  }
}

// ---------- LED / buzzer (como na V4) ----------
void ledOn()  { digitalWrite(LED_PIN, LOW);  }
void ledOff() { digitalWrite(LED_PIN, HIGH); }

void beep(int ms, int freq = BUZZER_FREQ_HZ) {
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

void signalIrqFell() {          // bip + piscada
  ledOn(); beep(100); delay(100); ledOff();
}

void signalBytesWithoutIrq() {  // 2 bips graves
  beep(120, BUZZER_LOW_HZ); delay(100); beep(120, BUZZER_LOW_HZ);
}

// ---------- PN532 ----------
// Mesma funcao da V4: 00 00 FF LEN LCS <TFI+data...> DCS 00
void sendPN532Frame(const uint8_t *data, uint8_t len) {
  uint8_t hdr[5];
  hdr[0] = 0x00; hdr[1] = 0x00; hdr[2] = 0xFF;
  hdr[3] = len;
  hdr[4] = (uint8_t)(0x100 - len);
  mySerial.write(hdr, 5);

  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) { mySerial.write(data[i]); sum += data[i]; }

  uint8_t dcs = (uint8_t)(0x100 - (sum & 0xFF));
  mySerial.write(dcs);
  mySerial.write((uint8_t)0x00);
  mySerial.flush();
}

void printHex(const uint8_t* d, int n) {
  for (int i = 0; i < n; i++) Serial.printf("%02X ", d[i]);
}

// Procura a resposta (D5 61 = InAutoPoll, D5 4B = InListPassiveTarget) e extrai o UID.
bool parseUid(String &uidOut) {
  for (int i = 0; i + 1 < rxLen; i++) {
    if (rx[i] != 0xD5 || (rx[i + 1] != 0x61 && rx[i + 1] != 0x4B)) continue;
    bool autopoll = (rx[i + 1] == 0x61);
    int p = i + 2;
    if (p >= rxLen) return false;
    uint8_t nbTg = rx[p++];
    if (nbTg == 0) return false;
    if (autopoll) p += 2;      // Type1 + AutoPollTargetDataLen
    p += 1;                    // Tg
    p += 3;                    // SENS_RES (2) + SEL_RES (1)
    if (p >= rxLen) return false;
    uint8_t uidLen = rx[p++];
    if (uidLen == 0 || uidLen > 10 || p + uidLen > rxLen) return false;
    uidOut = "";
    for (int k = 0; k < uidLen; k++) {
      if (rx[p + k] < 0x10) uidOut += "0";
      uidOut += String(rx[p + k], HEX);
    }
    uidOut.toUpperCase();
    return true;
  }
  return false;
}

void printEdges() {
  while (edgeTail != edgeHead) {
    uint8_t t = edgeTail;
    Serial.printf("    [borda] t=%lu us (%lu ms)  IRQ -> %s  (fase %s)\n",
                  (unsigned long)edgeUs[t], (unsigned long)(edgeUs[t] / 1000),
                  edgeLvl[t] ? "HIGH" : "LOW", phaseName(edgePh[t]));
    edgeTail = (t + 1) % EDGE_LOG;
  }
  if (edgeDropped) {
    Serial.printf("    [borda] %lu borda(s) perdidas (log cheio)\n", (unsigned long)edgeDropped);
    edgeDropped = 0;
  }
}

void armCommand() {
  cycleNo++;
  Serial.printf("\n========== CICLO %lu - %s ==========\n", (unsigned long)cycleNo,
                USE_AUTOPOLL ? "InAutoPoll (0x60)" : "InListPassiveTarget (0x4A)");

  // Wake-up do HSU (igual a V4) e descarta o que tiver no buffer
  uint8_t wake[] = {0x55, 0x55, 0x00, 0x00, 0x00};
  mySerial.write(wake, sizeof(wake));
  mySerial.flush();
  delay(15);
  while (mySerial.available()) mySerial.read();

#if USE_AUTOPOLL
  uint8_t cmd[] = { 0xD4, 0x60, 0xFF, AUTOPOLL_PERIOD, AUTOPOLL_TYPE };
#else
  uint8_t cmd[] = { 0xD4, 0x4A, 0x01, 0x00 };
#endif

  Serial.printf("IRQ antes do comando: %s\n", digitalRead(IRQ_PIN) ? "HIGH" : "LOW");
  phase = PH_ACK;
  irqFellInResp = false;
  uint32_t t0 = millis();
  sendPN532Frame(cmd, sizeof(cmd));
  Serial.print("Comando enviado: ");
  printHex(cmd, sizeof(cmd));
  Serial.println();

  // Le SO o ACK: 6 bytes ou 100 ms
  uint8_t ack[6];
  int n = 0;
  while (n < 6 && millis() - t0 < 100) {
    if (mySerial.available()) ack[n++] = (uint8_t)mySerial.read();
  }
  static const uint8_t ACK_OK[6] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
  bool ackOk = (n == 6 && memcmp(ack, ACK_OK, 6) == 0);
  Serial.printf("ACK (%d bytes em %lu ms): ", n, (unsigned long)(millis() - t0));
  printHex(ack, n);
  Serial.println(ackOk ? "-> OK" : "-> INESPERADO");

  // Espera o IRQ ficar HIGH (ate 100 ms), como a V4 faz antes de dormir
  uint32_t t1 = millis();
  while (digitalRead(IRQ_PIN) == LOW && millis() - t1 < 100) delay(1);
  Serial.printf("IRQ depois do ACK: %s\n", digitalRead(IRQ_PIN) ? "HIGH" : "LOW (nao subiu em 100 ms)");
  printEdges();

  rxLen = 0;
  irqFellInResp = false;
  armedAtMs = millis();
  phase = PH_RESP;
  Serial.println(">>> Armado. Encoste o cartao.");
}

void report(const char* why) {
  pumpRx();
  printEdges();
  Serial.println("------------------------------------------");
  Serial.printf("RESULTADO: %s\n", why);
  Serial.printf("  Armado ha %lu ms\n", (unsigned long)(millis() - armedAtMs));
  if (irqFellInResp) {
    Serial.printf("  IRQ caiu em t=%lu ms (%lu ms apos armar)\n",
                  (unsigned long)(irqFellUs / 1000),
                  (unsigned long)(irqFellUs / 1000 - armedAtMs));
  } else {
    Serial.println("  IRQ NAO caiu");
  }
  if (rxLen > 0) {
    Serial.printf("  %d byte(s) recebidos, 1o byte %lu ms apos armar:\n  ",
                  rxLen, (unsigned long)(firstByteMs - armedAtMs));
    printHex(rx, rxLen);
    Serial.println();
    String uid;
    if (parseUid(uid)) Serial.printf("  UID: %s\n", uid.c_str());
    else               Serial.println("  (nao achei UID nos bytes)");
  } else {
    Serial.println("  Nenhum byte recebido");
  }
  Serial.printf("  IRQ agora: %s\n", digitalRead(IRQ_PIN) ? "HIGH" : "LOW");
  Serial.println("------------------------------------------");
  rxLen = 0;   // o que chegar daqui pra frente e' da pausa
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("\n=== TESTE IRQ PN532 (HSU) ===");

  pinMode(LED_PIN, OUTPUT);
  ledOff();
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
#if IRQ_INTERNAL_PULLUP
  pinMode(IRQ_PIN, INPUT_PULLUP);
#else
  pinMode(IRQ_PIN, INPUT);
#endif
  Serial.printf("IRQ (GPIO %d) no boot: %s\n", IRQ_PIN, digitalRead(IRQ_PIN) ? "HIGH" : "LOW");

  mySerial.begin(115200, SERIAL_8N1, PN532_RX, PN532_TX);
  delay(50);
  while (mySerial.available()) mySerial.read();

  nfc.begin();
  uint32_t v = nfc.getFirmwareVersion();
  while (!v) {
    Serial.println("PN532 NAO encontrado - nova tentativa em 3s");
    for (int i = 0; i < 3; i++) { beep(150, BUZZER_LOW_HZ); delay(120); }
    delay(3000);
    v = nfc.getFirmwareVersion();
  }
  Serial.printf("PN532 OK - chip PN5%X, firmware %d.%d\n",
                (v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF);
  nfc.SAMConfig();
  Serial.printf("IRQ apos SAMConfig: %s\n", digitalRead(IRQ_PIN) ? "HIGH" : "LOW");
  beep(60); delay(80); beep(60);

  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), irqIsr, CHANGE);

  armCommand();
}

void loop() {
  static uint32_t lastSample = 0;
  static int lastLvl = -1;

  pumpRx();

  // Amostragem do IRQ
  if (millis() - lastSample >= SAMPLE_MS) {
    lastSample = millis();
    int lvl = digitalRead(IRQ_PIN);
#if PRINT_EVERY_SAMPLE
    Serial.printf("%8lu ms  IRQ=%d  bytes=%d  [%s]\n",
                  (unsigned long)millis(), lvl, rxLen, phaseName(phase));
#endif
    if (lastLvl != -1 && lvl != lastLvl) {
      Serial.printf(">>> %lu ms: IRQ mudou %s -> %s\n", (unsigned long)millis(),
                    lastLvl ? "HIGH" : "LOW", lvl ? "HIGH" : "LOW");
    }
    lastLvl = lvl;
  }

  printEdges();

  if (phase == PH_RESP) {
    if (irqFellInResp) {
      uint32_t t = millis();
      while (millis() - t < POST_IRQ_WAIT_MS) { pumpRx(); delay(1); }
      report("IRQ CAIU");
      signalIrqFell();
      phase = PH_WAIT;
      rearmAtMs = millis() + REARM_DELAY_MS;
    } else if (rxLen > 0 && millis() - lastByteMs > QUIET_MS) {
      report("BYTES CHEGARAM SEM O IRQ CAIR");
      signalBytesWithoutIrq();
      phase = PH_WAIT;
      rearmAtMs = millis() + REARM_DELAY_MS;
    } else if (millis() - armedAtMs > CYCLE_TIMEOUT_MS) {
      report("TIMEOUT (nenhum cartao / nada aconteceu)");
      phase = PH_WAIT;
      rearmAtMs = millis() + 200;
    }
  } else if (phase == PH_WAIT && (int32_t)(millis() - rearmAtMs) >= 0) {
    // descarta o que sobrou antes de re-armar
    pumpRx();
    if (rxLen > 0) {
      Serial.printf("(descartando %d byte(s) recebidos na pausa: ", rxLen);
      printHex(rx, rxLen);
      Serial.println(")");
    }
    rxLen = 0;
    armCommand();
  }
}
