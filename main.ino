/*
 * SantaFe ECU TCI-D CAN Reader  -  BAN GAN TREN XE (khong can go lenh)
 * Doc ECU Hyundai Santa Fe TCI-D bang Arduino Uno + MCP2515
 * Hien thi CRANK / CAM / BOOST / RAIL len 4 man hinh LCD I2C
 *
 * Noi day MCP2515 <-> Arduino Uno:
 *   VCC -> 5V      GND -> GND
 *   CS  -> D10     SO  -> D12 (MISO)
 *   SI  -> D11     SCK -> D13
 * CAN: CANH -> ECU CAN-H, CANL -> ECU CAN-L (hoac OBD pin 6 / pin 14)
 *      GND chung (OBD pin 4/5)
 *
 * 4 LCD I2C (module PCF8574) dung chung 1 bus:
 *   VCC -> 5V   GND -> GND   SDA -> A4   SCL -> A5
 *
 *   LCD     Dia chi   Hien thi
 *   LCD 1   0x27      CRANK (RPM truc khuyu)
 *   LCD 2   0x26      CAM   (RPM truc cam, suy ra)
 *   LCD 3   0x25      BOOST (bar)
 *   LCD 4   0x24      RAIL  (bar)
 *   (Sua 4 dong LCD_ADDR_* ben duoi neu dia chi that cua ban khac)
 *
 * CHAY TU DONG: bat nguon la tu dong do toc do CAN, doc ECU va cap nhat LCD.
 * Khong doc lenh tu Serial (tranh nhieu tren chan RX lam doi che do).
 *
 * CHONG SO 0:
 *   Neu gia tri moi = 0 (hoac khong doc duoc) ma truoc do da co gia tri hop le,
 *   LCD giu gia tri cu toi da HOLD_MS. Qua thoi gian do van la 0 thi moi hien 0.
 *   Gia tri dang giu (khong con moi) co dau '*' o cuoi dong.
 *
 * CAM: khong doc tu cam bien that. Gia tri CAM duoc SUY RA tu RPM khuyu.
 * 2 vong cam = 1 vong khuyu  =>  cam quay nhanh gap doi khuyu  =>  CRANK_PER_CAM = 0.5
 */

#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <mcp_can.h>
#include <LiquidCrystal_I2C.h>   // thu vien "LiquidCrystal I2C" (Frank de Brabander)

// ---------------- Cau hinh ----------------
#define CAN_CS_PIN 10
#define MCP_CLOCK  MCP_8MHZ     // doi thanh MCP_16MHZ neu thach anh la 16MHz

// He so RPM cua khung 0x316 (byte 3 cao, byte 2 thap).
// 0.25 suy ra tu doi chieu voi PID 0C; can xac nhan bang toc do that.
#define RPM_SCALE 0.25

// Ti so khuyu : cam. 2 vong cam = 1 vong khuyu => 0.5
// Neu thuc te cam quay cham hon khuyu thi doi thanh 2.0
#define CRANK_PER_CAM 0.5

#define LCD_COLS 16             // LCD 16x2. Neu dung 20x4 thi doi thanh 20 va 4
#define LCD_ROWS 2

#define LCD_ADDR_CRANK 0x27
#define LCD_ADDR_CAM   0x26
#define LCD_ADDR_BOOST 0x25
#define LCD_ADDR_RAIL  0x24

#define HOLD_MS     8000UL      // giu gia tri cu toi da bay nhieu ms khi gia tri moi = 0 / mat du lieu
#define FRESH_MS    2200UL      // gia tri cu hon moc nay thi coi la "dang giu" (hien dau *)
#define POLL_MS     1000UL      // chu ky hoi ECU (PID)
#define LCD_MS      500UL       // chu ky cap nhat LCD
#define LCD_REINIT_MS 60000UL   // khoi dong lai LCD dinh ky (chong nhieu tren xe)
#define NO_FRAME_REDETECT_MS 30000UL  // het khung CAN bay lau thi do lai toc do
#define SHOW_HOLD_MARK 1        // 1 = hien dau * khi dang giu gia tri cu
#define USE_WATCHDOG 1          // 1 = tu reset neu treo. Dat 0 neu bootloader khong ho tro watchdog

// ---------------- Kieu du lieu ----------------
// PHAI dat truoc ham dau tien cua file, neu khong Arduino IDE se bao loi "Chan was not declared"
enum { R_NONE = 0, R_ZERO = 1, R_GOOD = 2 };   // khong co du lieu / bang 0 / gia tri hop le

struct Chan {
  float val;                  // gia tri hop le gan nhat
  unsigned long goodMs;       // thoi diem nhan gia tri hop le gan nhat
  byte last;                  // trang thai lan doc gan nhat
  bool seen;                  // da tung co gia tri hop le
};
Chan chCrank, chCam, chBoost, chRail;

#if USE_WATCHDOG
  #define WDT_KICK() wdt_reset()
  // Tat watchdog som sau khi reset, tranh vong lap reset
  void wdtEarlyOff(void) __attribute__((naked)) __attribute__((section(".init3"))) __attribute__((used));
  void wdtEarlyOff(void) { MCUSR = 0; wdt_disable(); }
#else
  #define WDT_KICK()
#endif

// ---------------- Doi tuong ----------------
LiquidCrystal_I2C lcdCrank(LCD_ADDR_CRANK, LCD_COLS, LCD_ROWS);   // LCD 1
LiquidCrystal_I2C lcdCam  (LCD_ADDR_CAM,   LCD_COLS, LCD_ROWS);   // LCD 2
LiquidCrystal_I2C lcdBoost(LCD_ADDR_BOOST, LCD_COLS, LCD_ROWS);   // LCD 3
LiquidCrystal_I2C lcdRail (LCD_ADDR_RAIL,  LCD_COLS, LCD_ROWS);   // LCD 4
LiquidCrystal_I2C *lcdAll[4] = { &lcdCrank, &lcdCam, &lcdBoost, &lcdRail };
const char *lcdTitle[4] = { "CRANK (khuyu)", "CAM (suy ra)", "BOOST", "RAIL" };
char lastV[4][LCD_COLS + 1];     // noi dung dong 2 da ghi, de khong ghi lai neu khong doi

MCP_CAN CAN(CAN_CS_PIN);

const uint8_t speedCodes[] = { CAN_500KBPS, CAN_250KBPS, CAN_125KBPS };
const char *speedNames[]   = { "500 kbps", "250 kbps", "125 kbps" };
const uint8_t NUM_SPEEDS   = 3;

int  activeSpeed = -1;
unsigned long badFrames = 0;
unsigned long frameCount = 0;
static byte bigBuf[16];

// Bang PID Mode 01 ma ECU ho tro (PID 0x01..0xE0)
byte pidMap[28];
bool pidScanned = false;

// ---------------- Kenh du lieu co "giu gia tri cu" ----------------
unsigned long lastCrankMs = 0;   // thoi diem khung 0x316 gan nhat
int lastBaro = 101;              // ap suat khi quyen hop le gan nhat (kPa)

void chanFeed(Chan &c, byte st, float v) {
  c.last = st;
  if (st == R_GOOD) { c.val = v; c.goodMs = millis(); c.seen = true; }
}

// Lay gia tri de hien thi. Tra ve false neu khong co gi de hien (hien "--").
// held = true neu dang hien gia tri cu (khong con moi).
bool chanGet(Chan &c, float &out, bool &held) {
  unsigned long age = millis() - c.goodMs;
  if (c.seen && age <= HOLD_MS) {          // con trong thoi gian giu: dung gia tri cu
    out = c.val;
    held = (age > FRESH_MS);
    return true;
  }
  if (c.last == R_ZERO) {                  // 0 keo dai qua HOLD_MS: 0 that, cho hien
    out = 0;
    held = false;
    return true;
  }
  return false;                            // mat du lieu qua lau
}

// ---------------- Tien ich ----------------
void printHexByte(byte b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

// Goi moi khi nhan khung 0x316: cap nhat CRANK va CAM suy ra
void updateCam(const byte *d) {
  unsigned int raw = (unsigned int)d[3] * 256 + d[2];
  float rpm = raw * RPM_SCALE;
  byte st = raw ? R_GOOD : R_ZERO;
  chanFeed(chCrank, st, rpm);
  chanFeed(chCam,   st, rpm / CRANK_PER_CAM);
  lastCrankMs = millis();
}

// ---------------- LCD I2C ----------------
// In 1 dong len LCD, tu dong chen khoang trang cho du do rong
void lcdLine(LiquidCrystal_I2C &l, byte row, const char *s) {
  char buf[LCD_COLS + 1];
  byte i = 0;
  while (i < LCD_COLS && s[i]) { buf[i] = s[i]; i++; }
  while (i < LCD_COLS) buf[i++] = ' ';
  buf[LCD_COLS] = 0;
  l.setCursor(0, row);
  l.print(buf);
}

// Ghi dong gia tri (dong 2) cua LCD thu idx; chi ghi khi noi dung doi
void lcdValue(byte idx, const char *s, bool held) {
  char buf[LCD_COLS + 1];
  byte i = 0;
  while (i < LCD_COLS && s[i]) { buf[i] = s[i]; i++; }
  while (i < LCD_COLS) buf[i++] = ' ';
  buf[LCD_COLS] = 0;
#if SHOW_HOLD_MARK
  if (held) buf[LCD_COLS - 1] = '*';
#endif
  if (strcmp(buf, lastV[idx]) == 0) return;
  strcpy(lastV[idx], buf);
  lcdAll[idx]->setCursor(0, 1);
  lcdAll[idx]->print(buf);
}

// Ghi lai tieu de (dong 1) cho 4 LCD va xoa bo nho dem dong 2
void lcdTitles() {
  for (byte i = 0; i < 4; i++) {
    lcdLine(*lcdAll[i], 0, lcdTitle[i]);
    lastV[i][0] = 0;
  }
}

void lcdInitAll() {
  for (byte i = 0; i < 4; i++) {
    lcdAll[i]->init();
    lcdAll[i]->backlight();
    lcdAll[i]->clear();
    lcdLine(*lcdAll[i], 0, "SantaFe TCI-D");
    lcdLine(*lcdAll[i], 1, "Dang khoi dong");
    lastV[i][0] = 0;
    WDT_KICK();
  }
}

// Khoi dong lai LCD dinh ky (nhieu dien tren xe co the lam LCD loi hien thi)
void lcdReinit() {
  for (byte i = 0; i < 4; i++) {
    lcdAll[i]->init();
    lcdAll[i]->backlight();
    WDT_KICK();
  }
  lcdTitles();
}

void showChan(byte idx, Chan &c, byte decimals, const char *unit) {
  float v;
  bool held = false;
  char num[14];
  char line[24];
  if (chanGet(c, v, held)) {
    dtostrf(v, 1, decimals, num);
    snprintf(line, sizeof(line), "%s %s", num, unit);
  } else {
    snprintf(line, sizeof(line), "-- %s", unit);
  }
  lcdValue(idx, line, held);
}

void showLcd() {
  showChan(0, chCrank, 0, "rpm");
  showChan(1, chCam,   0, "rpm");
  showChan(2, chBoost, 2, "bar");
  showChan(3, chRail,  1, "bar");
}

// Quet I2C va in ra Serial (chi de kiem tra khi cam may tinh luc khoi dong)
void i2cScan() {
  Serial.print(F("I2C:"));
  byte found = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F(" 0x"));
      printHexByte(a);
      found++;
    }
    WDT_KICK();
  }
  Serial.print(F("  ("));
  Serial.print(found);
  Serial.println(F(" thiet bi)"));
}

// ---------------- CAN ----------------
bool readFrame(unsigned long &id, bool &ext, byte &len, byte *buf) {
  if (CAN.checkReceive() != CAN_MSGAVAIL) return false;
  CAN.readMsgBuf(&id, &len, bigBuf);
  ext = (id & 0x80000000UL) == 0x80000000UL;
  id &= 0x1FFFFFFFUL;
  if (id > 0x7FF) ext = true;            // ID vuot 11 bit thi chac chan la khung mo rong
  if (len > 8 || id == 0x1FFFFFFFUL) {   // du lieu rac
    badFrames++;
    return false;
  }
  frameCount++;
  memcpy(buf, bigBuf, len);

  if (!ext && len == 8 && id == 0x316) updateCam(bigBuf);   // CRANK + CAM
  return true;
}

// Doc het cac khung dang cho (de CRANK/CAM luon moi)
void pumpCan() {
  unsigned long id; bool ext; byte len; byte buf[8];
  for (byte n = 0; n < 16 && readFrame(id, ext, len, buf); n++) {}
}

void normalOn() { CAN.setMode(MCP_NORMAL); }

int detectSpeed(uint8_t canMode) {
  for (uint8_t i = 0; i < NUM_SPEEDS; i++) {
    if (CAN.begin(MCP_ANY, speedCodes[i], MCP_CLOCK) != CAN_OK) {
      Serial.println(F("Loi khoi tao MCP2515. Kiem tra day SPI, nguon 5V, thach anh."));
      return -2;
    }
    CAN.setMode(canMode);

    Serial.print(F("Thu "));
    Serial.print(speedNames[i]);
    Serial.println(canMode == MCP_LISTENONLY ? F(" (chi nghe)...") : F(" (normal, co ACK)..."));

    unsigned long t0 = millis();
    unsigned long id; bool ext; byte len; byte buf[8];
    int count = 0;
    badFrames = 0;
    while (millis() - t0 < 3000) {
      WDT_KICK();
      if (readFrame(id, ext, len, buf) && len >= 1) count++;
    }
    if (count == 0 && badFrames > 0) {
      Serial.println(F("LOI PHAN CUNG: MCP2515 tra ve du lieu rac."));
      return -2;
    }
    if (count >= 3) {
      Serial.print(F("=> Thay "));
      Serial.print(count);
      Serial.print(F(" khung o toc do "));
      Serial.println(speedNames[i]);
      return i;
    }
  }
  return -1;
}

void runDetection() {
  activeSpeed = detectSpeed(MCP_LISTENONLY);
  if (activeSpeed == -1) {
    Serial.println(F("Khong thay gi o che do chi nghe. Thu lai o che do NORMAL (co ACK)..."));
    activeSpeed = detectSpeed(MCP_NORMAL);
  }
  if (activeSpeed >= 0) {
    normalOn();
  } else if (activeSpeed == -1) {
    Serial.println(F("Khong thay ECU phat khung nao. Dung tam 500 kbps, se do lai sau."));
    CAN.begin(MCP_ANY, CAN_500KBPS, MCP_CLOCK);
    normalOn();
    activeSpeed = 0;
  }
}

// ---------------- OBD-II (ISO 15765) ----------------
// Gui yeu cau (1 khung), nhan phan hoi 1 khung hoac nhieu khung.
// Tra ve so byte payload (0 = khong phan hoi). out[0] la ma dich vu phan hoi.
int isoRequest(unsigned long txId, const byte *req, byte reqLen, byte *out, int maxLen) {
  byte f[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  if (reqLen > 7) return 0;
  f[0] = reqLen;
  memcpy(f + 1, req, reqLen);

  unsigned long id; bool ext; byte len; byte buf[8];
  for (byte i = 0; i < 30 && readFrame(id, ext, len, buf); i++) {}   // bo khung cu

  if (CAN.sendMsgBuf(txId, 0, 8, f) != CAN_OK) return 0;

  unsigned long t0 = millis();
  unsigned long limit = 400;
  unsigned long respId = 0;
  int total = 0, recv = 0;

  while (millis() - t0 < limit) {
    WDT_KICK();
    if (!readFrame(id, ext, len, buf)) continue;
    if (ext || id < 0x7E8 || id > 0x7EF) continue;   // bo qua khung phat dinh ky
    byte type = buf[0] >> 4;

    if (respId == 0) {
      if (type == 0) {                                  // single frame
        int n = buf[0] & 0x0F;
        if (n > 7) n = 7;
        if (n >= 3 && buf[1] == 0x7F && buf[3] == 0x78) { // "dang xu ly", cho them
          t0 = millis();
          limit = 2000;
          continue;
        }
        if (n > maxLen) n = maxLen;
        memcpy(out, buf + 1, n);
        return n;
      }
      if (type == 1) {                                  // first frame
        respId = id;
        total = ((buf[0] & 0x0F) << 8) | buf[1];
        for (byte i = 0; i < 6 && recv < total; i++, recv++)
          if (recv < maxLen) out[recv] = buf[2 + i];
        byte fc[8] = { 0x30, 0x00, 0x00, 0, 0, 0, 0, 0 };   // flow control
        CAN.sendMsgBuf(txId == 0x7DF ? (respId - 8) : txId, 0, 8, fc);
        if (recv >= total) return min(total, maxLen);
      }
    } else if (id == respId && type == 2) {             // consecutive frame
      for (byte i = 0; i < 7 && recv < total; i++, recv++)
        if (recv < maxLen) out[recv] = buf[1 + i];
      if (recv >= total) return min(total, maxLen);
    }
  }
  return (recv > 0) ? min(recv, maxLen) : 0;
}

int obdRequest(byte service, int pid, byte *out, int maxLen) {
  byte q[2];
  byte ql = 1;
  q[0] = service;
  if (pid >= 0) { q[1] = (byte)pid; ql = 2; }
  return isoRequest(0x7DF, q, ql, out, maxLen);
}

// Doc 1 PID Mode 01, tra ve 2 byte du lieu
bool readPid(byte pid, byte &a, byte &b) {
  byte r[16];
  int n = obdRequest(0x01, pid, r, 16);
  if (n < 3 || r[0] != 0x41 || r[1] != pid) return false;
  a = r[2];
  b = (n > 3) ? r[3] : 0;
  return true;
}

// PID co duoc ECU bao ho tro khong (dua tren ket qua quet PID)
bool isSupported(byte pid) {
  if (!pidScanned) return true;      // chua quet thi cu thu
  if (pid == 0) return true;
  byte idx = pid - 1;
  return (pidMap[idx >> 3] & (0x80 >> (idx & 7))) != 0;
}

bool getPid(byte pid, byte &a, byte &b) {
  return isSupported(pid) && readPid(pid, a, b);
}

void scanPids() {
  normalOn();
  memset(pidMap, 0, sizeof(pidMap));
  pidScanned = false;
  Serial.println(F("--- PID Mode 01 duoc ECU ho tro ---"));
  for (byte base = 0; base <= 0xC0; base += 0x20) {
    byte r[16];
    int n = obdRequest(0x01, base, r, 16);
    if (n < 6 || r[0] != 0x41 || r[1] != base) {
      if (base == 0) Serial.println(F("ECU khong tra loi Mode 01 (se thu lai sau)."));
      break;
    }
    if (base == 0) pidScanned = true;
    for (byte j = 0; j < 4; j++) pidMap[(base >> 3) + j] = r[2 + j];
    for (byte i = 0; i < 32; i++) {
      if (r[2 + (i >> 3)] & (0x80 >> (i & 7))) {
        Serial.print(F(" 0x"));
        printHexByte(base + 1 + i);
      }
    }
    Serial.println();
    if (!(r[5] & 0x01)) break;
  }
}

// ---------------- Doc BOOST, RAIL (va CRANK du phong) ----------------
void pollObd() {
  byte a, b;

  // CRANK du phong: neu khong co khung 0x316 thi doc PID 0C (chuan /4)
  if (millis() - lastCrankMs > 1000UL) {
    if (getPid(0x0C, a, b)) {
      unsigned int raw = (unsigned int)a * 256 + b;
      float rpm = raw / 4.0;
      byte st = raw ? R_GOOD : R_ZERO;
      chanFeed(chCrank, st, rpm);
      chanFeed(chCam,   st, rpm / CRANK_PER_CAM);
    } else {
      chanFeed(chCrank, R_NONE, 0);
      chanFeed(chCam,   R_NONE, 0);
    }
  }
  WDT_KICK();

  // BOOST = MAP - khi quyen. Khi quyen it doi nen chi doc 10 lan/1 lan
  static byte baroTick = 0;
  if (baroTick == 0) {
    if (getPid(0x33, a, b) && a >= 80 && a <= 110) lastBaro = a;
  }
  baroTick = (baroTick + 1) % 10;

  if (getPid(0x0B, a, b)) {
    if (a == 0)        chanFeed(chBoost, R_ZERO, 0);     // MAP = 0: khong hop le
    else if (a == 255) chanFeed(chBoost, R_NONE, 0);     // MAP = 255: khong hop le
    else               chanFeed(chBoost, R_GOOD, ((int)a - lastBaro) / 100.0);
  } else {
    chanFeed(chBoost, R_NONE, 0);
  }
  WDT_KICK();

  // RAIL: uu tien PID 23, du phong PID 59 (10 kPa/bit = 0.1 bar/bit)
  byte st = R_NONE;
  float v = 0;
  if (getPid(0x23, a, b)) {
    unsigned int raw = (unsigned int)a * 256 + b;
    if (raw) { st = R_GOOD; v = raw * 0.1; } else st = R_ZERO;
  }
  if (st != R_GOOD && getPid(0x59, a, b)) {
    unsigned int raw = (unsigned int)a * 256 + b;
    if (raw) { st = R_GOOD; v = raw * 0.1; } else if (st == R_NONE) st = R_ZERO;
  }
  chanFeed(chRail, st, v);
  WDT_KICK();
}

// In 1 dong trang thai ra Serial (chi de xem khi cam may tinh)
void printChanSerial(const __FlashStringHelper *name, Chan &c, byte dec) {
  float v;
  bool held = false;
  Serial.print(name);
  if (chanGet(c, v, held)) {
    Serial.print(v, dec);
    if (held) Serial.print('*');
  } else {
    Serial.print(F("--"));
  }
  Serial.print(F("  "));
}

void printStatus() {
  printChanSerial(F("CRANK="), chCrank, 0);
  printChanSerial(F("CAM="),   chCam,   0);
  printChanSerial(F("BOOST="), chBoost, 2);
  printChanSerial(F("RAIL="),  chRail,  1);
  Serial.println();
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);              // KHONG cho Serial (while(!Serial)) de chay duoc khi khong cam may tinh
  Serial.println(F("=== SantaFe ECU TCI-D CAN Reader (ban gan xe) ==="));

#if USE_WATCHDOG
  wdt_enable(WDTO_8S);
#endif

  Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);   // tranh treo khi bus I2C bi nhieu
#endif

  lcdInitAll();
  i2cScan();

  // Khoi dong MCP2515: neu loi thi bao tren LCD 1 va thu lai mai, khong dung han
  do {
    runDetection();
    if (activeSpeed == -2) {
      lcdLine(lcdCrank, 0, "LOI MCP2515");
      lcdLine(lcdCrank, 1, "Kiem tra day SPI");
      for (byte i = 0; i < 20; i++) { delay(100); WDT_KICK(); }
    }
  } while (activeSpeed == -2);

  normalOn();
  scanPids();
  lcdTitles();
  Serial.println(F("Chay tu dong."));
}

void loop() {
  static unsigned long lastPoll = 0, lastLcd = 0, lastReinit = 0, lastScan = 0;
  static unsigned long lastCountSeen = 0, lastFrameSeen = 0;

  WDT_KICK();
  unsigned long now = millis();

  pumpCan();   // luon doc khung CAN de CRANK/CAM moi

  if (now - lastPoll >= POLL_MS) {
    lastPoll = now;
    pollObd();
    printStatus();
  }

  if (now - lastLcd >= LCD_MS) {
    lastLcd = now;
    showLcd();
  }

  if (now - lastReinit >= LCD_REINIT_MS) {
    lastReinit = now;
    lcdReinit();
  }

  // Neu luc bat nguon ECU chua san sang thi quet lai PID dinh ky
  if (!pidScanned && now - lastScan >= 10000UL) {
    lastScan = now;
    scanPids();
  }

  // Neu mat het khung CAN qua lau (ECU tat hoac sai toc do) thi do lai
  if (frameCount != lastCountSeen) {
    lastCountSeen = frameCount;
    lastFrameSeen = now;
  } else if (now - lastFrameSeen > NO_FRAME_REDETECT_MS) {
    lastFrameSeen = now;
    runDetection();
    pidScanned = false;
    lastScan = now;
  }
}
