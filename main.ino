/*
 * SantaFe ECU CRDi CAN Reader
 * Doc ECU Bosch EDC16C39 (Hyundai Santa Fe) bang Arduino Uno + MCP2515
 *
 * Noi day MCP2515 <-> Arduino Uno:
 *   VCC -> 5V      GND -> GND
 *   CS  -> D10     SO  -> D12 (MISO)
 *   SI  -> D11     SCK -> D13
 * CAN: CANH -> ECU (K/84 hoac OBD pin 6), CANL -> ECU (K/83 hoac OBD pin 14)
 *      GND chung (OBD pin 4/5)
 *
 * Serial Monitor: 115200 baud, chon "Newline"
 * Lenh:
 *   t   = in 1 dong CSV moi giay (khung dinh ky + PID) de doi chieu may chan doan
 *   m   = theo doi de doc (RPM, boost, rail, cam...) moi giay
 *   p   = liet ke PID Mode 01 ma ECU ho tro
 *   d   = doc ma loi (Mode 03 + Mode 07)
 *   s   = nghe bus, chi in khi du lieu doi
 *   a   = nghe bus, in tat ca khung
 *   k   = quet dich vu rieng 0x21 (chi doc)
 *   wNN = theo doi 1 LID, vi du w05
 *   l   = che do chi nghe (khong ACK, khong gui)     n = che do NORMAL
 *   r   = do lai toc do CAN
 *
 * CAM: khong doc tu cam bien that. Gia tri CAM duoc SUY RA tu RPM khuyu
 * (khung 0x316) theo ti so CRANK_PER_CAM (2 vong khuyu = 1 vong cam).
 */

#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS_PIN 10
#define MCP_CLOCK  MCP_8MHZ   // doi thanh MCP_16MHZ neu thach anh la 16MHz

// He so RPM cua khung 0x316 (byte 3 cao, byte 2 thap), da xac nhan bang may chan doan
#define RPM_SCALE 0.25

// Ti so khuyu : cam. 2 vong khuyu = 1 vong cam
// Neu ban thu dung dai curoa/banh rang khac ti so thi sua so nay
#define CRANK_PER_CAM 2.0

MCP_CAN CAN(CAN_CS_PIN);

const uint8_t speedCodes[] = { CAN_500KBPS, CAN_250KBPS, CAN_125KBPS };
const char *speedNames[]   = { "500 kbps", "250 kbps", "125 kbps" };
const uint8_t NUM_SPEEDS   = 3;

char mode = 's';          // 's' nghe, 'm' monitor, 't' csv, 'w' xem 1 LID
bool showAll = false;
int  activeSpeed = -1;
unsigned long lastMon = 0;
byte watchLidNum = 0;

unsigned long badFrames = 0;
unsigned long frameCount = 0;
static byte bigBuf[256];

// Ban chup khung dinh ky moi nhat: 0x316, 0x329, 0x545
struct Snap { byte d[8]; bool seen; };
Snap snap[3];

// Gia tri CAM suy ra tu khuyu
float crankRpmNow = 0;          // RPM khuyu moi nhat (tu khung 0x316)
float camRpmNow   = 0;          // RPM cam = khuyu / CRANK_PER_CAM
float crankDeg    = 0;          // goc khuyu tich luy 0..720 do (tuong doi)
unsigned long lastCrankUs = 0;  // thoi diem khung 0x316 gan nhat

// Bang PID Mode 01 ma ECU ho tro (PID 0x01..0xE0)
byte pidMap[28];
bool pidScanned = false;

// ---------------------------------------------------------------
// Cap nhat CAM suy ra tu khung 0x316 (goi moi khi nhan khung nay)
void updateCam(const byte *d) {
  unsigned long now = micros();
  crankRpmNow = ((unsigned int)d[3] * 256 + d[2]) * RPM_SCALE;
  camRpmNow = crankRpmNow / CRANK_PER_CAM;
  if (lastCrankUs) {
    float dt = (now - lastCrankUs) / 1e6;
    if (dt < 0.5) {                          // bo qua neu mat khung lau
      crankDeg += crankRpmNow * 6.0 * dt;    // 1 rpm = 6 do/giay
      while (crankDeg >= 720.0) crankDeg -= 720.0;
    }
  }
  lastCrankUs = now;
}

// ---------------------------------------------------------------
void printHexByte(byte b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

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

  // Cap nhat ban chup cac khung dinh ky
  if (!ext && len == 8) {
    int k = -1;
    if (id == 0x316) k = 0;
    else if (id == 0x329) k = 1;
    else if (id == 0x545) k = 2;
    if (k >= 0) { memcpy(snap[k].d, bigBuf, 8); snap[k].seen = true; }
    if (k == 0) updateCam(bigBuf);       // suy ra CAM tu khuyu
  }
  return true;
}

void printFrame(unsigned long id, bool ext, byte len, byte *buf) {
  Serial.print(ext ? F("EXT 0x") : F("STD 0x"));
  Serial.print(id, HEX);
  Serial.print(F("  ["));
  Serial.print(len);
  Serial.print(F("]  "));
  for (byte i = 0; i < len; i++) {
    printHexByte(buf[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void normalOn() { CAN.setMode(MCP_NORMAL); }
void listenOn() { CAN.setMode(MCP_LISTENONLY); }

// ---------------------------------------------------------------
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
    normalOn();   // ECU de ban can module ACK; tren xe that co cac node khac ACK san
    Serial.println(F("San sang. Lenh: t m p d s a k wNN l n r"));
  } else if (activeSpeed == -1) {
    Serial.println(F("Khong thay ECU phat khung nao. Kiem tra nguon, CANH/CANL, GND chung."));
    Serial.println(F("Van thu duoc lenh p / m / d / k (mac dinh 500 kbps)."));
    CAN.begin(MCP_ANY, CAN_500KBPS, MCP_CLOCK);
    normalOn();
    activeSpeed = 0;
  }
}

// ---------------------------------------------------------------
// Nghe bus: in khi du lieu doi va cho biet byte nao doi
void handleSniff() {
  unsigned long id; bool ext; byte len; byte buf[8];
  if (!readFrame(id, ext, len, buf)) return;

  static unsigned long ids[16];
  static byte last[16][8];
  static byte n = 0;

  int k = -1;
  for (byte i = 0; i < n; i++) if (ids[i] == id) { k = i; break; }

  bool isNew = false;
  byte diff = 0;
  if (k >= 0) {
    for (byte i = 0; i < len && i < 8; i++) if (last[k][i] != buf[i]) diff |= (1 << i);
  } else if (n < 16) {
    k = n++;
    ids[k] = id;
    isNew = true;
  } else {
    diff = 0xFF;   // qua nhieu ID, khong theo doi duoc
  }
  if (k >= 0) memcpy(last[k], buf, len);

  if (showAll || isNew || diff) {
    Serial.print(millis());
    Serial.print(F(" ms  "));
    printFrame(id, ext, len, buf);
    if (!isNew && diff) {
      Serial.print(F("      byte doi:"));
      for (byte i = 0; i < 8; i++) if (diff & (1 << i)) { Serial.print(' '); Serial.print(i); }
      Serial.println();
    }
    if (id == 0x316 && len >= 4) {
      Serial.print(F("      RPM = "));
      Serial.print(((unsigned int)buf[3] * 256 + buf[2]) * RPM_SCALE, 1);
      Serial.print(F("  CAM = "));
      Serial.println(camRpmNow, 1);
    }
  }
}

// ---------------------------------------------------------------
// Gui yeu cau ISO 15765 (1 khung), nhan phan hoi 1 khung hoac nhieu khung.
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

// ---------------------------------------------------------------
// PID co duoc ECU bao ho tro khong (dua tren ket qua lenh p)
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
      if (base == 0) Serial.println(F("ECU khong tra loi Mode 01 (co the dung giao thuc rieng cua Hyundai)."));
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
  Serial.println(F("(Quan tam: 0C=RPM, 05=nuoc, 0B=MAP, 33=khi quyen, 23/59/22=rail, 11=ga, 04=tai)"));
}

// ---------------------------------------------------------------
// Che do CSV: 1 dong/giay, de doi chieu may chan doan
void printCsvHeader() {
  Serial.println(F("ms,rpm316,d316,d329,d545,rpm_pid,coolant,iat,map,baro,boost,rail23_raw,rail23_bar,rail59_raw,rail22_raw,thr_pct,load_pct,vbat,cam_rpm,cam_deg"));
}

void printSnapHex(byte k) {
  if (snap[k].seen) for (byte i = 0; i < 8; i++) printHexByte(snap[k].d[i]);
  Serial.print(',');
}

void doCsv() {
  byte a, b;
  Serial.print(millis());
  Serial.print(',');

  // rpm tu khung 0x316
  if (snap[0].seen) Serial.print(((unsigned int)snap[0].d[3] * 256 + snap[0].d[2]) * RPM_SCALE, 1);
  Serial.print(',');
  printSnapHex(0);
  printSnapHex(1);
  printSnapHex(2);

  // rpm theo PID 0C (chuan /4)
  if (getPid(0x0C, a, b)) Serial.print(((unsigned int)a * 256 + b) / 4);
  Serial.print(',');

  // nhiet do nuoc 05, khi nap 0F
  if (getPid(0x05, a, b)) Serial.print((int)a - 40);
  Serial.print(',');
  if (getPid(0x0F, a, b)) Serial.print((int)a - 40);
  Serial.print(',');

  // MAP 0B, khi quyen 33, boost = MAP - khi quyen
  int mapKpa = -1, baroKpa = -1;
  if (getPid(0x0B, a, b)) mapKpa = a;
  if (getPid(0x33, a, b)) baroKpa = a;
  if (mapKpa >= 0) Serial.print(mapKpa);
  Serial.print(',');
  if (baroKpa >= 0) Serial.print(baroKpa);
  Serial.print(',');
  if (mapKpa >= 0 && baroKpa >= 0) Serial.print(mapKpa - baroKpa);
  Serial.print(',');

  // Ap suat rail: PID 23 (10 kPa/bit), 59, 22 - in gia tri tho de doi chieu
  if (getPid(0x23, a, b)) {
    unsigned int raw = (unsigned int)a * 256 + b;
    Serial.print(raw);
    Serial.print(',');
    Serial.print(raw * 0.1, 1);     // 10 kPa/bit = 0.1 bar/bit
  } else {
    Serial.print(',');
  }
  Serial.print(',');
  if (getPid(0x59, a, b)) Serial.print((unsigned int)a * 256 + b);
  Serial.print(',');
  if (getPid(0x22, a, b)) Serial.print((unsigned int)a * 256 + b);
  Serial.print(',');

  // Ga 11, tai 04, dien ap 42
  if (getPid(0x11, a, b)) Serial.print((unsigned int)a * 100 / 255);
  Serial.print(',');
  if (getPid(0x04, a, b)) Serial.print((unsigned int)a * 100 / 255);
  Serial.print(',');
  if (getPid(0x42, a, b)) Serial.print(((unsigned int)a * 256 + b) / 1000.0, 2);

  // CAM suy ra tu khuyu
  if (micros() - lastCrankUs > 500000UL) camRpmNow = 0;
  Serial.print(',');
  Serial.print(camRpmNow, 1);
  Serial.print(',');
  Serial.println(crankDeg / CRANK_PER_CAM, 0);
}

// ---------------------------------------------------------------
// Theo doi de doc (4 thong so)
void doMonitor() {
  byte a, b;
  unsigned int rpm = 0;
  bool haveRpm = false;
  Serial.println(F("=== Thong so ==="));

  // 1) CRANK: ECU chi cho RPM, khong cho tin hieu tho cua cam bien
  if (snap[0].seen) {
    rpm = (unsigned int)((((unsigned int)snap[0].d[3] * 256 + snap[0].d[2]) * RPM_SCALE) + 0.5);
    haveRpm = true;
    Serial.print(F("[CRANK] RPM (khung 0x316): "));
    Serial.println(rpm);
  }
  if (getPid(0x0C, a, b)) {
    Serial.print(F("[CRANK] RPM (PID 0C, chuan /4): "));
    Serial.println(((unsigned int)a * 256 + b) / 4);
  }

  // 2) CAM: suy ra tu khuyu (2 vong khuyu = 1 vong cam), KHONG phai cam bien that
  if (micros() - lastCrankUs > 500000UL) camRpmNow = 0;
  Serial.print(F("[CAM]   (suy ra tu crank) RPM = "));
  Serial.print(camRpmNow, 1);
  Serial.print(F("  goc cam tuong doi = "));
  Serial.print(crankDeg / CRANK_PER_CAM, 0);
  Serial.println(F(" do"));

  // 3) BOOST = MAP (tuyet doi) - ap suat khi quyen
  byte mapKpa = 0, baroKpa = 0;
  bool haveMap  = getPid(0x0B, a, b); if (haveMap) mapKpa = a;
  bool haveBaro = getPid(0x33, a, b); if (haveBaro) baroKpa = a;
  if (haveMap) {
    Serial.print(F("[BOOST] MAP: "));
    Serial.print(mapKpa);
    Serial.print(F(" kPa, khi quyen: "));
    if (haveBaro) { Serial.print(baroKpa); Serial.print(F(" kPa")); }
    else          { Serial.print(F("101 kPa (gia dinh)")); }
    if (mapKpa == 255 || mapKpa == 0) {
      Serial.println();
      Serial.println(F("   ! MAP = 0/255: gia tri khong hop le, bo qua boost"));
    } else {
      int baro = haveBaro ? baroKpa : 101;
      int boost = (int)mapKpa - baro;
      Serial.print(F(" => boost = "));
      Serial.print(boost);
      Serial.print(F(" kPa = "));
      Serial.print(boost / 100.0, 2);
      Serial.println(F(" bar"));
    }
  } else {
    Serial.println(F("[BOOST] ECU khong tra loi PID 0B"));
  }

  // 4) RAIL
  bool railOk = false;
  if (getPid(0x23, a, b)) {
    unsigned int raw = (unsigned int)a * 256 + b;
    Serial.print(F("[RAIL]  PID 23: raw="));
    Serial.print(raw);
    Serial.print(F("  => "));
    Serial.print(raw * 0.1, 1);
    Serial.println(F(" bar"));
    railOk = true;
    if (haveRpm && rpm == 0 && raw * 0.1 > 300)
      Serial.println(F("   ! Rail cao du dong co dung: co the la gia tri mac dinh"));
  }
  if (getPid(0x59, a, b)) {
    unsigned int raw = (unsigned int)a * 256 + b;
    Serial.print(F("[RAIL]  PID 59: raw="));
    Serial.print(raw);
    Serial.print(F("  => "));
    Serial.print(raw * 0.1, 1);
    Serial.println(F(" bar"));
    railOk = true;
  }
  if (getPid(0x22, a, b)) {
    unsigned int raw = (unsigned int)a * 256 + b;
    Serial.print(F("[RAIL]  PID 22 (tuong doi): raw="));
    Serial.print(raw);
    Serial.print(F("  => "));
    Serial.print(raw * 0.00079, 2);
    Serial.println(F(" bar"));
    railOk = true;
  }
  if (!railOk) Serial.println(F("[RAIL]  ECU khong ho tro PID 23/59/22. Dung lenh 't' ghi khung 0x329/0x545 de tim."));

  // Thong so phu
  if (getPid(0x05, a, b)) {
    Serial.print(F("Nhiet do nuoc: "));
    Serial.print((int)a - 40);
    Serial.println(F(" C"));
  }
  if (getPid(0x0F, a, b)) {
    Serial.print(F("Nhiet do khi nap: "));
    Serial.print((int)a - 40);
    Serial.println(F(" C"));
  }
}

// ---------------------------------------------------------------
// Chu thich ma loi P0xxx (ma chuan SAE J2012)
const __FlashStringHelper *dtcHint(unsigned int c) {
  switch (c) {
    case 0x0335: case 0x0336: case 0x0337: case 0x0338: case 0x0339:
      return F("CRANK (cam bien truc khuyu)");
    case 0x0340: case 0x0341: case 0x0342: case 0x0343: case 0x0344:
      return F("CAM (cam bien truc cam)");
    case 0x0105: case 0x0106: case 0x0107: case 0x0108: case 0x0109:
      return F("MAP / ap suat khi nap");
    case 0x0236: case 0x0237: case 0x0238: case 0x0239:
      return F("BOOST (cam bien ap suat tang ap)");
    case 0x0234: case 0x0299:
      return F("boost qua cao / qua thap");
    case 0x0190: case 0x0191: case 0x0192: case 0x0193: case 0x0194:
      return F("RAIL (cam bien ap suat rail)");
    case 0x0087: case 0x0088:
      return F("ap suat rail qua thap / qua cao");
    case 0x0091: case 0x0092:
      return F("van dieu ap nhien lieu (rail)");
    case 0x0251: case 0x0252: case 0x0253: case 0x0254:
      return F("van dinh luong nhien lieu");
    case 0x0100: case 0x0101: case 0x0102: case 0x0103:
      return F("cam bien luu luong khi nap (MAF)");
    case 0x0112: case 0x0113: case 0x0117: case 0x0118: case 0x0119:
      return F("cam bien nhiet do nuoc / khi nap");
    case 0x0045: case 0x0046: case 0x0047: case 0x0048:
      return F("van dieu khien tang ap (turbo)");
    case 0x0470: case 0x0471: case 0x0472: case 0x0473: case 0x0474:
      return F("cam bien ap suat khi thai");
    case 0x0488: case 0x0489: case 0x0490: case 0x0403:
      return F("van EGR");
    default:
      return NULL;
  }
}

void readDtc(byte service) {
  normalOn();
  byte r[40];
  int n = obdRequest(service, -1, r, 40);
  Serial.print(F("Mode 0"));
  Serial.print(service, HEX);
  Serial.print(F(": "));
  if (n < 2 || r[0] != (byte)(service + 0x40)) {
    Serial.println(F("khong phan hoi"));
    return;
  }
  const char types[4] = { 'P', 'C', 'B', 'U' };
  int found = 0;
  for (int i = 2; i + 1 < n; i += 2) {
    byte hi = r[i], lo = r[i + 1];
    if (hi == 0 && lo == 0) continue;
    found++;
    Serial.print(' ');
    Serial.print(types[hi >> 6]);
    Serial.print((hi >> 4) & 3);
    Serial.print(hi & 0x0F, HEX);
    printHexByte(lo);
    if ((hi >> 6) == 0) {
      const __FlashStringHelper *h = dtcHint(((unsigned int)(hi & 0x3F) << 8) | lo);
      if (h) { Serial.print(F(" (")); Serial.print(h); Serial.print(')'); }
    }
  }
  if (found == 0) Serial.print(F("khong co ma loi"));
  Serial.println();
}

// ---------------------------------------------------------------
// Quet dich vu 0x21 (ReadDataByLocalIdentifier), CHI DOC, khong ghi gi vao ECU.
void kwpScan() {
  normalOn();
  byte r[24];
  byte s[2] = { 0x10, 0x81 };   // mo phien chan doan mac dinh
  int n = isoRequest(0x7E0, s, 2, r, 24);
  Serial.print(F("Phien chan doan: "));
  if (n >= 1 && r[0] == 0x50) Serial.println(F("OK"));
  else if (n >= 3 && r[0] == 0x7F) { Serial.print(F("bi tu choi, ma 0x")); printHexByte(r[2]); Serial.println(); }
  else Serial.println(F("khong phan hoi"));

  Serial.println(F("--- Quet LID 0x01..0xFF (gui phim bat ky de dung) ---"));
  byte found = 0, noResp = 0;
  for (int lid = 1; lid <= 0xFF; lid++) {
    byte q[2] = { 0x21, (byte)lid };
    n = isoRequest(0x7E0, q, 2, r, 24);
    if (n >= 2 && r[0] == 0x61) {
      found++;
      noResp = 0;
      Serial.print(F("LID 0x"));
      printHexByte(lid);
      Serial.print(F(": "));
      for (int i = 2; i < n; i++) { printHexByte(r[i]); Serial.print(' '); }
      Serial.println();
    } else if (n == 0) {
      noResp++;
      if (found == 0 && noResp >= 5) {
        Serial.println(F("ECU khong tra loi dich vu 0x21 o dia chi 0x7E0. Dung quet."));
        return;
      }
    }
    delay(20);
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println(F("Da dung quet."));
      return;
    }
  }
  Serial.print(F("Xong. So LID co du lieu: "));
  Serial.println(found);
}

// Xem 1 LID lien tuc
void watchLid() {
  byte r[24];
  byte q[2] = { 0x21, watchLidNum };
  int n = isoRequest(0x7E0, q, 2, r, 24);
  Serial.print(F("LID 0x"));
  printHexByte(watchLidNum);
  Serial.print(F(": "));
  if (n >= 2 && r[0] == 0x61) {
    for (int i = 2; i < n; i++) { printHexByte(r[i]); Serial.print(' '); }
    Serial.println();
  } else {
    Serial.println(F("khong phan hoi"));
  }
}

byte readHexByte() {
  delay(100);
  byte v = 0, cnt = 0;
  while (Serial.available() && cnt < 2) {
    char h = Serial.read();
    byte d;
    if (h >= '0' && h <= '9') d = h - '0';
    else if (h >= 'a' && h <= 'f') d = h - 'a' + 10;
    else if (h >= 'A' && h <= 'F') d = h - 'A' + 10;
    else continue;
    v = (v << 4) | d;
    cnt++;
  }
  return v;
}

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("=== SantaFe ECU CRDi CAN Reader ==="));
  runDetection();

  if (activeSpeed == -2) {
    Serial.println(F("DUNG: MCP2515 khong phan hoi. Sua phan cung roi reset Arduino."));
    while (true) { delay(1000); }
  }
}

void loop() {
  // Canh bao khi khong con khung nao den (chi o che do nghe)
  static unsigned long lastBeat = 0, lastCount = 0;
  if (mode == 's' && millis() - lastBeat > 3000) {
    lastBeat = millis();
    if (frameCount == lastCount) Serial.println(F("[!] Khong co khung moi trong 3 giay (kiem tra nguon ECU, day CAN, GND)"));
    lastCount = frameCount;
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') {
      // bo qua
    } else if (c == 's') { mode = 's'; showAll = false; Serial.println(F("Nghe bus: chi in khi doi.")); }
    else if (c == 'a')   { mode = 's'; showAll = true;  Serial.println(F("Nghe bus: in tat ca.")); }
    else if (c == 'm')   { mode = 'm'; normalOn(); if (!pidScanned) scanPids(); Serial.println(F("Theo doi thong so (moi 1 giay).")); }
    else if (c == 't')   { mode = 't'; normalOn(); if (!pidScanned) scanPids(); printCsvHeader(); }
    else if (c == 'p')   { mode = 's'; scanPids(); }
    else if (c == 'd')   { mode = 's'; readDtc(0x03); readDtc(0x07); }
    else if (c == 'k')   { mode = 's'; kwpScan(); }
    else if (c == 'w')   { watchLidNum = readHexByte(); mode = 'w'; normalOn(); }
    else if (c == 'l')   { listenOn(); Serial.println(F("Che do chi nghe (khong ACK, khong gui).")); }
    else if (c == 'n')   { normalOn(); Serial.println(F("Che do NORMAL.")); }
    else if (c == 'r')   { mode = 's'; pidScanned = false; runDetection(); }
  }

  if (mode == 's') {
    handleSniff();
  } else if (mode == 'm') {
    if (millis() - lastMon > 1000) { lastMon = millis(); doMonitor(); }
  } else if (mode == 't') {
    if (millis() - lastMon > 1000) { lastMon = millis(); doCsv(); }
  } else if (mode == 'w') {
    if (millis() - lastMon > 500) { lastMon = millis(); watchLid(); }
  }
}
