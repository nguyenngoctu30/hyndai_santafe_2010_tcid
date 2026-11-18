# SantaFe ECU TCI-D CAN Reader

Đọc dữ liệu từ ECU của Hyundai Santa Fe TCI-D bằng Arduino Uno + module MCP2515 qua CAN bus, giải mã **4 thông số chính** từ dữ liệu thô (raw): **CRANK, CAM, BOOST, RAIL**, và hiển thị mỗi thông số lên một màn hình **LCD I2C** riêng (4 LCD).

Phiên bản này dành cho **thiết bị gắn cố định trên xe**: không cần máy tính, không nhận lệnh, tự chạy khi cấp nguồn.

---

## 1. Tóm tắt: 4 thông số lấy từ đâu và hiển thị ở đâu

| Thông số | Lấy từ | Công thức | LCD | Địa chỉ I2C |
|---|---|---|---|---|
| **CRANK** (RPM trục khuỷu) | Khung CAN 0x316 (dự phòng: PID 0C) | `raw × 0.25` (khung 0x316), `(256A+B)/4` (PID 0C) | LCD 1 | 0x27 |
| **CAM** (trục cam) | **Không có tín hiệu thật**: tính từ CRANK | `RPM_cam = RPM_khuỷu × 2` | LCD 2 | 0x26 |
| **BOOST** (áp suất tăng áp) | PID 0B (MAP) và PID 33 (khí quyển) | `MAP − Khí quyển` | LCD 3 | 0x25 |
| **RAIL** (áp suất ray nhiên liệu) | PID 23 (dự phòng: PID 59) | `(256A+B) × 10` kPa | LCD 4 | 0x24 |

Hai cách để có dữ liệu:

| Nguồn | Cách lấy | Tính chất |
|---|---|---|
| Khung phát định kỳ (0x316) | ECU tự gửi, Arduino chỉ cần nghe | Định dạng riêng của hãng, phải suy ngược |
| Phản hồi OBD-II (ID 0x7E8) | Arduino gửi yêu cầu tới ID 0x7DF | Công thức chuẩn SAE J1979 |

---

## 2. Cách đọc một phản hồi OBD-II

Yêu cầu (Arduino gửi tới ID **0x7DF**): `[số byte] [Mode] [PID]`
Phản hồi (ECU gửi từ ID **0x7E8**): `[số byte] [Mode+0x40] [PID] [A] [B]`

A là byte dữ liệu thứ nhất, B là byte thứ hai. Hai byte ghép thành số 16 bit: `raw = 256×A + B`.

Byte đầu của khung (PCI) theo ISO 15765-2:

| Nibble cao | Loại | Ý nghĩa |
|---|---|---|
| 0 | Single Frame | Nibble thấp = số byte hợp lệ |
| 1 | First Frame | 12 bit tiếp theo = tổng độ dài; Arduino gửi Flow Control `30 00 00` |
| 2 | Consecutive Frame | Khung nối tiếp |

Phản hồi `7F xx 78` nghĩa là "đang xử lý", code chờ thêm tối đa 2 giây.

Khi khởi động, code tự quét PID để biết ECU hỗ trợ PID nào, và chỉ hỏi những PID có trong danh sách đó. Nếu lúc bật nguồn ECU chưa sẵn sàng, code quét lại mỗi 10 giây cho đến khi được.

---

## 3. Thông số 1: CRANK (tốc độ trục khuỷu)

Hiển thị trên **LCD 1 (0x27)**.

### 3.1 Nguồn chính: khung định kỳ 0x316

ECU tự gửi khung này, không cần yêu cầu. Ví dụ khung thu được:

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| Giá trị | 21 | FF | **94** | **0B** | FF | 15 | 00 | 00 |

Little-endian: byte 2 là byte thấp, byte 3 là byte cao.

```
raw = byte3 × 256 + byte2 = 0x0B94 = 2964
RPM = raw × RPM_SCALE = 2964 × 0.25 = 741
```

Trong code:

```cpp
#define RPM_SCALE 0.25
RPM = ((unsigned int)d[3] * 256 + d[2]) * RPM_SCALE;
```

### 3.2 Nguồn dự phòng: PID 0C (chuẩn)

Nếu quá 1 giây không nhận được khung 0x316, code chuyển sang hỏi PID 0C:

```
Yêu cầu : 02 01 0C
Phản hồi: 04 41 0C  A  B
RPM = (256×A + B) / 4
```

Ví dụ minh họa (dựng từ giá trị trong log: 741 vòng/phút):

```
Phản hồi: 04 41 0C 0B 94
A = 0x0B = 11,  B = 0x94 = 148
raw = 11×256 + 148 = 2964
RPM = 2964 / 4 = 741 vòng/phút
```

### 3.3 Vì sao hệ số là 0.25

Ban đầu code dùng hệ số 0.125, nhưng số của khung 0x316 luôn bằng khoảng một nửa PID 0C:

| Khung 0x316 (hệ số 0.125) | PID 0C | Tỉ lệ |
|---|---|---|
| 374 | 729 | 1,95 |
| 378 | 748 | 1,98 |
| 362 | 757 | 2,09 |
| 361 | 741 | 2,05 |
| 365 | 725 | 1,99 |
| 379 | 729 | 1,92 |
| 371 | 756 | 2,04 |

Hệ số 0.25 làm hai nguồn khớp nhau. Đây là **suy luận từ đối chiếu**, chưa phải kết quả kiểm chứng. Hãy xác nhận bằng một tốc độ thật (tốc độ motor × tỉ số bánh răng, hoặc số RPM trên máy chẩn đoán). Nếu hai số vẫn lệch, **tin PID 0C**.

---

## 4. Thông số 2: CAM (trục cam)

Hiển thị trên **LCD 2 (0x26)**.

### 4.1 Vì sao không đọc trực tiếp được

- Cảm biến cam **không có PID** trong OBD-II chuẩn.
- ECU không gửi tín hiệu thô của cảm biến cam lên CAN.

Vì vậy giá trị CAM trong code **không phải đo từ cảm biến cam**, mà được **tính** từ CRANK theo tỉ số của bàn thử: **2 vòng cam = 1 vòng khuỷu** (cam quay nhanh gấp đôi khuỷu).

### 4.2 Công thức

```
CRANK_PER_CAM = 0.5                 (tỉ số khuỷu : cam)
RPM_cam = RPM_khuỷu / CRANK_PER_CAM = RPM_khuỷu × 2
```

Ví dụ: khuỷu 741 vòng/phút → cam = 741 × 2 = 1482 vòng/phút.

Trong code:

```cpp
#define CRANK_PER_CAM 0.5
```

Lưu ý quan trọng:

- Với động cơ 4 kỳ thông thường, cam quay **chậm hơn** khuỷu (2 vòng khuỷu = 1 vòng cam, `CRANK_PER_CAM = 2.0`). Bàn thử này được cấu hình theo tỉ số **2 vòng cam = 1 vòng khuỷu** (`0.5`).
- Tỉ số thật phụ thuộc số răng hai bánh trên bàn thử: `CRANK_PER_CAM = số răng bánh cam / số răng bánh khuỷu`. Hãy đếm răng để xác nhận rồi sửa hằng số cho khớp.
- Phiên bản gắn xe **không còn tính góc cam tương đối**, chỉ hiển thị RPM cam.

### 4.3 Cách kiểm tra cam thật sự có vấn đề

Phiên bản gắn xe **không đọc mã lỗi**. Để kiểm tra mã lỗi (ví dụ P0341), dùng máy chẩn đoán hoặc nạp lại bản code có lệnh (bản dùng trên bàn thử, lệnh `d`).

Cách giải mã mã lỗi OBD-II (Mode 03 đã lưu, Mode 07 đang chờ), mỗi mã gồm 2 byte `hi lo`:

- 2 bit cao của `hi`: loại (0 = P, 1 = C, 2 = B, 3 = U)
- 2 bit tiếp: chữ số thứ nhất (0–3)
- 4 bit thấp của `hi`: chữ số thứ hai (hex)
- `lo`: hai chữ số cuối (hex)

Ví dụ `hi = 0x03`, `lo = 0x41` → **P0341** (tương quan cảm biến cam/khuỷu sai). Mode 07 còn mã nghĩa là lỗi vẫn đang xảy ra ở chu kỳ hiện tại.

### 4.4 Hạn chế

- Giá trị tính toán, **không thay được** việc đo cảm biến cam thật bằng oscilloscope.
- Độ đúng phụ thuộc hệ số `RPM_SCALE` (mục 3.3) và tỉ số `CRANK_PER_CAM` (mục 4.2).

---

## 5. Thông số 3: BOOST (áp suất tăng áp)

Hiển thị trên **LCD 3 (0x25)**, đơn vị bar.

Không có PID riêng cho boost. Cần 2 PID rồi trừ nhau.

| PID | Thông số | Công thức | Yêu cầu | Phản hồi |
|---|---|---|---|---|
| 0B | MAP (áp suất tuyệt đối đường nạp) | `A` kPa | `02 01 0B` | `03 41 0B A` |
| 33 | Áp suất khí quyển | `A` kPa | `02 01 33` | `03 41 33 A` |

```
Boost (kPa) = MAP − Khí quyển
Boost (bar) = Boost (kPa) / 100
```

Ví dụ minh họa dựng từ log (MAP 47, khí quyển 100):

```
Phản hồi MAP : 03 41 0B 2F      → A = 0x2F = 47 kPa
Phản hồi baro: 03 41 33 64      → A = 0x64 = 100 kPa
Boost = 47 − 100 = −53 kPa = −0,53 bar
```

Lưu ý trong code:

- Áp suất khí quyển ít thay đổi nên chỉ đọc 1 lần trong 10 chu kỳ. Giá trị chỉ được nhận nếu nằm trong khoảng 80 đến 110 kPa; ngoài khoảng này giữ giá trị trước (mặc định ban đầu 101 kPa).
- MAP = 0 được coi là giá trị không hợp lệ (tính là "bằng 0"), MAP = 255 coi là không có dữ liệu.
- Kết quả boost tính ra bằng 0 **không** bị lọc, vì khi không tải MAP gần bằng khí quyển nên boost ≈ 0 là số thật.

Về số liệu đo: trong log, MAP đứng yên ở 47 kPa (boost −0,53 bar) qua mọi lần đo. Với động cơ diesel đang nổ, MAP lúc cầm chừng thường gần bằng áp suất khí quyển, nên số này **cần được kiểm tra** (có thể là giá trị mô phỏng của bàn thử). Chưa kết luận được.

---

## 6. Thông số 4: RAIL (áp suất ray nhiên liệu)

Hiển thị trên **LCD 4 (0x24)**, đơn vị bar.

| PID | Ý nghĩa | Công thức | Đơn vị | Dùng cho LCD |
|---|---|---|---|---|
| 23 | Áp suất ray (đo, dùng cho diesel) | `(256A + B) × 10` | kPa | Ưu tiên |
| 59 | Áp suất ray (tuyệt đối) | `(256A + B) × 10` | kPa | Dự phòng nếu PID 23 không có |

```
Yêu cầu : 02 01 23
Phản hồi: 04 41 23  A  B
Rail (kPa) = (256×A + B) × 10
Rail (bar) = raw × 0.1          (vì 10 kPa = 0,1 bar)
```

Ví dụ minh họa dựng từ log (raw 3560):

```
Phản hồi: 04 41 23 0D E8
A = 0x0D = 13,  B = 0xE8 = 232
raw = 13×256 + 232 = 3560
Rail = 3560 × 10 = 35 600 kPa = 356,0 bar
```

Nếu ECU không hỗ trợ PID 23 và 59, LCD 4 hiện `-- bar`.

Nhận xét từ số liệu đo: rail dao động từ 336 đến 393 bar trong chưa đầy 3 giây ở tốc độ khoảng 740 vòng/phút. Mức dao động này cần đối chiếu với máy chẩn đoán trước khi kết luận.

---

## 7. Cơ chế chống hiển thị số 0

ECU có lúc trả về 0 (hoặc không trả lời) dù động cơ vẫn hoạt động. Khi động cơ đang nổ, áp ray hay RPM không thể bằng 0, nên số 0 đó là lỗi dữ liệu. Code xử lý như sau cho cả 4 thông số:

1. Mỗi thông số nhớ **giá trị hợp lệ gần nhất** và thời điểm nhận được.
2. Khi lần đọc mới bằng 0 hoặc không có phản hồi, LCD **giữ giá trị cũ** tối đa 5 giây (`HOLD_MS`).
3. Nếu giá trị cũ không được làm mới quá 2,2 giây (`FRESH_MS`), LCD thêm dấu `*` ở cuối dòng để báo "đang giữ số cũ".
4. Nếu sau 5 giây vẫn là 0, LCD mới hiện **0 thật** (ví dụ khi tắt máy). Nếu sau 5 giây vẫn không có dữ liệu, LCD hiện `--`.

| Hiển thị | Ý nghĩa |
|---|---|
| `741 rpm` | Giá trị mới, hợp lệ |
| `741 rpm*` | ECU vừa lỡ nhịp, đang giữ giá trị cũ |
| `0 rpm` | ECU báo 0 liên tục hơn 5 giây (0 thật) |
| `-- rpm` | Chưa có dữ liệu, hoặc mất dữ liệu quá 5 giây |

Hằng số có thể chỉnh trong code:

```cpp
#define HOLD_MS  5000UL   // giữ giá trị cũ tối đa bao nhiêu ms
#define FRESH_MS 2200UL   // quá mốc này thì hiện dấu *
#define SHOW_HOLD_MARK 1  // 0 = không hiện dấu *
```

---

## 8. Màn hình LCD I2C

### 8.1 Địa chỉ 4 LCD

Bốn module LCD I2C (chip PCF8574) dùng chung một đường I2C, phân biệt bằng địa chỉ đặt qua 3 chân A2, A1, A0 (hàn cầu trên module). Địa chỉ PCF8574 là `0x20 + (A2A1A0)`:

| LCD | A2 | A1 | A0 | Địa chỉ | Hiển thị |
|---|---|---|---|---|---|
| LCD 1 | 1 | 1 | 1 | 0x27 | CRANK (RPM trục khuỷu) |
| LCD 2 | 1 | 1 | 0 | 0x26 | CAM (RPM trục cam, suy ra) |
| LCD 3 | 1 | 0 | 1 | 0x25 | BOOST (bar) |
| LCD 4 | 1 | 0 | 0 | 0x24 | RAIL (bar) |

(Các giá trị A2, A1, A0 ở bảng này suy ra từ địa chỉ theo công thức trên; hãy đối chiếu với trạng thái cầu hàn thực tế trên module.)

Nếu module dùng chip **PCF8574A** thì địa chỉ cơ sở là 0x38 thay vì 0x20. Khi đó phải sửa 4 dòng định nghĩa địa chỉ trong code:

```cpp
#define LCD_ADDR_CRANK 0x27
#define LCD_ADDR_CAM   0x26
#define LCD_ADDR_BOOST 0x25
#define LCD_ADDR_RAIL  0x24
```

**Kiểm tra trước khi gắn cố định lên xe:** khi bật nguồn, code in danh sách địa chỉ I2C tìm thấy ra Serial một lần (dòng `I2C: 0x.. 0x.. ...`). Cắm máy tính vào, mở Serial Monitor 115200 baud và xác nhận có đủ 4 địa chỉ. Sau khi gắn cố định sẽ không kiểm tra được nữa.

### 8.2 Nối dây I2C

Nối song song cả 4 LCD vào cùng một đường:

| LCD I2C | Arduino Uno |
|---|---|
| VCC | 5V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

```
                      +--> LCD 1 (0x27)  CRANK
   Arduino A4 (SDA) --+--> LCD 2 (0x26)  CAM
   Arduino A5 (SCL) --+--> LCD 3 (0x25)  BOOST
   5V, GND -----------+--> LCD 4 (0x24)  RAIL
```

### 8.3 Nội dung hiển thị

Mỗi LCD (16x2) hiện tên thông số ở dòng 1 và giá trị ở dòng 2:

```
LCD 1          LCD 2         LCD 3        LCD 4
CRANK (khuyu)  CAM (suy ra)  BOOST        RAIL
741 rpm        1482 rpm      -0.53 bar    356.0 bar
```

(Số trong ví dụ là số minh họa dựng từ log, không phải kết quả đo trực tiếp.) LCD cập nhật 2 lần mỗi giây, chỉ ghi lại khi nội dung thay đổi để đỡ nhấp nháy.

### 8.4 Thư viện và cấu hình

- Cài thư viện **LiquidCrystal I2C** (tác giả Frank de Brabander) trong Library Manager. `Wire.h` có sẵn trong Arduino IDE.
- Code mặc định LCD 16x2. Nếu dùng LCD 20x4, sửa `LCD_COLS` thành 20 và `LCD_ROWS` thành 4.
- Arduino Uno chỉ có 32 KB flash và 2 KB RAM. Nếu biên dịch báo thiếu bộ nhớ, cần rút gọn bớt code.

---

## 9. ECU và sơ đồ chân (tham khảo, chưa xác nhận)

Loại ECU của Santa Fe TCI-D chưa được xác nhận. Cách chắc chắn nhất: đọc nhãn trên thân ECU (số Bosch `0 281 0xx xxx`, số Hyundai `39xxx-xxxxx`) rồi tra sơ đồ chân đúng.

Bảng dưới là của **Bosch EDC16C39 bản Alfa Romeo / Fiat / Lancia** (carlabimmo.com). Đầu giắc của Hyundai có thể khác, **không dùng để cấp nguồn khi chưa đối chiếu sơ đồ điện của bàn thử**.

| Chức năng | Chân giắc K | Màu trong sơ đồ |
|---|---|---|
| Mass (GND) | K/4 | xanh dương |
| +12V thường trực | K/1 và K/5 | đỏ |
| +12V sau khóa (ACC) | K/48 | cam |
| K-line | K/25 | xanh lá |
| CAN-H | K/84 | đen |
| CAN-L | K/83 | trắng |

<img width="713" height="379" alt="image" src="https://github.com/user-attachments/assets/997ba2b2-816e-40b5-ac04-2e3deb19c533" />

---

## 10. Sơ đồ nối dây MCP2515

### MCP2515 <-> Arduino Uno

| MCP2515 | Arduino Uno |
|---|---|
| VCC | 5V |
| GND | GND |
| CS | D10 |
| SO (MISO) | D12 |
| SI (MOSI) | D11 |
| SCK | D13 |

### MCP2515 <-> ECU

```
   +-----------+        +-----------+        +--------------------+
   |  Arduino  |  SPI   |  MCP2515  |  CAN   |        ECU         |
   |    Uno    |<------>|           |        |                    |
   |           |        |  CANH  ---+--------+-- CAN-H            |
   |           |        |  CANL  ---+--------+-- CAN-L            |
   |    GND ---+--------+-- GND ----+--------+-- GND (chung)      |
   +-----------+        +-----------+        +--------------------+
```

Nếu nối qua cổng OBD: CAN-H = chân 6, CAN-L = chân 14, GND = chân 4/5.

MCP2515 cần điện trở đầu cuối 120 Ω giữa CANH và CANL (nhiều module có sẵn jumper). Bus chỉ nên có đúng 2 điện trở đầu cuối.

---

## 11. Cách hoạt động và cấu hình

### 11.1 Cấu hình trong code

```cpp
#define MCP_CLOCK     MCP_8MHZ   // đổi thành MCP_16MHZ nếu thạch anh module là 16 MHz
#define RPM_SCALE     0.25       // hệ số RPM của khung 0x316 (xem mục 3.3)
#define CRANK_PER_CAM 0.5        // 2 vòng cam = 1 vòng khuỷu (xem mục 4.2)
#define LCD_COLS      16         // LCD 16x2; đổi 20 và 4 nếu dùng LCD 20x4
#define LCD_ROWS      2
#define USE_WATCHDOG  1          // 1 = tự reset nếu treo
```

### 11.2 Trình tự khi cấp nguồn

1. Khởi động 4 LCD, hiện "SantaFe TCI-D / Dang khoi dong".
2. Quét I2C và in danh sách địa chỉ ra Serial.
3. Dò tốc độ CAN (500, 250, 125 kbps), mỗi tốc độ nghe 3 giây.
4. Quét PID ECU hỗ trợ.
5. Chạy vòng lặp chính: đọc khung CAN liên tục, hỏi PID BOOST/RAIL mỗi 1 giây, cập nhật LCD mỗi 0,5 giây.

### 11.3 Không nhận lệnh

Code **không đọc Serial**. Lý do: thiết bị gắn trên xe không có người gõ lệnh, và nhiễu trên chân RX có thể vô tình gửi một ký tự làm đổi chế độ. Serial (115200 baud) chỉ in thông tin để xem khi cắm máy tính: dòng khởi động, danh sách PID, và mỗi giây một dòng trạng thái:

```
CRANK=741  CAM=1482  BOOST=-0.53  RAIL=356.0
```

Giá trị đang được giữ (không còn mới) có dấu `*`, giá trị mất dữ liệu hiện `--`.

Các chức năng của bản có lệnh (đọc mã lỗi, nghe bus, xuất CSV, quét dịch vụ 0x21) **đã bỏ** khỏi bản gắn xe. Nếu cần các chức năng đó, dùng bản code có lệnh trên bàn thử.

### 11.4 Tự phục hồi khi chạy lâu trên xe

| Tình huống | Cách xử lý |
|---|---|
| MCP2515 không phản hồi | Hiện `LOI MCP2515 / Kiem tra day SPI` trên LCD 1, thử lại liên tục |
| ECU chưa sẵn sàng lúc bật nguồn | Quét PID lại mỗi 10 giây |
| Mất hết khung CAN hơn 30 giây | Dò lại tốc độ CAN và quét lại PID |
| Không nhận khung 0x316 hơn 1 giây | CRANK lấy từ PID 0C |
| LCD lỗi hiển thị do nhiễu | Khởi động lại 4 LCD mỗi 60 giây |
| Bus I2C bị treo | Có timeout I2C (nếu thư viện Wire hỗ trợ) |
| Chương trình bị treo | Watchdog 8 giây tự reset Arduino |

Nếu bản mạch Arduino là bản clone cũ bị reset lặp khi bật watchdog, đặt `USE_WATCHDOG 0`.

---

## 12. Giá trị khi hoạt động bình thường (từ log đo)

Ba lần đo liên tiếp khoảng 16:35:21 đến 16:35:23:

| Thông số | Lần 1 | Lần 2 | Lần 3 |
|---|---|---|---|
| CRANK, khung 0x316 (hệ số cũ 0.125) | 365 | 379 | 371 |
| CRANK, PID 0C | 725 | 729 | 756 |
| CAM suy ra (code cũ: khuỷu / 2) | 181,3 | 182,3 | 189,0 |
| MAP / khí quyển | 47 / 100 kPa | 47 / 100 kPa | 47 / 100 kPa |
| BOOST | −0,53 bar | −0,53 bar | −0,53 bar |
| RAIL, PID 23 | 393,4 bar | 345,0 bar | 336,2 bar |
| Nhiệt độ nước / khí nạp | 31 / 53 °C | 31 / 53 °C | 31 / 53 °C |

Log này đo bằng **bản code cũ** (hệ số 0.125, cam = khuỷu / 2). Với code mới, giá trị kỳ vọng khi chạy bình thường:

| Thông số | Khoảng giá trị kỳ vọng |
|---|---|
| CRANK | khoảng 730 đến 760 vòng/phút (khung 0x316 × 0.25) |
| CAM | khoảng 1460 đến 1510 vòng/phút (khuỷu × 2) |
| BOOST | −0,53 bar (MAP đứng yên 47 kPa trong log) |
| RAIL | khoảng 336 đến 393 bar, đang dao động |

Vì ở trạng thái bình thường RAIL luôn ở mức hàng trăm bar và RPM luôn ở khoảng 700 đến 760, một giá trị 0 xuất hiện giữa lúc đó là lỗi dữ liệu, và là lý do có cơ chế giữ giá trị cũ ở mục 7. Hãy chạy lại với code mới để cập nhật log và xác nhận các khoảng giá trị trên.

---

## 13. Nhận xét kết quả đo

- Các mã lỗi **P0341, P0087, P0091, P0253** từng được đọc từ ECU (cả Mode 03 và Mode 07 với P0341). P0341 là tương quan cam/khuỷu sai, còn đang xảy ra.
- Áp rail dao động cùng với nhóm mã lỗi áp ray: phù hợp với việc ECU không đồng bộ được pha cam/khuỷu. Đây là suy luận, cần đo tín hiệu thật trước khi kết luận cảm biến cam hỏng.
- P0047, P0101, P0489, P0473 nhiều khả năng là lỗi thứ cấp; chưa nên kết luận từng mã.
- Một nguyên nhân có thể gây P0341 trên bàn thử là tỉ số bánh răng/dây curoa giữa khuỷu và cam chưa đúng với ECU mong đợi.
- MAP không đổi (47 kPa) và áp ray dao động mạnh là hai điểm cần đối chiếu thêm với máy chẩn đoán.

Cách xác nhận cam:

1. Xóa lỗi, chạy lại, xem P0341 có quay lại ngay không.
2. Kiểm tra giắc, dây tín hiệu và nguồn 5V của cảm biến cam.
3. Kiểm tra khe hở cảm biến, vành tín hiệu cam, và dây curoa có nhảy răng không.
4. Dùng oscilloscope hoặc máy chẩn đoán có đồ thị trực tiếp để so tín hiệu cam với khuỷu.

---

## 14. An toàn

Code chỉ đọc (Mode 01 và khung phát định kỳ). Không ghi gì vào ECU.

---

## 15. Tác giả

Cấu trúc code + nghiên cứu thuộc về tác giả TUIOT
