# SantaFe ECU TCI-D CAN Reader

Đọc dữ liệu từ ECU của Hyundai Santa Fe TCI-D bằng Arduino Uno + module MCP2515 qua CAN bus, giải mã **4 thông số chính** từ dữ liệu thô (raw): **CRANK, CAM, BOOST, RAIL**, và hiển thị mỗi thông số lên một màn hình **LCD I2C** riêng (4 LCD).


---

## 1. Tóm tắt: 4 thông số lấy từ đâu và hiển thị ở đâu

| Thông số | Lấy từ | Công thức | LCD | Địa chỉ I2C |
|---|---|---|---|---|
| **CRANK** (RPM trục khuỷu) | Khung CAN 0x316 **và** PID 0C | `(256A+B)/4` (PID 0C) | LCD 1 | 0x20 |
| **CAM** (trục cam) | **Không có tín hiệu thật**: tính từ CRANK | `RPM_cam = RPM_khuỷu × 2` | LCD 2 | 0x21 |
| **BOOST** (áp suất tăng áp) | PID 0B (MAP) và PID 33 (khí quyển) | `MAP − Khí quyển` | LCD 3 | 0x22 |
| **RAIL** (áp suất ray nhiên liệu) | PID 23 (và 59, 22 nếu ECU hỗ trợ) | `(256A+B) × 10` kPa | LCD 4 | 0x23 |

Hai cách để có dữ liệu:

| Nguồn | Cách lấy | Tính chất |
|---|---|---|
| Khung phát định kỳ (0x316, 0x329, 0x545) | ECU tự gửi, Arduino chỉ cần nghe | Định dạng riêng của hãng, phải suy ngược |
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

Khi khởi động, code tự quét PID (lệnh `p`) để biết ECU hỗ trợ PID nào, và chỉ hỏi những PID có trong danh sách đó.

---

## 3. Thông số 1: CRANK (tốc độ trục khuỷu)

Hiển thị trên **LCD 1 (0x20)**.

### 3.1 Cách 1: PID 0C (chuẩn, đáng tin nhất)

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

### 3.2 Cách 2: khung định kỳ 0x316

ECU tự gửi khung này, không cần yêu cầu. Ví dụ khung thu được:

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| Giá trị | 21 | FF | **94** | **0B** | FF | 15 | 00 | 00 |

Little-endian: byte 2 là byte thấp, byte 3 là byte cao.

```
raw = byte3 × 256 + byte2 = 0x0B94 = 2964
RPM = raw × RPM_SCALE = 2964 × 0.25 = 741
```

Cách đọc trong code:

```cpp
#define RPM_SCALE 0.25
RPM = ((unsigned int)d[3] * 256 + d[2]) * RPM_SCALE;
```

**LCD 1 hiển thị giá trị từ khung 0x316** (cập nhật liên tục theo khung ECU gửi). Nếu quá 0,5 giây không có khung mới, LCD hiện `-- rpm`.

### 3.3 Vì sao hệ số là 0.25

Ban đầu code dùng hệ số 0.125, nhưng số của khung 0x316 luôn bằng khoảng một nửa PID 0C:

| Khung 0x316 (hệ số 0.125) | PID 0C | Tỉ lệ |
|---|---|---|
| 374 | 729 | 1,95 |
| 378 | 748 | 1,98 |
| 362 | 757 | 2,09 |
| 361 | 741 | 2,05 |

Hệ số 0.25 làm hai nguồn khớp nhau. Đây là **suy luận từ đối chiếu**, chưa phải kết quả kiểm chứng. Hãy xác nhận bằng một tốc độ thật (tốc độ motor × tỉ số bánh răng, hoặc số RPM trên máy chẩn đoán). Nếu hai số vẫn lệch, **tin PID 0C**.

---

## 4. Thông số 2: CAM (trục cam)

Hiển thị trên **LCD 2 (0x21)**.

### 4.1 Vì sao không đọc trực tiếp được

- Cảm biến cam **không có PID** trong OBD-II chuẩn.
- ECU không gửi tín hiệu thô của cảm biến cam lên CAN.

Vì vậy giá trị CAM trong code **không phải đo từ cảm biến cam**, mà được **tính** từ CRANK theo tỉ số của bàn thử: **2 vòng cam = 1 vòng khuỷu** (cam quay nhanh gấp đôi khuỷu).

### 4.2 Công thức

```
CRANK_PER_CAM = 0.5                 (tỉ số khuỷu : cam)
RPM_cam    = RPM_khuỷu / CRANK_PER_CAM = RPM_khuỷu × 2
góc_khuỷu += RPM_khuỷu × 6 × Δt     (1 vòng/phút = 6 độ/giây; Δt là thời gian giữa 2 khung 0x316, tính bằng giây)
góc_khuỷu  = góc_khuỷu mod 720      (một chu trình 4 kỳ = 720°)
góc_cam    = (góc_khuỷu / CRANK_PER_CAM) mod 360    (0–360°)
```

Ví dụ: khuỷu 741 vòng/phút → cam = 741 × 2 = 1482 vòng/phút.

Trong code:

```cpp
#define CRANK_PER_CAM 0.5
```

Lưu ý quan trọng:

- Với động cơ 4 kỳ thông thường, cam quay **chậm hơn** khuỷu (2 vòng khuỷu = 1 vòng cam, `CRANK_PER_CAM = 2.0`). Bàn thử này được cấu hình theo tỉ số **2 vòng cam = 1 vòng khuỷu** (`0.5`).
- Tỉ số thật phụ thuộc số răng hai bánh trên bàn thử: `CRANK_PER_CAM = số răng bánh cam / số răng bánh khuỷu`. Hãy đếm răng để xác nhận rồi sửa hằng số cho khớp.
- Góc cam được quấn về 0–360° bằng hàm `camDegNow()` (dùng `fmod`), nếu không góc sẽ vượt 360° khi tỉ số 0.5.

### 4.3 Cách kiểm tra cam thật sự có vấn đề

Dùng lệnh `d` để đọc mã lỗi:

```
Yêu cầu : 01 03  (Mode 03, mã đã lưu)   hoặc   01 07  (Mode 07, mã đang chờ)
Phản hồi: 43 (hoặc 47), byte đếm, rồi các cặp 2 byte "hi lo" của từng mã
```

Giải mã mỗi mã lỗi:

- 2 bit cao của `hi`: loại (0 = P, 1 = C, 2 = B, 3 = U)
- 2 bit tiếp: chữ số thứ nhất (0–3)
- 4 bit thấp của `hi`: chữ số thứ hai (hex)
- `lo`: hai chữ số cuối (hex)

Ví dụ `hi = 0x03`, `lo = 0x41` → **P0341** (tương quan cảm biến cam/khuỷu sai). Mode 07 còn mã nghĩa là lỗi vẫn đang xảy ra ở chu kỳ hiện tại.

### 4.4 Hạn chế

- Góc cam chỉ là góc **tương đối** tính từ lúc bật Arduino (không có mốc TDC hay răng thiếu), nên sẽ trôi dần.
- In mỗi giây một lần thì góc nhảy lung tung vì cam quay nhiều vòng mỗi giây. Chỉ nên dùng `cam_rpm`. LCD 2 chỉ hiển thị RPM cam.
- **Không thay được** việc đo cảm biến cam thật bằng oscilloscope.

---

## 5. Thông số 3: BOOST (áp suất tăng áp)

Hiển thị trên **LCD 3 (0x22)**, đơn vị bar.

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

Giải thích: boost âm là **bình thường** khi động cơ chỉ quay khởi động, vì cổ hút đang hút chân không. Boost dương chỉ có khi turbo thật sự nạp áp.

Lưu ý trong code:

- Nếu ECU không hỗ trợ PID 33, code giả định khí quyển là 101 kPa.
- Nếu MAP = 0 hoặc 255, code coi là giá trị không hợp lệ, bỏ qua boost và LCD hiện `-- bar`.

---

## 6. Thông số 4: RAIL (áp suất ray nhiên liệu)

Hiển thị trên **LCD 4 (0x23)**, đơn vị bar.

ECU có thể hỗ trợ một hoặc nhiều PID sau, nên code thử cả ba:

| PID | Ý nghĩa | Công thức | Đơn vị |
|---|---|---|---|
| 23 | Áp suất ray (đo, dùng cho diesel) | `(256A + B) × 10` | kPa |
| 59 | Áp suất ray (tuyệt đối) | `(256A + B) × 10` | kPa |
| 22 | Áp suất ray tương đối với chân không đường nạp | `(256A + B) × 0.079` | kPa |

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

LCD 4 ưu tiên PID 23; nếu không có thì dùng PID 59. PID 22 là áp suất tương đối nên chỉ in ra Serial, không đưa lên LCD.

Nếu ECU không hỗ trợ PID 23/59/22, code báo "không hỗ trợ" và LCD hiện `-- bar`. Khi đó dùng lệnh `t` ghi các khung 0x329 và 0x545 để tìm byte nào đổi theo áp rail (phải so với máy chẩn đoán).

Nhận xét từ số liệu đo: rail dao động 340–398 bar khi động cơ chỉ quay khoảng 370–740 vòng/phút là bất thường. Đây gợi ý ECU không ổn định, nhưng cần đo thêm mới kết luận được.

---

## 7. Màn hình LCD I2C

### 7.1 Địa chỉ 4 LCD

Bốn module LCD I2C (chip PCF8574) dùng chung một đường I2C, phân biệt bằng địa chỉ đặt qua 3 chân A2, A1, A0 (hàn cầu trên module):

| LCD | A2 | A1 | A0 | Địa chỉ | Hiển thị |
|---|---|---|---|---|---|
| LCD 1 | 0 | 0 | 0 | 0x20 | CRANK (RPM trục khuỷu) |
| LCD 2 | 0 | 0 | 1 | 0x21 | CAM (RPM trục cam, suy ra) |
| LCD 3 | 0 | 1 | 0 | 0x22 | BOOST (bar) |
| LCD 4 | 0 | 1 | 1 | 0x23 | RAIL (bar) |

Địa chỉ 0x20 đến 0x23 áp dụng cho chip **PCF8574**. Nếu module dùng chip **PCF8574A** thì địa chỉ là 0x38 đến 0x3B; khi đó phải sửa 4 dòng khai báo LCD trong code cho khớp. Dùng lệnh `i` để kiểm tra địa chỉ thực tế.

### 7.2 Nối dây I2C

Nối song song cả 4 LCD vào cùng một đường:

| LCD I2C | Arduino Uno |
|---|---|
| VCC | 5V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

```
                      +--> LCD 1 (0x20)  CRANK
   Arduino A4 (SDA) --+--> LCD 2 (0x21)  CAM
   Arduino A5 (SCL) --+--> LCD 3 (0x22)  BOOST
   5V, GND -----------+--> LCD 4 (0x23)  RAIL
```

### 7.3 Nội dung hiển thị

Mỗi LCD (16x2) hiện tên thông số ở dòng 1 và giá trị ở dòng 2:

```
LCD 1        LCD 2        LCD 3        LCD 4
CRANK (khuyu)  CAM (suy ra)  BOOST        RAIL
741 rpm        1482 rpm      -0.53 bar    356.0 bar
```

(Số trong ví dụ minh họa dựng từ log, không phải kết quả đo trực tiếp.) Giá trị không có dữ liệu hiện `--`. LCD cập nhật mỗi giây khi chạy chế độ `m` hoặc `t`.

### 7.4 Thư viện và cấu hình

- Cài thư viện **LiquidCrystal I2C** (tác giả Frank de Brabander) trong Library Manager. `Wire.h` có sẵn trong Arduino IDE.
- Code mặc định LCD 16x2. Nếu dùng LCD 20x4, sửa `LCD_COLS` thành 20 và `LCD_ROWS` thành 4.
- Arduino Uno chỉ có 32 KB flash và 2 KB RAM. Nếu biên dịch báo thiếu bộ nhớ, cần rút gọn bớt code.

---

## 8. ECU và sơ đồ chân (tham khảo, chưa xác nhận)

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

## 9. Sơ đồ nối dây MCP2515

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

## 10. Cách dùng

Cấu hình trong code:

```cpp
#define MCP_CLOCK     MCP_8MHZ   // đổi thành MCP_16MHZ nếu thạch anh module là 16 MHz
#define RPM_SCALE     0.25       // hệ số RPM của khung 0x316 (xem mục 3.3)
#define CRANK_PER_CAM 0.5        // 2 vòng cam = 1 vòng khuỷu (xem mục 4.2)
#define LCD_COLS      16         // LCD 16x2; đổi 20 và 4 nếu dùng LCD 20x4
#define LCD_ROWS      2
```

Mở Serial Monitor: **115200 baud**, chọn **Newline**.

Khi bật nguồn, code tự dò tốc độ CAN, quét PID rồi **chạy luôn chế độ theo dõi `m`**, nên 4 LCD tự cập nhật mà không cần gõ lệnh.

| Lệnh | Chức năng |
|---|---|
| `m` | Theo dõi 4 thông số (CRANK, CAM, BOOST, RAIL) mỗi giây, cập nhật LCD (mặc định khi bật nguồn) |
| `t` | In 1 dòng CSV mỗi giây để đối chiếu với máy chẩn đoán, cập nhật LCD |
| `p` | Liệt kê PID Mode 01 mà ECU hỗ trợ |
| `d` | Đọc mã lỗi (Mode 03 đã lưu + Mode 07 đang chờ) |
| `s` | Nghe bus, chỉ in khi dữ liệu đổi (LCD không cập nhật) |
| `a` | Nghe bus, in tất cả khung (LCD không cập nhật) |
| `k` | Quét dịch vụ riêng 0x21 (chỉ đọc) |
| `wNN` | Theo dõi 1 LID, ví dụ `w05` |
| `l` / `n` | Chế độ chỉ nghe (không ACK) / chế độ NORMAL |
| `r` | Dò lại tốc độ CAN |
| `i` | Quét I2C, kiểm tra 4 LCD (mong đợi 0x20 0x21 0x22 0x23) |

Các lệnh `p`, `d`, `k`, `s`, `a` chuyển sang chế độ nghe/đọc nên LCD ngừng cập nhật cho đến khi gõ lại `m`.

### Ví dụ kết quả lệnh `m`

```
=== Thong so ===
[CRANK] RPM (khung 0x316): 374
[CRANK] RPM (PID 0C, chuan /4): 729
[CAM]   (suy ra tu crank) RPM = 182.3  goc cam tuong doi = 75 do
[BOOST] MAP: 47 kPa, khi quyen: 100 kPa => boost = -53 kPa = -0.53 bar
[RAIL]  PID 23: raw=3560  => 356.0 bar
```

Mỗi dòng ứng với một thông số ở mục 3–6. Log này đo bằng **bản code cũ** (hệ số 0.125 và cam = khuỷu / 2), nên số khung 0x316 đang bằng khoảng một nửa PID 0C và số CAM chưa theo tỉ số mới. Hãy chạy lại với code mới và cập nhật log.

### Cột CSV của lệnh `t`

| Cột | Ý nghĩa |
|---|---|
| `rpm316` | CRANK từ khung 0x316 |
| `rpm_pid` | CRANK từ PID 0C |
| `map`, `baro`, `boost` | BOOST và hai thành phần |
| `rail23_raw`, `rail23_bar`, `rail59_raw`, `rail22_raw` | RAIL |
| `cam_rpm`, `cam_deg` | CAM suy ra |
| `d316`, `d329`, `d545` | Dữ liệu thô của 3 khung định kỳ |

---

## 11. Nhận xét kết quả đo

- **P0341** xuất hiện ở cả Mode 03 và Mode 07: tương quan cam/khuỷu sai, còn đang xảy ra.
- Áp rail dao động cùng với P0087/P0091/P0253: phù hợp với việc ECU không đồng bộ được pha cam/khuỷu. Đây là suy luận, cần đo tín hiệu thật trước khi kết luận cảm biến cam hỏng.
- P0047, P0101, P0489, P0473 nhiều khả năng là lỗi thứ cấp do động cơ chưa chạy; chưa nên kết luận từng mã.
- Một nguyên nhân có thể gây P0341 trên bàn thử là tỉ số bánh răng/dây curoa giữa khuỷu và cam chưa đúng với ECU mong đợi.

Cách xác nhận cam:

1. Xóa lỗi, chạy lại, xem P0341 có quay lại ngay không.
2. Kiểm tra giắc, dây tín hiệu và nguồn 5V của cảm biến cam.
3. Kiểm tra khe hở cảm biến, vành tín hiệu cam, và dây curoa có nhảy răng không.
4. Dùng oscilloscope hoặc máy chẩn đoán có đồ thị trực tiếp để so tín hiệu cam với khuỷu.

---

## 12. An toàn

Code chỉ đọc (Mode 01/03/07 và dịch vụ 0x21 đọc). Không ghi gì vào ECU. Nên dùng chế độ `l` (chỉ nghe) khi chỉ cần quan sát bus.

---

## 13. Tác giả

Cấu trúc code + nghiên cứu thuộc về tác giả TUIOT
