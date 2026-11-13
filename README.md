# SantaFe ECU TCI-D CAN Reader

Đọc dữ liệu từ ECU  (Hyundai Santa Fe TCI-D ) bằng Arduino Uno + module MCP2515 qua CAN bus. Giải mã các giá trị từ dữ liệu thô (raw) và suy ra giá trị CAM từ trục khuỷu.

---

## 1. Cấu hình quan trọng trong code

```cpp
#define MCP_CLOCK     MCP_8MHZ   // đổi thành MCP_16MHZ nếu thạch anh module là 16 MHz
#define RPM_SCALE     0.25       // hệ số RPM của khung 0x316 (xem mục 5)
#define CRANK_PER_CAM 2.0        // 2 vòng khuỷu = 1 vòng cam
```
---

## 2. Sơ đồ chân ECU Bosch EDC16C39

Chân quan trọng trên giắc **K** (theo sơ đồ carlabimmo.com cho bản Alfa Romeo / Fiat / Lancia):

| Chức năng | Chân giắc K | Màu trong sơ đồ |
|---|---|---|
| Mass (GND) | K/4 | xanh dương |
| +12V thường trực | K/1 và K/5 | đỏ |
| +12V sau khóa (ACC) | K/48 | cam |
| K-line | K/25 | xanh lá |
| CAN-H | K/84 | đen |
| CAN-L | K/83 | trắng |

> **Lưu ý:** sơ đồ này là của biến thể Alfa/Fiat/Lancia. Cùng tên EDC16C39 nhưng đầu giắc của Santa Fe có thể khác. Hãy đối chiếu với sơ đồ điện của bàn thử/xe trước khi cấp nguồn.

```
Giắc K (tóm tắt chân dùng):

   K/1, K/5  ---- +12V  (thường trực)
   K/48      ---- +12V  (ACC, sau khóa)
   K/4       ---- GND
   K/84      ---- CAN-H
   K/83      ---- CAN-L
   K/25      ---- K-line (không dùng trong code này)
```
<img width="713" height="379" alt="image" src="https://github.com/user-attachments/assets/997ba2b2-816e-40b5-ac04-2e3deb19c533" />

---

## 3. Sơ đồ nối dây

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
   |  Arduino  |  SPI   |  MCP2515  |  CAN   |   ECU EDC16C39     |
   |    Uno    |<------>|           |        |                    |
   |           |        |  CANH  ---+--------+-- K/84 (CAN-H)     |
   |           |        |  CANL  ---+--------+-- K/83 (CAN-L)     |
   |    GND ---+--------+-- GND ----+--------+-- K/4  (GND)       |
   +-----------+        +-----------+        +--------------------+

   Nguồn ECU: +12V vào K/1, K/5 (và K/48 để bật), GND vào K/4.
```

Nếu nối qua cổng OBD: CAN-H = chân 6, CAN-L = chân 14, GND = chân 4/5.

Lưu ý: MCP2515 cần điện trở đầu cuối 120 Ω giữa CANH và CANL (nhiều module có sẵn jumper). Bus chỉ nên có đúng 2 điện trở đầu cuối.

---

## 4. Cách dùng

Mở Serial Monitor: **115200 baud**, chọn **Newline**.

| Lệnh | Chức năng |
|---|---|
| `t` | In 1 dòng CSV mỗi giây (khung định kỳ + PID) để đối chiếu máy chẩn đoán |
| `m` | Theo dõi thông số (RPM, CAM, boost, rail...) mỗi giây |
| `p` | Liệt kê PID Mode 01 mà ECU hỗ trợ |
| `d` | Đọc mã lỗi (Mode 03 đã lưu + Mode 07 đang chờ) |
| `s` | Nghe bus, chỉ in khi dữ liệu đổi |
| `a` | Nghe bus, in tất cả khung |
| `k` | Quét dịch vụ riêng 0x21 (chỉ đọc) |
| `wNN` | Theo dõi 1 LID, ví dụ `w05` |
| `l` / `n` | Chế độ chỉ nghe (không ACK) / chế độ NORMAL |
| `r` | Dò lại tốc độ CAN |

---

## 5. Công thức giải mã từ dữ liệu thô

### 5.1 Cấu trúc khung CAN

Mỗi khung gồm ID (11 bit) và tối đa 8 byte dữ liệu. Có hai nguồn dữ liệu:

| Nguồn | Cách lấy | Tính chất |
|---|---|---|
| Khung phát định kỳ (0x316, 0x329, 0x545) | ECU tự gửi, chỉ cần nghe | Định dạng riêng của hãng, phải suy ngược |
| Phản hồi OBD-II (ID 0x7E8) | Arduino gửi yêu cầu tới 0x7DF | Công thức chuẩn SAE J1979 |

### 5.2 RPM từ khung 0x316

Ví dụ khung: `21 FF 94 0B FF 15 00 00`

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| Giá trị | 21 | FF | **94** | **0B** | FF | 15 | 00 | 00 |

Little-endian: byte 2 là byte thấp, byte 3 là byte cao.

```
raw = byte3 × 256 + byte2 = 0x0B94 = 2964
RPM = raw × RPM_SCALE = 2964 × 0.25 = 741
```

### 5.3 Vì sao hệ số là 0.25

PID 0C là chuẩn: `RPM = (A×256 + B) / 4`. Với cùng raw 2964 ta được 741 vòng/phút. Trong log đo, PID 0C luôn gấp khoảng 2 lần số khung 0x316 khi dùng hệ số 0.125:

| Khung 0x316 (hệ số 0.125) | PID 0C | Tỉ lệ |
|---|---|---|
| 374 | 729 | 1,95 |
| 378 | 748 | 1,98 |
| 362 | 757 | 2,09 |
| 361 | 741 | 2,05 |

Vì vậy hệ số 0.25 khớp hơn. Đây là **suy luận từ đối chiếu**; cần xác nhận bằng tốc độ thật (tốc độ motor × tỉ số bánh răng, hoặc số trên máy chẩn đoán).

### 5.4 Phản hồi OBD-II

Yêu cầu (tới 0x7DF): `[số byte] [Mode] [PID]`, ví dụ đọc RPM: `02 01 0C`.
Phản hồi (từ 0x7E8): `[số byte] [Mode+0x40] [PID] [A] [B]`, ví dụ `04 41 0C 0B 94`.

Byte đầu (PCI) theo ISO 15765-2:

| Nibble cao | Loại | Ý nghĩa |
|---|---|---|
| 0 | Single Frame | Nibble thấp = số byte hợp lệ |
| 1 | First Frame | 12 bit tiếp theo = tổng độ dài; Arduino gửi Flow Control `30 00 00` |
| 2 | Consecutive Frame | Khung nối tiếp |

`7F xx 78` là phản hồi "đang xử lý", code chờ thêm tối đa 2 giây.

### 5.5 Bảng công thức PID (A = byte 1, B = byte 2)

| PID | Thông số | Công thức | Đơn vị |
|---|---|---|---|
| 0C | RPM | (256A + B) / 4 | vòng/phút |
| 05 | Nhiệt độ nước | A − 40 | °C |
| 0F | Nhiệt độ khí nạp | A − 40 | °C |
| 0B | MAP | A | kPa |
| 33 | Áp suất khí quyển | A | kPa |
| 23 | Áp suất rail (đo) | (256A + B) × 10 | kPa |
| 59 | Áp suất rail (tuyệt đối) | (256A + B) × 10 | kPa |
| 22 | Áp suất rail tương đối | (256A + B) × 0.079 | kPa |
| 11 | Bướm ga | A × 100 / 255 | % |
| 04 | Tải động cơ | A × 100 / 255 | % |
| 42 | Điện áp ECU | (256A + B) / 1000 | V |

Boost không có PID riêng: `Boost = MAP − Khí quyển` (kPa), chia 100 để ra bar.

### 5.6 Ví dụ tính từ log

| Thông số | Raw | Tính | Kết quả |
|---|---|---|---|
| Nhiệt độ nước | A = 71 | 71 − 40 | 31 °C |
| Nhiệt độ khí nạp | A = 93 | 93 − 40 | 53 °C |
| Boost | MAP 47, baro 100 | 47 − 100 | −53 kPa = −0,53 bar |
| Rail (PID 23) | 3560 | 3560 × 10 kPa | 356,0 bar |

Boost âm là bình thường khi động cơ chỉ quay, cổ hút đang hút chân không.

### 5.7 Giải mã mã lỗi DTC (Mode 03 / 07)

Mỗi mã gồm 2 byte `hi lo`:

- 2 bit cao của `hi`: loại (0 = P, 1 = C, 2 = B, 3 = U)
- 2 bit tiếp: chữ số thứ nhất (0–3)
- 4 bit thấp của `hi`: chữ số thứ hai (hex)
- `lo`: hai chữ số cuối (hex)

Ví dụ `hi = 0x03`, `lo = 0x41` → **P0341**.
Mode 03 = mã đã lưu, Mode 07 = mã đang chờ (còn xuất hiện ở chu kỳ hiện tại).

---

## 6. Giá trị CAM suy ra từ khuỷu

Cam không có PID và ECU không gửi tín hiệu thô cảm biến cam, nên CAM chỉ được **tính** từ quan hệ 2 vòng khuỷu = 1 vòng cam:

```
RPM_cam    = RPM_khuyu / 2
góc_khuyu += RPM_khuyu × 6 × Δt     (1 vòng/phút = 6 độ/giây, Δt tính bằng giây)
góc_khuyu  = góc_khuyu mod 720      (một chu trình 4 kỳ = 720°)
góc_cam    = góc_khuyu / 2          (0–360°)
```

Hạn chế:

- Là giá trị tính toán, **không thay được** việc kiểm tra mã lỗi P0341 hay đo cảm biến cam thật.
- Góc cam chỉ là góc tương đối từ lúc bật Arduino (không có mốc TDC/răng thiếu), sẽ trôi dần. In mỗi giây một lần thì giá trị góc nhảy lung tung vì cam quay nhiều vòng mỗi giây.
- Độ đúng phụ thuộc `RPM_SCALE` và tỉ số cơ khí thật của bàn thử (đếm số răng hai bánh để xác nhận 2:1).

---

## 7. Nhận xét kết quả đo

- **P0341** xuất hiện ở cả Mode 03 và Mode 07: tương quan cam/khuỷu sai, còn đang xảy ra.
- Áp rail dao động 340–398 bar khi động cơ chỉ quay khoảng 370–740 vòng/phút, cùng với P0087/P0091/P0253: phù hợp với việc ECU không đồng bộ được pha cam/khuỷu. Đây là suy luận, cần đo tín hiệu thật (oscilloscope) trước khi kết luận cảm biến cam hỏng.
- P0047, P0101, P0489, P0473 nhiều khả năng là lỗi thứ cấp do động cơ chưa chạy; chưa nên kết luận từng mã.

### Cách xác nhận cam

1. Xóa lỗi, chạy lại, xem P0341 có quay lại ngay không.
2. Kiểm tra giắc, dây tín hiệu và nguồn 5V của cảm biến cam.
3. Kiểm tra khe hở cảm biến, vành tín hiệu cam, và dây curoa có nhảy răng không.
4. Dùng oscilloscope hoặc máy chẩn đoán có đồ thị trực tiếp để so tín hiệu cam với khuỷu.

---

## 8. An toàn

Code chỉ đọc (Mode 01/03/07 và dịch vụ 0x21 đọc). Không ghi gì vào ECU. Nên dùng chế độ `l` (chỉ nghe) khi chỉ cần quan sát bus.
