# Smart Scale System with RFID Card Management

## GIỚI THIỆU
**Đề bài**: *Thiết kế hệ thống cân điện tử thông minh sử dụng vi điều khiển STM32F4 tích hợp cảm biến trọng lượng HX711, module RFID MFRC522 và màn hình LED 7 đoạn. Hệ thống cho phép lưu trữ trọng lượng của các thẻ RFID khác nhau và quản lý lịch sử cân.*

**Sản phẩm:**
1. Cân điện tử chính xác với độ chính xác ±0.1 kg (100g)
2. Quản lý thẻ RFID với khả năng lưu trữ tối đa 20 thẻ
3. Lưu trữ và hiển thị lịch sử cân (50 lần cân gần nhất)
4. Giao tiếp UART với các lệnh quản lý: LIST, SET, HISTORY
5. Hiển thị trọng lượng trên LED 7 đoạn với 1 chữ số thập phân
6. Bộ lọc trọng lượng thông minh với thuật toán low-pass filter

- Ảnh chụp minh họa:\
  ![Ảnh minh họa](https://soict.hust.edu.vn/wp-content/uploads/logo-soict-hust-1-1024x416.png)

## TÁC GIẢ
- Tên nhóm: Lẩu Thái
- Thành viên trong nhóm
  |STT|Họ tên|MSSV|Công việc|
  |--:|--|--|--|
  |1|Trương Đức Bình|20215531|Lập trình cân, quản lý thẻ, lập trình cho RFID MFRC522 và hiệu chuẩn cân|
  |2|Bùi Trung Đức|20215565|Phần cứng cho RFID, HX711, LED 7 đoạn, lập trình cân và quản lý lịch sử|
  |3|Pham Công Thành|20215642|Xử lý UART, Lập trình các lệnh giao tiếp với UART với Hercules, lập trình xử lý lưu thẻ|

## MÔI TRƯỜNG HOẠT ĐỘNG
- **MCU**: STM32F429-DISC
- **Cảm biến trọng lượng**: Load cell 10kg với IC HX711 (ADC 24-bit)
- **Module RFID**: MFRC522
- **Hiển thị**: 2 LED 7 đoạn 1 chữ số
- **Giao tiếp**: UART 115200 baud
- **Thư viện**: STM32 HAL, TM_MFRC522, HX711 Library

## SƠ ĐỒ SCHEMATIC

### Bảng kết nối chính:
|STM32F429|Module ngoại vi|Chức năng|
|--|--|--|
|PA11|HX711 DT|Đọc dữ liệu từ load cell|
|PA12|HX711 SCK|Clock cho HX711|
|SPI4|MFRC522|Giao tiếp RFID |
|PE4-PE14|LED 7 đoạn|Hiển thị segments A-G, DP|
|PE15|LED 7 đoạn|Common cathode control|
|PG2-PG3|LED 7 đoạn|Digit select (1-2)|
|PA9/PA10|UART1|Debug và giao tiếp PC|
|PG13|LED đỏ|Thẻ đang được quét|
|PG14|LED xanh|Hệ thống sẵn sàng|

### Sơ đồ khối hệ thống:
```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  Load Cell  │────│   HX711     │────│  STM32F429  │
└─────────────┘    └─────────────┘    │             │
                                      │             │
┌─────────────┐    ┌─────────────┐    │             │
│ RFID Reader │────│  MFRC522    │────│             │
└─────────────┘    └─────────────┘    │             │
                                      │             │
┌─────────────┐                       │             │
│7-Seg Display│───────────────────────│             │
└─────────────┘                       │             │
                                      │             │
┌─────────────┐                       │             │
│  UART/PC    │───────────────────────│             │
└─────────────┘                       └─────────────┘
```

## TÍCH HỢP HỆ THỐNG

### Thành phần phần cứng:
- **STM32F429**: Vi điều khiển chính, xử lý dữ liệu và điều khiển hệ thống
- **Load Cell + HX711**: Cảm biến trọng lượng chính xác cao với ADC 24-bit chuyên dụng
- **MFRC522**: Module RFID đọc/ghi thẻ, lưu trữ dữ liệu người dùng
- **LED 7 đoạn**: Hiển thị trọng lượng trực quan với 1 chữ số phần nguyên và 1 chữ số thập phân
- **LED trạng thái**: Báo hiệu trạng thái hoạt động và đang nhận thẻ

### Thành phần phần mềm:
- **Scale Module**: Xử lý đọc và lọc dữ liệu trọng lượng, thuật toán ổn định
- **RFID Manager**: Quản lý thẻ RFID, lưu trữ và truy xuất dữ liệu
- **Display Driver**: Điều khiển LED 7 đoạn, hiển thị multiplexing
- **UART Handler**: Xử lý giao tiếp với PC, các lệnh quản lý
- **Data Manager**: Quản lý lịch sử cân, lưu trữ dữ liệu trong RAM

## ĐẶC TẢ HÀM

### Hàm xử lý cân:
```C
/**
 * Khởi tạo hệ thống cân điện tử
 * Thiết lập HX711, hiệu chuẩn và về không
 */
void scale_init(void);

/**
 * Đọc trọng lượng thô từ HX711
 * @return float Trọng lượng đọc được (kg)
 */
float scale_read_weight(void);

/**
 * Bộ lọc trọng lượng sử dụng low-pass filter
 * @param raw_weight Trọng lượng thô đầu vào
 * @return float Trọng lượng sau khi lọc
 */
float scale_filter_weight(float raw_weight);

/**
 * Kiểm tra độ ổn định của trọng lượng
 * @param weight Trọng lượng cần kiểm tra
 * @return bool true nếu ổn định, false nếu không
 */
bool scale_is_weight_stable(float weight);
```

### Hàm xử lý RFID:
```C
/**
 * Tìm kiếm hoặc đăng ký thẻ mới
 * @param uid Mã UID của thẻ RFID
 * @return CardData* Con trỏ đến dữ liệu thẻ
 */
CardData* find_or_register_card(uint8_t* uid);

/**
 * Thêm trọng lượng vào lịch sử thẻ
 * @param weight_data Con trỏ đến dữ liệu trọng lượng
 * @param weight Trọng lượng cần thêm
 */
void push_weight_to_history(WeightData* weight_data, float weight);

/**
 * Lưu trọng lượng ổn định vào thẻ RFID
 * @param uid Mã UID thẻ
 * @param weight Trọng lượng cần lưu
 */
void save_weight_to_rfid_card(uint8_t* uid, float weight);
```

### Hàm xử lý UART:
```C
/**
 * Xử lý lệnh LIST - hiển thị danh sách thẻ
 */
void handle_list_command(void);

/**
 * Xử lý lệnh SET - đặt tên cho thẻ
 * @param name Tên cần đặt
 * @param index Chỉ số thẻ
 */
void handle_set_command(char* name, int index);

/**
 * Xử lý lệnh HISTORY - hiển thị lịch sử cân
 */
void handle_history_command(void);
```

## THÔNG SỐ KỸ THUẬT

### Đặc tính cân:
- **Độ chính xác**: ±0.1 kg (100g)
- **Khả năng cân**: 10kg
- **Tần suất đo**: 10Hz (100ms/lần)
- **Thời gian ổn định**: 500ms (5 lần đo liên tiếp)
- **Bộ lọc**: Low-pass filter (α = 0.3)

### Khả năng lưu trữ:
- **Số thẻ RFID**: Tối đa 20 thẻ
- **Lịch sử cân**: 50 lần cân gần nhất
- **Lịch sử mỗi thẻ**: 5 giá trị trọng lượng

### Giao tiếp:
- **UART**: 115200 baud, 8N1
- **SPI RFID**: 11.25MHz
- **Lệnh hỗ trợ**: LIST, SET, HISTORY

## KẾT QUẢ

### Giao diện LED 7 đoạn:
- Hiển thị trọng lượng với format: X.X (ví dụ: 5.2 kg)
- Tần suất refresh: 2Hz (500ms)
- Độ sáng ổn định, dễ đọc

### Chức năng RFID:
- Đọc thẻ nhanh chóng (<1 giây)
- Lưu trữ tên và trọng lượng cho mỗi thẻ
- Quản lý lịch sử cân chi tiết

### Giao tiếp UART:
```
--- Registered Card List ---
Card 1 | Name: John_Doe      | UID: 1234567890 | Saved Weight: 75.2 kg
Card 2 | Name: Jane_Smith    | UID: ABCDEF1234 | Saved Weight: 68.5 kg
----------------------------
```

### Video demo:
[Link video sản phẩm hoạt động]

---
*Dự án được phát triển cho môn Hệ Nhúng - Trường Đại học Bách khoa Hà Nội*
