# Smart Scale System with RFID Card Management

## Tổng quan dự án

Hệ thống cân điện tử thông minh sử dụng vi điều khiển STM32F4 tích hợp cảm biến trọng lượng HX711, module RFID MFRC522 và màn hình LED 7 đoạn. Hệ thống cho phép lưu trữ trọng lượng của các thẻ RFID khác nhau và quản lý lịch sử cân.

## Nguyên lý hoạt động

### 1. Cân điện tử và Load cell

#### Nguyên lý hoạt động của Load cell:
- **Load cell** là cảm biến chuyển đổi lực cơ học thành tín hiệu điện
- Sử dụng nguyên lý **strain gauge** (điện trở biến dạng):
  - Khi có lực tác dụng, load cell bị biến dạng
  - Điện trở của strain gauge thay đổi theo mức độ biến dạng
  - Tạo ra sự thay đổi điện áp đầu ra tỷ lệ với trọng lượng

#### IC HX711:
- **Bộ chuyển đổi ADC 24-bit** chuyên dụng cho load cell
- Khuếch đại tín hiệu từ load cell (gain 32, 64, 128)
- Giao tiếp với vi điều khiển qua 2 chân: SCK (clock) và DT (data)
- Hỗ trợ tính năng tare (về không) và calibration (hiệu chuẩn)

### 2. Cấu hình phần cứng

#### Kết nối HX711:
- **DT (Data)**: PA11 (Input)
- **SCK (Clock)**: PA12 (Output)
- **Scale Factor**: 83000 (cho load cell 10kg) hoặc 44000 (cho load cell 20kg)

#### Kết nối RFID MFRC522:
- Sử dụng giao tiếp SPI4
- Tần số SPI: 180MHz/16 = 11.25MHz

#### LED 7 đoạn:
- 2 LED 7 đoạn hiển thị trọng lượng với 1 chữ số thập phân
- Điều khiển qua các chân GPIO trên cổng PE và PG

#### LED trạng thái:
- **LED xanh (PG14)**: Hệ thống sẵn sàng
- **LED đỏ (PG13)**: Báo lỗi

## Sơ đồ mạch

### Kết nối LED 7 đoạn:

```
STM32F4 GPIO Pins:
PE4  -> A (segment a)
PE8  -> B (segment b)  
PE9  -> C (segment c)
PE10 -> D (segment d)
PE11 -> E (segment e)
PE12 -> F (segment f)
PE13 -> G (segment g)
PE14 -> DP (decimal point)
PE15 -> Common cathode control

PG2  -> Digit 1 select
PG3  -> Digit 2 select
```

### Sơ đồ kết nối chi tiết:

```
[STM32F4]
    |
    |-- PE4-PE14 --> 7-Segment Display 1
    |-- PG2-PG3  --> Digit Select
    |-- PA11     --> HX711 DT
    |-- PA12     --> HX711 SCK
    |-- SPI4     --> MFRC522 RFID
    |-- PA9/PA10 --> UART1 (Debug)
    |-- PG13     --> LED Red (Error)
    |-- PG14     --> LED Green (Ready)
```

## Luồng hoạt động chương trình

### 1. Khởi tạo hệ thống:
```c
// Khởi tạo các peripheral
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_SPI4_Init();
MX_USART1_UART_Init();
MX_TIM6_Init();

// Khởi tạo cân và RFID
scale_init();
TM_MFRC522_Init();
```

### 2. Vòng lặp chính:
```c
while (1) {
    // 1. Xử lý lệnh UART
    if (uart_command_ready) {
        process_uart_command();
    }
    
    // 2. Đọc trọng lượng (mỗi 100ms)
    if (current_time - last_measurement_time >= 100) {
        float raw_weight = scale_read_weight();
        float filtered_weight = scale_filter_weight(raw_weight);
        
        // 3. Kiểm tra độ ổn định
        if (scale_is_weight_stable(filtered_weight)) {
            // 4. Hiển thị trọng lượng
            scale_display_weight(filtered_weight);
            
            // 5. Xử lý thẻ RFID
            scale_process_rfid(filtered_weight);
        }
    }
}
```

### 3. Xử lý thẻ RFID:
```c
void scale_process_rfid(float weight) {
    uint8_t CardUID[5];
    
    if (TM_MFRC522_Check(CardUID) == MI_OK) {
        // Tìm hoặc đăng ký thẻ mới
        CardData* current_card = find_or_register_card(CardUID);
        
        // Thêm trọng lượng vào lịch sử
        push_weight_to_history(&current_card->weight_data, weight);
        
        // Kiểm tra độ ổn định (5 lần đo liên tiếp)
        if (are_all_weights_stable(&current_card->weight_data)) {
            // Lưu trọng lượng ổn định
            save_weight_to_rfid_card(current_card->uid, stable_weight);
            current_card->weight_data.saved_weight = stable_weight;
            add_to_weighing_history(current_card, stable_weight);
        }
    }
}
```

### 4. Bộ lọc trọng lượng:
```c
float scale_filter_weight(float raw_weight) {
    // Bộ lọc thông thấp: y[n] = α * x[n] + (1-α) * y[n-1]
    filtered_weight = WEIGHT_FILTER_ALPHA * raw_weight + 
                     (1.0f - WEIGHT_FILTER_ALPHA) * filtered_weight;
    return filtered_weight;
}
```

## Các lệnh UART

Hệ thống hỗ trợ các lệnh qua UART với tốc độ **115200 baud**:

### 1. `LIST`
- **Mô tả**: Hiển thị danh sách tất cả thẻ đã đăng ký
- **Cú pháp**: `LIST`
- **Ví dụ output**:
```
--- Registered Card List ---
Card 1 | Name: John_Doe      | UID: 1234567890 | Saved Weight: 75.2 kg
Card 2 | Name: (not set)     | UID: ABCDEF1234 | Saved Weight: 0.0 kg
----------------------------
```

### 2. `SET <name> <index>`
- **Mô tả**: Đặt tên cho thẻ theo index
- **Cú pháp**: `SET <tên_thẻ> <số_thứ_tự>`
- **Ví dụ**: `SET John_Doe 1`
- **Output**: `Success: Name 'John_Doe' has been set for Card 1.`

### 3. `HISTORY`
- **Mô tả**: Hiển thị lịch sử cân (50 lần cân gần nhất)
- **Cú pháp**: `HISTORY`
- **Ví dụ output**:
```
--- Weighing History (Oldest to Newest) ---
#1 | Time: 12345 ms | Name: John_Doe      | Weight: 75.2 kg
#2 | Time: 23456 ms | Name: Jane_Smith    | Weight: 68.5 kg
--------------------------------------------
```

## Cấu hình và thông số kỹ thuật

### Thông số cân:
- **Độ chính xác**: ±0.01 kg (10g)
- **Số lần đo ổn định**: 5 lần liên tiếp
- **Tần suất đo**: 100ms/lần
- **Tần suất hiển thị**: 500ms/lần
- **Bộ lọc**: Low-pass filter (α = 0.3)

### Giới hạn lưu trữ:
- **Số thẻ tối đa**: 20 thẻ
- **Lịch sử cân**: 50 lần cân gần nhất
- **Lịch sử trọng lượng mỗi thẻ**: 5 giá trị

### Cấu hình GPIO:
```c
// HX711 Load Cell
#define HX711_DT_PORT     GPIOA
#define HX711_DT_PIN      GPIO_PIN_11
#define HX711_SCK_PORT    GPIOA  
#define HX711_SCK_PIN     GPIO_PIN_12

// LED Status
#define LED_RED_PORT      GPIOG
#define LED_RED_PIN       GPIO_PIN_13
#define LED_GREEN_PORT    GPIOG
#define LED_GREEN_PIN     GPIO_PIN_14

// 7-Segment Display
#define SEG_PORT          GPIOE
#define SEG_PINS          GPIO_PIN_4 to GPIO_PIN_15
```

## Thư viện sử dụng

### 1. STM32 HAL Library
- **Nguồn**: STMicroelectronics
- **Mục đích**: Điều khiển peripheral STM32F4
- **Modules**: GPIO, SPI, UART, Timer

### 2. TM_MFRC522 Library
- **Nguồn**: Tilen Majerle
- **File**: `tm_stm32f4_mfrc522.h`
- **Mục đích**: Giao tiếp với module RFID MFRC522
- **Functions**: `TM_MFRC522_Init()`, `TM_MFRC522_Check()`

### 3. HX711 Library
- **File**: `HX711.h`
- **Mục đích**: Giao tiếp với IC HX711 load cell
- **Functions**: 
  - `hx711_init()`: Khởi tạo
  - `get_weight()`: Đọc trọng lượng
  - `tare_all()`: Về không
  - `set_scale()`: Hiệu chuẩn

### 4. 7-Segment Display Library
- **File**: `7seg.h`
- **Mục đích**: Điều khiển màn hình LED 7 đoạn
- **Functions**:
  - `Set7SegDisplayWithDecimal()`: Hiển thị số với dấu thập phân
  - `Run7SegDisplay()`: Chạy hiển thị

## Compilation và Debug

### Compiler Settings:
- **Toolchain**: ARM GCC
- **Optimization**: -O0 (Debug) / -O2 (Release)
- **MCU**: STM32F446RET6
- **Clock**: 180MHz (HSE + PLL)
---
