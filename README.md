# STM32F4xx + LCD + RESISTIVE TOUCH SCREEN
Đồ Án Được Thực Hiện trên Board STM32F4xx do BOSCH phát triển.

Link Board: https://www.waveshare.com/open405r-c-package-a.htm

Link Document: https://www.waveshare.com/wiki/Open405R-C

## CÁC THƯ VIỆN SỬ DỤNG
- LCD_Drive.h    : thiết lập SPI1 để giaop tiếp với LCD
- LCD_lib.h      : định nghĩa các ký tự và front chữ ra LCD
- touch.h        : thiết lập để đọc tọa độ
- xpt2046.h      : kết hợp vơis touch.h để đọc tọa độ
- music_data.h   : dùng để phát qua loa một đoạn nhạc bất kỳ dùng TIMER và DAC
## SETUP CHÂN
Dựa trên sơ đồ chân của mạch thì ta thiết lập các chân như sau, nối theo schematic nhaaa !!!

Link Drive : https://drive.google.com/drive/folders/18jjTm6sdkxmp6SZXy9Hkgw3NPsd61N0m?usp=sharing

![image](https://github.com/user-attachments/assets/739aef66-e247-4458-86b3-8c4c79a9f860)

## HƯỚNG DẪN SỬ DỤNG
1 .LCD
- Chạy lcd_init() để khơi tạo LCD (chạy trong hàm main )

      ...
        lcd_init();
      ...
   
- Khi đó có thể vẽ lên LCD theo trục tọa độ ( đọc kỹ các hàm được viết trong LCD_Driver.h)
2 .touch
- Chạy tp_init() và tp_adjust() để có thể đọc tọa độ ( chạy sau khi khơi tạo LCD)

      ...
        lcd_init();
        tp_init();
        tp_adjust();
      ...
- Sau đó cần 2 biến x, y để có thể lưu giá trị tọa độ đọc được:

      osMutexWait(touchMutexHandle, osWaitForever);
      tp_scan(0);
      tp_get_xy(&x, &y);
      osMutexRelease(touchMutexHandle);

- Sử dụng Mutex để đọc tọa độ an toàn tránh xung đột giữa các Task ( sử dụng khi dùng FreeRTOS chạy nhiều Task, khi chạy 1 task thì không cần).

- Có tọa độ xử lý theo mục đích.

3. Đọc nhiệt độ nội
- Bật ADC1 IN16 để đọc nhiệt độ của chip
- Hàm Read_Temperature() trả về nhiệt độ được làm tròn có thể chưa chính xác, bỏ tính làm tròn sử dụng kiểu dữ liệu float để tăng tính chính xác.
## CÁC LỖI THƯỜNG GẶP
1. Cấu hình chân SPI1
- Đọc thật kỹ Schematic trước khi thiết lập, theo sơ đồ thì PB3 là chân SPI1_SCK chứ không phải chân PA5.
- Đẩy tốc độ Clock lên tối đa (168MHz) để tránh lỗi không hiện thị được LCD, NÊN cấu hình Baud Rate: 16, 32...
2. Giao tiếp CAN
- Khi thiết lập chân PA3 là SPI1_SCK thì PB13 là CAN2_TX, PB12 là CAN2_RX theo mạch là mặc định trên nối lỗi truyền CAN không được.
- Module CAN đã có điện trở 120 Ohm rồi nên không cần nối điện trở giữa hai đầu CAN_H và CAN_L. Đo lại ( nếu cần) điện trở giữa CAN_H và CAN_L là 60 Ohm khi nối vào mạch.
### CODE ĐƯỢC THỰC HIỆN BỞI KUNT1710, MỌI THẮC MẮC CÓ THỂ LIÊN KỆ QUA FACEBOOK.
