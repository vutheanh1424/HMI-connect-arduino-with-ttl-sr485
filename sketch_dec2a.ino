// Khai báo và thiết lập ban đầu
#define ID_slave 0x01 // ID của slave (arduino UNO)   ID của thiết bị Arduino được dùng để xác định trong hệ thống Modbus (như một "địa chỉ" riêng biệt).
#define BUFFER_SIZE 64 // Kích thước BUFFER    Kích thước của bộ đệm lưu trữ dữ liệu truyền/nhận từ HMI.
// Hai mảng lưu trữ dữ liệu gửi (TxData) và nhận (RxData) qua giao tiếp Serial.
byte TxData[BUFFER_SIZE]; // Dữ liệu truyền đi      
byte RxData[BUFFER_SIZE]; // Dữ liệu nhận về
//Lưu trạng thái của các nút trên HMI, như bắt đầu hàn (StartWelding), bắt đầu kiểm tra (StartTesting), kích hoạt lực sơ cấp, bật dòng hàn, kích hoạt lực thứ cấp.
// Trạng thái của các nút trên HMI
bool StartWelding = false;  // Nút bắt đầu hàn
bool StartTesting = false;  // Nút bắt đầu kiểm tra
bool PrimaryForce = false;  // Nút kích hoạt lực sơ cấp
bool WeldCurrent = false;   // Nút bật dòng hàn
bool SecondaryForce = false;// Nút kích hoạt lực thứ cấp

//Lưu giá trị thời gian của từng giai đoạn hàn như lực sơ cấp, hàn, lực thứ cấp.
// Thời gian trong các giai đoạn hàn
// uint16_t là một kiểu dữ liệu nguyên thủy không dấu (unsigned integer) có kích thước cố định: 16 bit (2 byte).
//Nó lưu trữ các giá trị nguyên từ 0 đến 65535.
uint16_t PrimaryForceTime = 0; 
uint16_t WeldingTime = 0;
uint16_t SecondaryForceTime = 0;

//Biến lưu độ dài dữ liệu nhận được qua Serial.
// Độ dài dữ liệu nhận
uint8_t RxLength = 0;      // 8 bit

//Thiết lập ban đầu
void setup() {
  Serial.begin(115200); // Khởi tạo giao tiếp Serial với tốc độ 115200 baud.
  Serial.setTimeout(10); // Thiết lập thời gian chờ nhận dữ liệu qua Serial là 10ms.

  // Thiết lập chế độ cho các chân
  for (int i = 4; i <= 13; i++) {
    pinMode(i, OUTPUT);   // Chân 4-13: OUTPUT để điều khiển thiết bị.
  }
  pinMode(3, INPUT_PULLUP); // Nút bắt đầu (PIN 3)
  pinMode(2, INPUT_PULLUP); // Nút dừng (PIN 2)
  //Chân 2 và 3: INPUT_PULLUP để đọc tín hiệu nút nhấn (bắt đầu và dừng).
}

//Vòng lặp chính
void loop() {
  serialEvent();  // Xử lý dữ liệu nhận từ HMI.


  // Kiểm tra tín hiệu từ HMI hoặc nút bắt đầu để khởi động hàn
  if (StartWelding || digitalRead(3) == LOW) {        //Nếu StartWelding hoặc nút bắt đầu được nhấn (digitalRead(3) == LOW)
    StartWelding = true; // Bật trạng thái hàn
    updateHMI(); // Gửi trạng thái tới HMI

    // Tính tổng thời gian các giai đoạn hàn
    int totalTime = PrimaryForceTime + WeldingTime + SecondaryForceTime;

    //Điều khiển các thiết bị lần lượt theo trình tự
    digitalWrite(6, HIGH); // Kích hoạt lực sơ cấp
    delayWithCheck(totalTime - (WeldingTime + SecondaryForceTime));
    digitalWrite(5, HIGH); // Bật dòng hàn
    delayWithCheck(WeldingTime);
    digitalWrite(4, HIGH); // Kích hoạt lực thứ cấp
    digitalWrite(5, LOW);  // Tắt dòng hàn
    delayWithCheck(SecondaryForceTime);

    // Reset trạng thái sau khi hoàn thành.
    resetWelding();
  }

  // Xử lý trạng thái kiểm tra
  if (StartTesting) {
    digitalWrite(6, PrimaryForce ? HIGH : LOW); // Lực sơ cấp
    digitalWrite(5, WeldCurrent ? HIGH : LOW); // Dòng hàn
    digitalWrite(4, SecondaryForce ? HIGH : LOW); // Lực thứ cấp
  }
}

// Hàm delay với kiểm tra nút dừng
void delayWithCheck(int time) {                    //Dừng chương trình trong khoảng thời gian time ms nhưng vẫn kiểm tra nút dừng.
  while (time > 0 && digitalRead(2) == HIGH) {      //Nếu nút dừng được nhấn, hủy hàn và reset trạng thái.
    delay(100);
    time -= 100;

    // Kiểm tra nút dừng
    if (digitalRead(2) == LOW) {
      resetWelding();    //Reset trạng thái các thiết bị và gửi cập nhật trạng thái đến HMI.

      return;
    }
  }
}

// Hàm reset trạng thái hàn
void resetWelding() {
  digitalWrite(6, LOW);
  digitalWrite(5, LOW);
  digitalWrite(4, LOW);
  StartWelding = false;
  updateHMI(); // Cập nhật trạng thái lên HMI
}

// Hàm xử lý dữ liệu từ HMI
void serialEvent() {                                                     //Đọc dữ liệu từ HMI vào RxData.
  RxLength = Serial.readBytes(RxData, BUFFER_SIZE);               
  if (RxLength > 0) {
    uint16_t crc_check = CRC16(RxData, RxLength - 2);                    // Kiểm tra CRC (tính toàn vẹn dữ liệu).
    uint16_t crc_receive = RxData[RxLength - 2] << 8 | RxData[RxLength - 1];
    if (RxData[0] == ID_slave && crc_check == crc_receive) {
      processHMIData();                                                 //Nếu dữ liệu hợp lệ, xử lý dữ liệu thông qua processHMIData().
    }
  }
}

// Hàm xử lý các loại lệnh từ HMI (đọc trạng thái nút, đọc/gửi thời gian, cập nhật trạng thái nút).

void processHMIData() {
  switch (RxData[1]) {
    case 0x01: // Đọc trạng thái nút
      sendHMIStatus();
      break;
    case 0x03: // Đọc giá trị thời gian
      sendHMITime();
      break;
    case 0x05: // Cập nhật trạng thái nút
      updateButtonState();
      break;
    case 0x06: // Cập nhật giá trị thời gian
      updateHMITime();
      break;
  }
}

// Hàm cập nhật trạng thái nút
void updateButtonState() {
  uint16_t buttonAddress = RxData[2] << 8 | RxData[3];
  bool buttonValue = (RxData[4] == 0xFF);

  switch (buttonAddress) {
    case 0: StartWelding = buttonValue; break;
    case 1: StartTesting = buttonValue; break;
    case 2: PrimaryForce = buttonValue; break;
    case 3: WeldCurrent = buttonValue; break;
    case 4: SecondaryForce = buttonValue; break;
  }

  sendHMIResponse();
}

// Hàm cập nhật giá trị thời gian
void updateHMITime() {
  uint16_t timeAddress = RxData[2] << 8 | RxData[3];
  uint16_t timeValue = RxData[4] << 8 | RxData[5];

  switch (timeAddress) {
    case 2: PrimaryForceTime = timeValue; break;
    case 3: WeldingTime = timeValue; break;
    case 4: SecondaryForceTime = timeValue; break;
  }

  sendHMIResponse();
}

// Các hàm gửi dữ liệu trạng thái hoặc phản hồi lại HMI.

void sendHMIStatus() { /* Gửi trạng thái nút về HMI */ }
void sendHMITime() { /* Gửi giá trị thời gian về HMI */ }
void sendHMIResponse() { /* Gửi phản hồi tới HMI */ }
uint16_t CRC16(byte *data, int length) { /* Tính CRC */ }
void updateHMI() {
  // Gửi trạng thái các nút đến HMI qua giao tiếp Serial
  Serial.print("StartWelding: ");
  Serial.println(StartWelding);
  Serial.print("StartTesting: ");
  Serial.println(StartTesting);
  Serial.print("PrimaryForce: ");
  Serial.println(PrimaryForce);
  Serial.print("WeldCurrent: ");
  Serial.println(WeldCurrent);
  Serial.print("SecondaryForce: ");
  Serial.println(SecondaryForce);

  // Gửi thời gian các giai đoạn
  Serial.print("PrimaryForceTime: ");
  Serial.println(PrimaryForceTime);
  Serial.print("WeldingTime: ");
  Serial.println(WeldingTime);
  Serial.print("SecondaryForceTime: ");
  Serial.println(SecondaryForceTime);
}