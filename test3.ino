// Khai báo và thiết lập ban đầu
#define ID_slave 0x01        // ID của slave (Arduino UNO)
#define BUFFER_SIZE 64       // Kích thước BUFFER (cho truyền nhận dữ liệu)
#define DE_RE 8              // Chân điều khiển DE/RE của module RS485
#include<ModbusMaster.h>
#include <SoftwareSerial.h>
SoftwareSerial RS485Serial(10, 11); // RX, TX (chọn chân phù hợp)

byte TxData[BUFFER_SIZE];          // Dữ liệu truyền đi
byte RxData[BUFFER_SIZE];          // Dữ liệu nhận về

// Trạng thái của các nút trên HMI
bool StartWelding = false;         // Nút bắt đầu hàn
bool StartTesting = false;         // Nút bắt đầu kiểm tra
bool PrimaryForce = false;         // Lực sơ cấp
bool WeldCurrent = false;          // Dòng hàn
bool SecondaryForce = false;       // Lực thứ cấp

// Thời gian trong các giai đoạn hàn
uint16_t PrimaryForceTime = 0;     
uint16_t WeldingTime = 0;
uint16_t SecondaryForceTime = 0;

// Độ dài dữ liệu nhận
uint8_t RxLength = 0;

// Tính CRC16 cho dữ liệu
uint16_t CRC16(byte *data, int length) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

// Thiết lập ban đầu
void setup() {
  RS485Serial.begin(115200); // Giao tiếp RS485 với HMI
  Serial.begin(9600);        // Dùng cho debug
  pinMode(DE_RE, OUTPUT);    // Chân điều khiển DE/RE
  digitalWrite(DE_RE, LOW);  // Mặc định chế độ nhận

  // Thiết lập chế độ cho các chân
  for (int i = 4; i <= 13; i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(3, INPUT_PULLUP);  // Nút bắt đầu (PIN 3)
  pinMode(2, INPUT_PULLUP);  // Nút dừng (PIN 2)
}

// Vòng lặp chính
void loop() {
  if (RS485Serial.available()) {
    serialEvent();  // Xử lý dữ liệu từ HMI
  }

  if (StartWelding || digitalRead(3) == LOW) {
    StartWelding = true;
    updateHMI();  // Gửi trạng thái tới HMI
    performWeldingSequence();
  }

  if (StartTesting) {
    controlTestingProcess();
  }
}

// Thực hiện chu trình hàn
void performWeldingSequence() {
  int totalTime = PrimaryForceTime + WeldingTime + SecondaryForceTime;

  digitalWrite(6, HIGH); // Kích hoạt lực sơ cấp
  delayWithCheck(PrimaryForceTime);
  digitalWrite(5, HIGH); // Bật dòng hàn
  delayWithCheck(WeldingTime);
  digitalWrite(4, HIGH); // Kích hoạt lực thứ cấp
  digitalWrite(5, LOW);  // Tắt dòng hàn
  delayWithCheck(SecondaryForceTime);

  resetWelding();
}

// Điều khiển kiểm tra trạng thái
void controlTestingProcess() {
  digitalWrite(6, PrimaryForce ? HIGH : LOW);
  digitalWrite(5, WeldCurrent ? HIGH : LOW);
  digitalWrite(4, SecondaryForce ? HIGH : LOW);
}

// Hàm delay với kiểm tra nút dừng
void delayWithCheck(int time) {
  while (time > 0 && digitalRead(2) == HIGH) {
    delay(100);
    time -= 100;

    if (digitalRead(2) == LOW) {
      resetWelding();
      return;
    }
  }
}

// Reset trạng thái hàn
void resetWelding() {
  digitalWrite(6, LOW);
  digitalWrite(5, LOW);
  digitalWrite(4, LOW);
  StartWelding = false;
  updateHMI();
}

// Xử lý dữ liệu nhận từ HMI
void serialEvent() {
  digitalWrite(DE_RE, LOW); // Chuyển về chế độ nhận
  RxLength = RS485Serial.readBytes(RxData, BUFFER_SIZE);
  if (RxLength > 2) {
    uint16_t crc_check = CRC16(RxData, RxLength - 2);
    uint16_t crc_receive = RxData[RxLength - 2] << 8 | RxData[RxLength - 1];
    if (RxData[0] == ID_slave && crc_check == crc_receive) {
      processHMIData();
    }
  }
}

// Xử lý lệnh từ HMI
void processHMIData() {
  switch (RxData[1]) {
    case 0x01: sendHMIStatus(); break;
    case 0x03: sendHMITime(); break;
    case 0x05: updateButtonState(); break;
    case 0x06: updateHMITime(); break;
  }
}

// Cập nhật trạng thái nút
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

// Cập nhật thời gian từ HMI
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

// Gửi phản hồi tới HMI
void sendHMIStatus() {
  // Gửi trạng thái các nút về HMI
}

void sendHMITime() {
  // Gửi thời gian về HMI
}

void sendHMIResponse() {
  digitalWrite(DE_RE, HIGH); // Chuyển sang chế độ gửi
  RS485Serial.write(TxData, BUFFER_SIZE); // Gửi dữ liệu
  digitalWrite(DE_RE, LOW); // Quay lại chế độ nhận
}

void updateHMI() {
  // Gửi trạng thái thiết bị qua Serial
}

