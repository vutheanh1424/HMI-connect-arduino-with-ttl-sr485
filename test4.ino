#include <ModbusRTU.h>
#include <SoftwareSerial.h>

#define Rx 6
#define Tx 5
#define DE_RE 7

SoftwareSerial mySerial(Rx, Tx);
ModbusRTU mb;

#define SLAVE_ID 10
#define HMI_AMPERE 9
#define HMI_VOLTAGE 5

unsigned long Time2;

void preTransmission() {
   digitalWrite(DE_RE, HIGH);
}

void postTransmission() {
   digitalWrite(DE_RE, LOW);
}

void setup() {
   pinMode(DE_RE, OUTPUT);
   digitalWrite(DE_RE, LOW);

   Serial.begin(9600);
   mySerial.begin(9600);
   
   mb.begin(&mySerial);
   mb.master();
   mb.setTimeout(100);  // Thêm timeout

   mb.preTransmission(preTransmission);
   mb.postTransmission(postTransmission);

   delay(2000);
   Serial.println("Modbus RTU Communication Initialized!");
   Time2 = millis();
}

void loop() {
   mb.task();  // Di chuyển vào loop()

   if (millis() - Time2 >= 1000) {
     uint16_t ampere, voltage;

     // Kiểm tra kết nối trước khi đọc
     if (mb.readHreg(SLAVE_ID, HMI_AMPERE, &ampere, 1)) {
       while(!mb.isTransactionCompleted()) {
         mb.task();
       }
       
       if (mb.getLastError() == 0) {
         Serial.print("Dòng điện hàn là: ");
         Serial.println(ampere);
       } else {
         Serial.println("Lỗi đọc dòng điện!");
       }
     }

     if (mb.readHreg(SLAVE_ID, HMI_VOLTAGE, &voltage, 1)) {
       while(!mb.isTransactionCompleted()) {
         mb.task();
       }
       
       if (mb.getLastError() == 0) {
         Serial.print("Điện áp hàn là: ");
         Serial.println(voltage);
       } else {
         Serial.println("Lỗi đọc điện áp!");
       }
     }

     Time2 = millis();
   }
}