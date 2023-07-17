#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>

HardwareSerial SerialPort(2);

typedef struct struct_message {
  int id;
  uint8_t state;
} struct_message;

struct_message myData;
struct_message esp_mpu;

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  memcpy(&myData, incomingData, sizeof(myData));
  esp_mpu.state = myData.state;
}

void setup() {
  Serial.begin(115200);
  SerialPort.begin(9600, SERIAL_8N1, 16, 17);
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
  //  Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  if(SerialPort.available())
  {
    SerialPort.write(esp_mpu.state);
   // Serial.println("send ok"); 
  }else
  {
  //  Serial.println("not available!");
  }
  Serial.println(esp_mpu.state);
  delay(100);
}