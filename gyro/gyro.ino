#include <MPU6050_tockn.h>
#include <Wire.h>

#include <esp_now.h>
#include <WiFi.h>

// 小车wifmac地址
uint8_t broadcastAddress[] = { 0xD4, 0xD4, 0xDA, 0xCF, 0x85, 0x54 };

// 数据结构体
typedef struct struct_message {
  int id;  // must be unique for each sender board
  uint8_t state;
} struct_message;


struct_message myData;

esp_now_peer_info_t peerInfo;

void data_comucation_setup();
void data_commucation_loop();

MPU6050 mpu6050(Wire);

long timer = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin(19,23);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  data_comucation_setup();
}

void loop() {
  mpu6050.update();
  if(millis() - timer > 500){
    Serial.println("=======================================================");
    if (mpu6050.getAngleY()>=50 )
    {
      myData.state = 4;//right
    }
    if (mpu6050.getAngleY()<=-50)
    {
      myData.state = 3;//left
    }
    if (mpu6050.getAngleX()>=50)
    {
      myData.state = 2;//up
    }
    if (mpu6050.getAngleX()<=-50)
    {
      myData.state = 1;//down
    }
          Serial.print("state: ");Serial.println(myData.state);
    Serial.println("=======================================================\n");
    timer = millis();  
  }
    data_commucation_loop();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void data_comucation_setup() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //Serial.println("Failed to add peer");
    return;
  }
}

void data_commucation_loop() {
  myData.id = 1;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  } else {
    //Serial.println("Error sending the data");
  }
}