#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11);  // RX,TX
#include <MsTimer2.h>
uint8_t state = 0;
// 超声波
#include "SR04.h"
#define TRIG_PIN 2
#define ECHO_PIN 4
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long distance;
char str_buf[16] = { 0 };

#include "dht11.h"//温湿度传感器
#define DHT11PIN 7
dht11 DHT11;

#include <Wire.h>//液晶屏
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);  //配置LCD地址及行列

//电机驱动模块
#define IN1 3  //定义IN1为3口
#define IN2 5  //定义IN2为4口
#define IN3 9  //定义IN1为3口
#define IN4 6  //定义IN2为4口

void lcd_show();
void get_distance();
void dht();
void forward();
void backward();
void turn_left();
void turn_right();
void event_loop();

void setup()
{ //电机驱动模块
  Serial.begin(115200);
  mySerial.begin(9600);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  lcd.init(); //初始化LCD
  lcd.backlight(); //打开背光

  MsTimer2::set(1000, event_loop);
  MsTimer2::start();
}

void loop()
{ get_distance();
if(distance<=10)
{
  turn_left();
  delay(250);
}
  dht();//温湿度模块
  lcd_show();//屏幕控制
    if (mySerial.available()) //读取手势
    {
    state = mySerial.read();
    Serial.println(state);
    Serial.write(state + '0');
    Serial.write("\n");
  }
    switch (state) {
    case 4:
      {
        turn_right();
        break;
      }
    case 3:
      {
        turn_left();
        break;
      }
    case 2:
      {
        forward();
        break;
      }
    case 1:
      {
        backward();
        break;
      }
  }
  state = 0;
}
void get_distance()
{
  //超声波模块
   distance=sr04.Distance();
   //Serial.print(distance);
   //Serial.println("cm");
   //delay(1000);
}
void event_loop() {
  dht();
}
void turn_left()
{
    analogWrite(IN1,255);  //控制电机正转
    analogWrite(IN2,0);
    analogWrite(IN3,0);  //控制电机正转
    analogWrite(IN4,255);
}

void turn_right()
{
    analogWrite(IN1,0);  //控制电机正转
    analogWrite(IN2,255);
    analogWrite(IN3,255);  //控制电机正转
    analogWrite(IN4,0);
}

void forward()
{
    //正转速度为满速的 200/255                     前进
   analogWrite(IN1,255);  //控制电机正转
   analogWrite(IN2,0);
   analogWrite(IN3,255);  //控制电机正转
   analogWrite(IN4,0);
}

void backward()
{
    //正转速度为满速的 200/255                     后退
   analogWrite(IN1,0);  //控制电机正转
   analogWrite(IN2,255);
   analogWrite(IN3,0);  //控制电机正转
   analogWrite(IN4,255);
}
void dht()
{
   //温湿度模块
  int chk = DHT11.read(DHT11PIN);

 // Serial.print("Read sensor: ");
  switch (chk) {
    case DHTLIB_OK:
 //     Serial.println("OK");
      break;
    case DHTLIB_ERROR_CHECKSUM:
 //     Serial.println("Checksum error");
      break;
    case DHTLIB_ERROR_TIMEOUT:
 //     Serial.println("Time out error");
      break;
    default:
 //     Serial.println("Unknown error");
      break;
  }

  //Serial.print("Humidity (%): ");
 // Serial.println((float)DHT11.humidity, 2);

  //Serial.print("Temperature (oC): ");
 // Serial.println((float)DHT11.temperature, 2);
}

void lcd_show()
{
 //屏幕控制 
  lcd.setCursor(0,0);//设置显示位置
  lcd.print("temp" );//显示字符数据
  lcd.setCursor(5,0);//设置显示位置
  lcd.print((float)DHT11.temperature);//显示字符数据
  lcd.setCursor(10,0);//设置显示位置
  lcd.print("C" );//显示字符数据
  lcd.setCursor(0,1);//设置显示位置
  lcd.print("humi");//显示字符数据
  lcd.setCursor(5,1);//设置显示位置
  lcd.print((float)DHT11.humidity);//显示字符数据
  lcd.setCursor(10,1);//设置显示位置
  lcd.print("%");//显示字符数据
}
