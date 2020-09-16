#include "heltec.h"
#include "Wire.h"
#include <TinyGPS.h> 

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6

#define RXD2 23
#define TXD2 17

#define Threshold 60 /* Greater the value, more the sensitivity */

int touch_sensor_value=0;
RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;

TinyGPS gps;

int loader = 0;

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265;
int maxVal=402;

double x;
double y;
double z;

unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet ;

float lat = 28.5458;
float lon = 77.1703;

String latitude = "28.5458"; 
String longitude = "77.1703";

void callback(){
  //placeholder callback function
}

void GetAngle()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);

  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
}

void logo()
{
  Heltec.display->clear();
  Heltec.display->drawString(35, 25, "PROJECT C");
  Heltec.display->display();
}

void setup()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  pinMode(34, INPUT);
   //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
 
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  logo();
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  touchAttachInterrupt(T4, callback, Threshold);
  esp_sleep_enable_touchpad_wakeup();

  LoRa.beginPacket();
  LoRa.setTxPower(20,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print("5f44241c9a6cd952e85ddccd");
  LoRa.print("\n");
  LoRa.print("NOW ALIVE");
  LoRa.print("\n");
  LoRa.endPacket();
}

void loop()
{
  
  GetAngle();

  if(Serial2.available()){
    if(gps.encode(Serial2.read())){
      gps.f_get_position(&lat, &lon);
        latitude = String(lat,6); 
        longitude = String(lon,6); 
    }
  }

  LoRa.beginPacket();
  LoRa.setTxPower(20,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print("5f44241c9a6cd952e85ddccd");
  LoRa.print("\n");
  LoRa.print(x);
  LoRa.print("\n");
  LoRa.print(y);
  LoRa.print("\n");
  LoRa.print(z);
  LoRa.print("\n");
  LoRa.print(latitude);
  LoRa.print("\n");
  LoRa.print(longitude);
  LoRa.print("\n");
  LoRa.endPacket();

  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  if(loader == 0)
  {
    Heltec.display->drawString(120, 0, "--");
    loader++;
  }else if(loader == 1){
    Heltec.display->drawString(120, 0, "\\");
    loader++;
  }else if(loader == 2){
    Heltec.display->drawString(120, 0, "|");
    loader++;
  }else if(loader == 3){
    Heltec.display->drawString(120, 0, "/");
    loader = 0;
  }
    
  Heltec.display->drawString(0, 0, "x");
  Heltec.display->drawString(30, 0, "y");
  Heltec.display->drawString(60, 0, "z");
  Heltec.display->drawString(0, 15, String((int) x));
  Heltec.display->drawString(30, 15, String((int) y));
  Heltec.display->drawString(60, 15, String((int) z));
  Heltec.display->drawString(0, 23, "-------------------------------------------");
  Heltec.display->drawString(0, 30, "latitude");
  Heltec.display->drawString(60, 30, "longitude");
  Heltec.display->drawString(0, 45, latitude);
  Heltec.display->drawString(60, 45, longitude);
  Heltec.display->display();
  


  //delay(500);

  touch_sensor_value = touchRead(T5);
  if(digitalRead(34) == HIGH){
    LoRa.beginPacket();
    LoRa.setTxPower(20,RF_PACONFIG_PASELECT_PABOOST);
    LoRa.print("5f44241c9a6cd952e85ddccd");
    LoRa.print("\n");
    LoRa.print("SLEEP MODE");
    LoRa.print("\n");
    LoRa.endPacket();
    digitalWrite(2, LOW);
    esp_deep_sleep_start();
  }
  
}
