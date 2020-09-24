#include "heltec.h"
#include "Wire.h"
#include <TinyGPS++.h>

#define BAND 915E6 //you can set band here directly,e.g. 868E6,915E6

#define RXD2 23
#define TXD2 17

#define Threshold 40 /* Greater the value, more the sensitivity */

int btnRead = 0;
RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;

TinyGPSPlus gps;

int emergent = 0;
int loader = 0;
int emergentLoader = 0;

const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int minVal = 265;
int maxVal = 402;

double x;
double y;
double z;

unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet;

float lat = 28.5458;
float lon = 77.1703;

String latitude = "0";
String longitude = "0";

void callback()
{
  //placeholder callback function
}

void GetAngle()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
}

void logo()
{
  Heltec.display->clear();
  Heltec.display->drawString(35, 25, "PROJECT C");
  Heltec.display->display();
}

void setup()
{
  emergent = 0;
  loader = 0;
  emergentLoader = 0;
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
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
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  touchAttachInterrupt(T4, callback, Threshold);
  esp_sleep_enable_touchpad_wakeup();

  LoRa.beginPacket();
  LoRa.setTxPower(20, RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print("5f44241c9a6cd952e85ddccd");
  LoRa.print("\n");
  LoRa.print("NOW ALIVE");
  LoRa.print("\n");
  LoRa.endPacket();
}

void loop()
{

  GetAngle();

  while (Serial2.available() > 0)
  {
    if (gps.encode(Serial2.read()))
    {
      if(gps.location.isValid())
      {
        latitude = String(gps.location.lat(), 6);
        longitude = String(gps.location.lng(), 6);
      }
      else
      {
        latitude = "INVALID";
        longitude = "INVALID";
      }
    }
  }

  LoRa.beginPacket();
  LoRa.setTxPower(20, RF_PACONFIG_PASELECT_PABOOST);
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
  if (emergent == 1)
  {
    LoRa.print("EMERGENCY");
    LoRa.print("\n");
  }
  LoRa.endPacket();

  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  if (emergent == 1)
  {
    if (emergentLoader == 0)
    {
      Heltec.display->drawString(30, 25, "EMERGENCY");
      Heltec.display->display();
      emergentLoader = 1;
    }
    else if (emergentLoader == 1)
    {
      emergentLoader = 0;
      if (loader == 0)
      {
        Heltec.display->drawString(120, 0, "--");
        loader++;
      }
      else if (loader == 1)
      {
        Heltec.display->drawString(120, 0, "\\");
        loader++;
      }
      else if (loader == 2)
      {
        Heltec.display->drawString(120, 0, "|");
        loader++;
      }
      else if (loader == 3)
      {
        Heltec.display->drawString(120, 0, "/");
        loader = 0;
      }

      Heltec.display->drawString(0, 0, "x");
      Heltec.display->drawString(30, 0, "y");
      Heltec.display->drawString(60, 0, "z");
      Heltec.display->drawString(0, 15, String((int)x));
      Heltec.display->drawString(30, 15, String((int)y));
      Heltec.display->drawString(60, 15, String((int)z));
      Heltec.display->drawString(0, 23, "--------------------------------------");
      Heltec.display->drawString(0, 30, "latitude");
      Heltec.display->drawString(60, 30, "longitude");
      Heltec.display->drawString(0, 45, latitude);
      Heltec.display->drawString(60, 45, longitude);
      Heltec.display->display();
    }
  }
  else if ((emergent == 0))
  {
    if (loader == 0)
    {
      Heltec.display->drawString(120, 0, "--");
      loader++;
    }
    else if (loader == 1)
    {
      Heltec.display->drawString(120, 0, "\\");
      loader++;
    }
    else if (loader == 2)
    {
      Heltec.display->drawString(120, 0, "|");
      loader++;
    }
    else if (loader == 3)
    {
      Heltec.display->drawString(120, 0, "/");
      loader = 0;
    }

    Heltec.display->drawString(0, 0, "x");
    Heltec.display->drawString(30, 0, "y");
    Heltec.display->drawString(60, 0, "z");
    Heltec.display->drawString(0, 15, String((int)x));
    Heltec.display->drawString(30, 15, String((int)y));
    Heltec.display->drawString(60, 15, String((int)z));
    Heltec.display->drawString(0, 23, "--------------------------------------");
    Heltec.display->drawString(0, 30, "latitude");
    Heltec.display->drawString(60, 30, "longitude");
    Heltec.display->drawString(0, 45, latitude);
    Heltec.display->drawString(60, 45, longitude);
    Heltec.display->display();
  }

  //delay(500);

  if (digitalRead(34) == HIGH)
  {
    LoRa.beginPacket();
    LoRa.setTxPower(20, RF_PACONFIG_PASELECT_PABOOST);
    LoRa.print("5f44241c9a6cd952e85ddccd");
    LoRa.print("\n");
    LoRa.print("SLEEP MODE");
    LoRa.print("\n");
    LoRa.endPacket();
    digitalWrite(2, LOW);
    esp_deep_sleep_start();
  }

  if (digitalRead(35) == HIGH)
  {
    if (emergent == 0)
    {
      emergent = 1;
    }
  }
  
}
