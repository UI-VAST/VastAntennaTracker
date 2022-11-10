#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>
#include "CRC.h"
#include <iomanip>
#include <cstdint>  
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//antenna 
//46.729963519513205, -117.01708388641099
//ballon
//46.73802679470464, -117.147533986255

//shack
//46.73032939284863, -117.0115247558488
//ballon
//46.73440900125548, -117.03565166345032
//dome
//46.72635185065161, -117.01735499923765
//walmart
//46.73440383906368, -117.03598306509241

//46.737524001312956, -117.1409430634887

double balloonLat = 46.73440383906368;
double balloonLong = -117.03598306509241;
float balloonAlt = .7;

double antennaLat = 46.729963519513205;
double antennaLong = -117.01708388641099;
float antennaAlt = .7;
float antennaHeading;

int setangle;
int setheading;

int curElev;
int curAzim;

unsigned long previousMillis;
unsigned long previousMillis1;

#define upPin 33
#define downPin 27
#define leftPin 25
#define rightPin 26

#define azimPin 34
#define elevPin 35

double unitCircToAzimith(double unit){
    //#input MUST be in deg
    double degrees = 450 - unit;    //#rotate cordinate system 90 deg CCW
    if(degrees < 0){
      degrees += 360;
    }
        
    if (degrees > 360){
      degrees = degrees - 360;
    }     
    return degrees;
}

float DegreeToRadians(float d){
  return (d*PI)/180;
}

void desiredAbsolutePose(double B_LAT, double B_LONG, float B_ALT, double A_LAT, double A_LONG, float A_ALT){
    //#returns the desired antenna tracker pose in degrees, relative to the horizon and compass azimuth
    //#Only for distances not exceeding 475 km (295 miles)
    //#antenna coordinate frame
    //#determine distance from balloon to antenna
    //#Ellipsoidal Earth Projected Plane - FLAT EARTH ASSUMPTION
    //#Source - https://www.govinfo.gov/content/pkg/CFR-2005-title47-vol4/pdf/CFR-2005-title47-vol4-sec73-208.pdf
    //#mean coords
    double ML = (B_LAT+A_LAT)/2;
    double K1 = 111.13209 - 0.56605*cos(DegreeToRadians(2*ML)) + 0.0012*cos(DegreeToRadians(4*ML));
    double K2 = 111.41513*cos(DegreeToRadians(ML)) - 0.09455*cos(DegreeToRadians(3*ML)) + 0.00012*cos(DegreeToRadians(5*ML)); 
    double dy = K1 * (B_LAT - A_LAT);
    double dx = K2 * (B_LONG - A_LONG);
    //#distance in km
    double d = sqrt((dx*dx) + (dy*dy));
    
    //#compass heading, in deg1
    float unitCirc = atan2(dy,dx);
    unitCirc = unitCirc * 180/PI;

    float azimuth = unitCircToAzimith(unitCirc);
    
    //#elevation
    float dz = B_ALT - A_ALT;
    float elevation = atan2(dz,d);    //#make sure these distances are in same units
    float abs_elevation = (elevation) * 180/PI;

    setheading = azimuth;
    setangle = abs_elevation;
}


//controller MAC
//78:21:84:7D:27:44
uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x7D, 0x27, 0x44};
esp_now_peer_info_t peerInfo;
char WIFI_SSID[] = "router";    // Change to a non-const char array
const char* WIFI_PASS = "pass"; // You can leave this one as-is

static const uint32_t GPSBaud = 9600;  
TinyGPSPlus gps;
PacketSerial rfd900;
HardwareSerial uart2(2);
//HardwareSerial uart0(0);
HardwareSerial uart1(1);

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//recieve from RFD900
typedef struct packet {
  double lat;
  double lng;
  float alt;
  float speed;
  int sats;
  int packetcount;
  int cutdown_time;
  bool timer_running;
  bool cutdown_status;
  bool parachute_status;
  int lora_bad_packet;
  int rfd_bad_packet;
} packet;
packet myPacket;

//data send to RFD900
typedef struct recieved_data{
    int cutdown_time;
    bool update_cutdown_time;
    bool RunTimer;
    bool trigger_cutdown;
    bool trigger_parachute;   
}recieved_data;
recieved_data txbuf;

//data share with esp
typedef struct espdata {
  double lat;
  double lng;
  float alt;
  float speed;
  int sats;
  int packetcount;
  int cutdown_time;
  bool timer_running;
  bool cutdown_status;
  bool parachute_status;
  int lora_bad_packet;
  int rfd_bad_packet;
  int heading;
  int angle;
} espdata;
espdata espTX;

typedef struct ControllerData{
  float x;
  float y;
  char mode;
  int heading_offset;
  int angle_offset;

  int cutdown_time;
  bool update_cutdown_time;
  bool RunTimer;
  bool trigger_cutdown;
  bool trigger_parachute;
} ControllerData;
ControllerData espRX;




void RFDSend(){
    //update data in the txbuf
    txbuf.trigger_cutdown = espRX.trigger_cutdown;
    txbuf.RunTimer = espRX.RunTimer;
    txbuf.cutdown_time = espRX.cutdown_time;
    txbuf.update_cutdown_time = espRX.update_cutdown_time;
    txbuf.trigger_parachute = espRX.trigger_parachute;
    //send the  data
    uint32_t crc = CRC::Calculate(&txbuf, sizeof(txbuf), CRC::CRC_32());
    uint8_t payload[sizeof(txbuf)+sizeof(crc)];
    memcpy(&payload[0], &txbuf, sizeof(txbuf));
    memcpy(&payload[sizeof(txbuf)], &crc, sizeof(crc));
    rfd900.send(payload, sizeof(payload));

}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}
 
void espNowSend(){
  // Send message via ESP-NOW 
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &espTX, sizeof(espTX));
  if (result == ESP_ERR_ESPNOW_NO_MEM) {
    //Serial.println("Sent with success");
    //digitalWrite(2, HIGH);
    //delay(100);
    //digitalWrite(2, LOW);
  }
  else {
    //Serial.println("Error sending the data");

  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&espRX, incomingData, sizeof(espRX));
    if(espRX.mode == 'm'){

    }
    
}

void RFDPacketReceived(const uint8_t* buffer, size_t size)
{
  uint8_t buf[size];
  memcpy(buf, buffer, size);
  uint32_t crc1 = CRC::Calculate(buf, sizeof(myPacket), CRC::CRC_32());
  uint32_t crc2;
  memcpy(&crc2, &buf[sizeof(myPacket)], sizeof(crc2));
  if(crc1 == crc2){
    memcpy(&myPacket, &buf[0], sizeof(myPacket));
    
    balloonLat = myPacket.lat;
    balloonLong = myPacket.lng;
    balloonAlt = myPacket.alt/1000;

    espTX.lat = myPacket.lat;
    espTX.lng = myPacket.lng;
    espTX.alt = myPacket.alt;
    espTX.packetcount = myPacket.packetcount;
    espTX.cutdown_time = myPacket.cutdown_time;
    espTX.timer_running = myPacket.timer_running;
    espTX.cutdown_status = myPacket.cutdown_status;
  }
  else{
    espTX.rfd_bad_packet++;
  }  
}

void read_gps(){
  if(uart1.available() > 0){
    if (gps.encode(uart1.read())){
      if (gps.location.isValid() && gps.altitude.isValid())
      {
        antennaLat = gps.location.lat();
        antennaLong = gps.location.lng();
        antennaAlt = gps.altitude.kilometers();
      }
    }
  }  
}

void espNowSetup(){
  // Set device as a Wi-Fi Station
 
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin();
  //Serial.print(WiFi.softAPmacAddress());
  WiFi.disconnect();
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }
}


//calculate azmith and elevation angle
//desiredAbsolutePose(balloonLat, balloonLong, balloonAlt, antennaLat, antennaLong, antennaAlt);

void ManualControl(){
  if(espRX.x > 500){
    digitalWrite(downPin, LOW);
    digitalWrite(upPin, HIGH);
  }
  if(espRX.x < -500){
    digitalWrite(upPin, LOW);
    digitalWrite(downPin, HIGH);
  }
  if(espRX.x < 500 && espRX.x > -500){
    digitalWrite(downPin, LOW);
    digitalWrite(upPin, LOW);
  }

  if(espRX.y > 500){
    digitalWrite(rightPin, LOW);
    digitalWrite(leftPin, HIGH);
  }
  if(espRX.y < -500){
    digitalWrite(leftPin, LOW);
    digitalWrite(rightPin, HIGH);
  }
  if(espRX.y < 500 && espRX.y > -500){
    digitalWrite(leftPin, LOW);
    digitalWrite(rightPin, LOW);
  }
}

void ReadPosition(){
  if(millis() > previousMillis1 + 1000){
    //2000mv 910 
    //3000mv 1770 
    //4000mv 2055
    //4500mv 2340

    //1000mv = 3033 6db
    
    int elev = analogRead(elevPin);
    int azim = analogRead(azimPin);

    curElev = map(elev, 0, 2150, 0, 90);
    curAzim = map(azim, 550, 4000, 0, 360);
    
    
    espTX.angle = curElev;
    espTX.heading = curAzim;

    Serial.print("azim: ");
    Serial.println(curAzim);
    Serial.print("elev: ");
    Serial.println(curElev);
    
    previousMillis1 = millis();
  }
}

void AutoControl(){
  
  if(millis() > previousMillis + 1000){


    if(curAzim > setheading){
      digitalWrite(rightPin, LOW);
      digitalWrite(leftPin, HIGH);
    }
    if(curAzim < setheading){
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, HIGH);
    }

    if(curElev > setangle){
      digitalWrite(upPin, LOW);
      digitalWrite(downPin, HIGH);
    }
    if(curElev < setangle){
      digitalWrite(downPin, LOW);
      digitalWrite(upPin, HIGH);
    }

    previousMillis = millis();
  }
  
  
}



void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);
  

  pinMode(upPin, OUTPUT);
  pinMode(downPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  
  digitalWrite(upPin, LOW);
  digitalWrite(downPin, LOW);
  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, LOW);

  pinMode(azimPin, INPUT);
  pinMode(elevPin, INPUT);
  
  adcAttachPin(azimPin);
  adcAttachPin(elevPin);

  analogSetPinAttenuation(azimPin, ADC_0db);
  analogSetPinAttenuation(elevPin, ADC_0db);

  analogSetClockDiv(255);

  espNowSetup();
  Serial.begin(57600);

  uart2.begin(57600, SERIAL_8N1, 19, 18);  
  rfd900.setStream(&uart2);
  rfd900.setPacketHandler(&RFDPacketReceived);

  //begin gps
  uart1.begin(GPSBaud, SERIAL_8N1, 17, 16);  
  
  

  espRX.mode = 'a'; 
  espRX.cutdown_time = 3600;
  espRX.heading_offset = 12;
  espRX.trigger_cutdown = false;
  espRX.update_cutdown_time = false;
  espRX.RunTimer = false;
}

void loop() {
  rfd900.update();
  read_gps();
  ReadPosition();

  if(espRX.mode == 'm'){
    ManualControl();
  }
  if(espRX.mode == 'a'){
    AutoControl();
  }

}

