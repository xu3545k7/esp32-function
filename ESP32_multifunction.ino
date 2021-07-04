#include "WiFi.h"
#include "FirebaseESP32.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>//BLE lib include
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <vector>
#include <MQUnifiedsensor.h>
#include "SPIFFS.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
LiquidCrystal_I2C lcd(0x27,16,2);
//dht22設定
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN  2   // 設定溫溼度感測器腳位
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();
int ADXL345 = 0x53;
const int AOUT=0;//MQ-7 AOUT腳位
const int DOUT=8;//MQ-7 DOUT腳位
#define DHTTYPE           DHT22 
DHT_Unified dht(DHTPIN, DHTTYPE);
float X_out, Y_out, Z_out;
uint32_t delayMS;
using namespace std;
const char* ssid = "Xperia n"; 
const char* password = "xu3545k7";
struct point{
  float x,y;
};
int flag;
point finalPose;
char *UUID[]={"ASED-5976",
              "QWSE-5976",
              "DEFH-5976",
              "RSVD-5976"
  };
point beaconlocation[] ={(3.5,4.2),
                        (2.4,3.7),
                        (1.7,5.8),
                        (7.9,6.2),
  };
point p[4];  

float sensor_value = 0;
float sensor_value_percentage = 0;
String BeaconUUID;
//declare a BLE scan object
FirebaseData firebaseData;
int scanTime=2;//set scantime, if time too less, you will lost some beacons that not discovered
//int buzzerPin = 12;   
int txPower =-65;
double beaconABSdis[3];
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{void onResult(BLEAdvertisedDevice advertisedDevice){}
};
void logMemory() {
  log_d("Used PSRAM: %d", ESP.getPsramSize() - ESP.getFreePsram());
}
void DHT22fun(){
    // 延遲
  delay(delayMS);
  // 獲取感應器數值
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  float tn = event.relative_humidity/5 ;
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
    
  }
  else {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
    
  }
  
  // 獲取感應器數值(濕度)
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
    
  }
  else {
    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }
  }
void beaconranging() {
  BLEDevice::init("");//clear beacon list
  BLEScan* pBLEScan = BLEDevice::getScan(); //create scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //start to scan
  BLEScanResults foundDevices = pBLEScan->start(scanTime);//wait for scanning
  Serial.println(foundDevices.getCount());
  int BeaconCount=foundDevices.getCount();
  //list discover beacons data
  if(BeaconCount >= 3){
    flag=1;
   for (int i=0; i<BeaconCount; i++)
   {
       BLEAdvertisedDevice d=foundDevices.getDevice(i);

       String BeaconAddress=d.getAddress().toString().c_str();
       Serial.print(BeaconAddress);
       int BeaconRSSI=d.getRSSI();
       if (d.haveServiceUUID())//show UUID
       {
         Serial.print(",UUID=");
         BeaconUUID=d.getServiceUUID().toString().c_str();
         Serial.print(BeaconUUID);
         int j =0;
         int f =0;
         do{
          if(String(UUID[j])=BeaconUUID){
            p[i].x=beaconlocation[j].x;
            p[i].y=beaconlocation[j].y;
            f=1;
            }
            j++;
          }while(j!=4||f!=1);
       }
       Serial.print(",RSSI=");
       Serial.print(BeaconRSSI);
       beaconABSdis[BeaconCount]= pow( 10,((txPower-BeaconRSSI)/20));
       Serial.print(beaconABSdis[BeaconCount]);
   }
      
  }
  else{
  Serial.println("beacons not enough..");
  }
}
void connectWifi() {
  // Let us connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(".......");
  Serial.println("WiFi Connected....IP Address:");
}
float norm (point p) // get the norm of a vector
{
    return pow(pow(p.x,2)+pow(p.y,2),.5);
}
point trilateration(point point1, point point2, point point3, double r1, double r2, double r3)
{
    point resultPose;
    //unit vector in a direction from point1 to point 2
    double p2p1Distance = pow(pow(point2.x-point1.x,2) + pow(point2.y-   point1.y,2),0.5);
    point ex = {(point2.x-point1.x)/p2p1Distance, (point2.y-point1.y)/p2p1Distance};
    point aux = {point3.x-point1.x,point3.y-point1.y};
    //signed magnitude of the x component
    double i = ex.x * aux.x + ex.y * aux.y;
    //the unit vector in the y direction.
    point aux2 = { point3.x-point1.x-i*ex.x, point3.y-point1.y-i*ex.y};
    point ey = { aux2.x / norm (aux2), aux2.y / norm (aux2) };
    //the signed magnitude of the y component
    double j = ey.x * aux.x + ey.y * aux.y;
    //coordinates
    double x = (pow(r1,2) - pow(r2,2) + pow(p2p1Distance,2))/ (2 * p2p1Distance);
    double y = (pow(r1,2) - pow(r3,2) + pow(i,2) + pow(j,2))/(2*j) - i*x/j;
    //result coordinates
    double finalX = point1.x+ x*ex.x + y*ey.x;
    double finalY = point1.y+ x*ex.y + y*ey.y;
    resultPose.x = finalX;
    resultPose.y = finalY;
    return resultPose;
}
void Accelerationfun(){
   sensors_event_t event; 
   accel.getEvent(&event);

   Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
   Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
   Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");
   Serial.println("m/s^2 ");
   delay(500);
  }
void COfun(){
  sensor_value = analogRead(AOUT);
  sensor_value_percentage = sensor_value / 1023 *100;
    if(sensor_value > 120){
      Serial.print("The CO percextage is ");
      Serial.print(sensor_value_percentage);
      Serial.println( "is over suggest value.");
    }
    else{
      Serial.print("The CO percextage is ");
      Serial.print(sensor_value_percentage);
      Serial.println( "is in suggest value.");
    }
  }
int main(){
      double r1,r2,r3;
      r1 = beaconABSdis[0];
      r2 = beaconABSdis[1];
      r3 = beaconABSdis[2];
      finalPose = trilateration(p[0],p[1],p[2],r1,r2,r3);
      cout<<"X:::  "<<finalPose.x<<endl;
      cout<<"Y:::  "<<finalPose.y<<endl;
       if(flag=1){
      for (int i = 0; i < 10 ; ++i) {
        for (int j = 0; j < 10; ++j) {
          if (beaconABSdis[j] > beaconABSdis[i]) {
            int temp = beaconABSdis[j];
            beaconABSdis[j] = beaconABSdis[i];
            beaconABSdis[i] = temp;
          }
        }
      }
      FirebaseJson updateData;
      FirebaseJson json;
        updateData.set("xvalue",finalPose.x);
      if (Firebase.updateNode(firebaseData, "/esp32location", updateData)) {
      
        Serial.println(firebaseData.dataPath());
      
        Serial.println(firebaseData.dataType());
      
        Serial.println(firebaseData.jsonString()); 
      
      } else {
        Serial.println(firebaseData.errorReason());
      }
        updateData.set("yvalue",finalPose.y);
       if (Firebase.updateNode(firebaseData, "/esp32location",updateData)) {
      
        Serial.println(firebaseData.dataPath());
      
        Serial.println(firebaseData.dataType());
      
        Serial.println(firebaseData.jsonString()); 
      
      } else {
        Serial.println(firebaseData.errorReason());
      }
    }
  }
void setup() {
  Serial.begin(9600);
  logMemory();
  byte* psdRamBuffer = (byte*)ps_malloc(500000);
  logMemory();
  free(psdRamBuffer);
  logMemory();
  ledcSetup(0,1E5,12);
  ledcAttachPin(12,0);
  Wire.begin();
  if(!accel.begin())
   {
      Serial.println("No ADXL345 sensor detected.");
      while(1);
   }
  flag =0;
  connectWifi();
  Firebase.begin("wisdomcrutch-30df7.firebaseio.com", "AIzaSyD_oSM8mydYEY5Sd589AnYZAln5x__gWOo");//apikey()
  //dht22
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
}

void loop() {
   for(int i=1;i<10;i++){
    p[i].x == 100.0;
    p[i].y == 100.0;
  }
   beaconranging();//信標測距功能
   DHT22fun();//溫溼度
   main();//三角定位、距離排序
   COfun();//一氧化碳偵測
   Accelerationfun();//加速度偵測
  }
