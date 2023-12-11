//Pengujian 3 Integrasi

#include "PPG.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>

//--Define Pins
int V_out = 14;
int MOSFET_1460NM = 33;
int MOSFET_1650NM = 25;
int SDA_OLED = 21;
int SCL_OLED = 22;
int PB_1460 = 26;
int PB_1650 = 32;

//--Variables 20nd algorithm
float aLPF[4] = {-1, 2.6548, -2.3669, 0.7075};
float bLPF[4] = {0.007, -0.0048, -0.0048, 0.007};
int n_reflection = 50;
int n_data = 3100;
int maxADS = 30;
int ref_1460 = -100;
int ref_1650 = -100;
int ref = 0;
float sinyal_PPG[3100] = {0};
float ID_1460 = 0;
float IS_1460 = 0;
float ID_1650 = 0;
float IS_1650 = 0;
float KNN_input = 0;
int n_train = 90;
int n_neighbors = 4;
float x_train[90] = {}; // if you want this data, contact me: syifakush@gmail.com
float y_train[90] = {}; // if you want this data, contact me: syifakush@gmail.com
int BG_estimation = 0;

//--Others Variable
volatile unsigned long DebounceTimer = 0;
volatile unsigned int delayTime = 500;
bool start_measurement = false;
bool on_BLE = false;
int n_raw = 0;
int sinyal = 0;
bool LED_1650;

//--OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//--Button
void IRAM_ATTR turnBLE(){
  if (millis() - DebounceTimer > delayTime){
    on_BLE = !on_BLE;
  }
  DebounceTimer = millis();
}
void IRAM_ATTR startMeasuring(){
  if (millis() - DebounceTimer > delayTime){
    start_measurement = !start_measurement;
  }
  DebounceTimer = millis();
}

//--BLE-------------------------------------------------------------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLEServer* pServer = NULL;
BLEService *pService = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLEAdvertising *pAdvertising = NULL;
bool deviceConnected = false;
bool finishMeasure = false;

// check whether the device is already connected or not
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
class MyCharacteristicsCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue(); //send anything
      if (value.length() > 0) {
        start_measurement = true;
      }
    }
};
//---------------------------------------------------------------



//--Timer data acquisition
hw_timer_t *My_timer_1 = NULL;
void IRAM_ATTR onTimerGetDataAnalog(){
 sinyal = analogRead(V_out);
 if (n_raw >= 1000){
  sinyal_PPG[n_raw+49] = sinyal;
 }
 else if(n_raw < 200 & sinyal > ref_1460){
  ref = sinyal;
 }
 n_raw++;
}

//--Membuat objek PPG
PPG ppg(n_data, &sinyal_PPG[0]);


void setup() {
  Serial.begin(115200);

  //-------------------------------------------------------------------
  BLEDevice::init("BLUEMON");
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID,
                       BLECharacteristic::PROPERTY_READ  |
                       BLECharacteristic::PROPERTY_WRITE |
                       BLECharacteristic::PROPERTY_NOTIFY );             
  pCharacteristic->setCallbacks(new MyCharacteristicsCallbacks());
  
  pService->start();
  
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  //-------------------------------------------------------------------
  
  //-------------------------------------------------------------------
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("BLUMON");
  display.display(); 
  delay(100);
  //-------------------------------------------------------------------

  //-------------------------------------------------------------------
  pinMode(V_out, INPUT);
  pinMode(MOSFET_1460NM, OUTPUT);
  pinMode(MOSFET_1650NM, OUTPUT);
  pinMode(PB_1460, INPUT);
  pinMode(PB_1650, INPUT);
  //-------------------------------------------------------------------

  //-------------------------------------------------------------------
  My_timer_1 = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer_1, &onTimerGetDataAnalog, true);
  timerAlarmWrite(My_timer_1, 5000, true); //200 Hz Fs
  //-------------------------------------------------------------------

  //-------------------------------------------------------------------
  attachInterrupt(PB_1460, turnBLE, RISING);
  attachInterrupt(PB_1650, startMeasuring, RISING);
  digitalWrite(MOSFET_1460NM, LOW);
  digitalWrite(MOSFET_1650NM, LOW);
  //-------------------------------------------------------------------
  
  
}

void loop() {

  if (start_measurement){
    display.clearDisplay();
    display.setCursor(5, 10);
    display.setTextSize(1);
    display.println("Pengkuran");
    display.println("Dimulai");
    display.display();
    delay(100);
    display.clearDisplay();
    display.setCursor(5, 10);
    display.setTextSize(1);
    display.println("...");
    display.display();

    start_measurement = false;
    LED_1650 = true;
    n_raw = 0;
    timerAlarmEnable(My_timer_1);
    digitalWrite(MOSFET_1460NM, HIGH);
    digitalWrite(MOSFET_1650NM, LOW);
  }

  if (n_raw >= 4000 && LED_1650){
    timerAlarmDisable(My_timer_1);
    digitalWrite(MOSFET_1460NM, LOW);
    digitalWrite(MOSFET_1650NM, LOW);
    ref_1460 = ref;
    ppg.signalReflection(n_reflection);
    ppg.digitalFilter(3, &aLPF[0], &bLPF[0]);
    ppg.ADS(n_reflection, n_data-n_reflection, maxADS);
    ppg.meanIDIS(ppg.detected_PV);
    ID_1460 = ppg.meanID;
    IS_1460 = ppg.meanIS;

    LED_1650 = false;
    n_raw = 0;
    timerAlarmEnable(My_timer_1);
    digitalWrite(MOSFET_1460NM, LOW);
    digitalWrite(MOSFET_1650NM, HIGH);
  }
  else if (n_raw >= 4000 & !LED_1650){
    n_raw = 0;
    timerAlarmDisable(My_timer_1);
    digitalWrite(MOSFET_1460NM, LOW);
    digitalWrite(MOSFET_1650NM, LOW);
    ref_1650 = ref;
    ppg.signalReflection(n_reflection);
    ppg.digitalFilter(3, &aLPF[0], &bLPF[0]);
    ppg.ADS(n_reflection, n_data-n_reflection, maxADS);
    ppg.meanIDIS(ppg.detected_PV);
    ID_1650 = ppg.meanID;
    IS_1650 = ppg.meanIS;
    
    KNN_input = log10((ref_1650-ID_1650)/(ref_1650-IS_1650));
    KNN_input = KNN_input/log10((ref_1460-ID_1460)/(ref_1460-IS_1460));
    BG_estimation = ppg.KNN(n_neighbors, n_train, &x_train[0], &y_train[0], KNN_input);
    
    display.clearDisplay();
    display.setCursor(5, 10);
    display.setTextSize(1);
    display.println("Blood Glucose:");
    display.setCursor(5, 40); 
    display.setTextSize(2);
    display.print(BG_estimation); display.println(" mg/dl");
    display.display(); 

    finishMeasure = true;
  }

  if (deviceConnected) {
    if (finishMeasure){
      Serial.print(BG_estimation);
      pCharacteristic->setValue(BG_estimation);
      pCharacteristic->notify();
      finishMeasure = false;
      BG_estimation = 0;
    }  
  }
  
}
