/****
*
* Simple Non-Humanoid Robot controler
* Single segment continus robot
* Controling robot movment and comunicating with IMU sensor and Data Recorder
* (c) 2022 Wieslaw Bartkowski, University of Warsaw, Poland
*
****/


#include <Arduino.h>
#include <AccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>

bool newSensorData = false;

//========ESP NOW================================================

//MAC addres of Data Recorder (ESP32-CAM :: 94:b9:7e:f9:e6:8c)
uint8_t CamAddress[] = {0x94, 0xB9, 0x7E, 0xF9, 0xE6, 0x8C};

// Structure to send data
// Must match the receiver structure
// messType jak w serial z UNO plus 'S' dla danych z sensora
typedef struct struct_message {
    char messType;
    float value;
    float YPR[3];
} struct_message;

typedef struct struct_sensor_message {
    float YPR[3];
} struct_sensor_message;

// Create a struct_message to hold data to send
struct_message dataToSend;

// Create a struct_message to hold incoming data
struct_sensor_message incomingData;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));

  newSensorData = true;
}

void setupEspNow() {
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, CamAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
//========ESP NOW END=============================================

// Send message via ESP-NOW to store on SD Card
void sendToCAM() {

  //czas rejestruje CAM wewnetrznie 

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(CamAddress, (uint8_t *) &dataToSend, sizeof(dataToSend));
   
}

#define STEP_X 26 //2
#define DIR_X 16 //5
#define END_X 13 //9

#define STEP_Y 25 //3
#define DIR_Y 27 //6
#define END_Y 5 //10

#define STEP_Z 17 //4
#define DIR_Z 14 //7
#define END_Z 23 //11

#define EN 12 //8

// 1/2 obrotu = 15.7 mm 
// S2209 domyslenie micorstep 1/8 (LOW, LOW);
#define MAX_X   200*10 
#define MAX_Y   200*10 
#define MAX_Z   200*10
#define MAX_MAX 200*8*2+200*4

#define KR 200*2 //400
#define MAX_RANGE 2*200*8

#define HOMING_SPEED 1000
#define HOMING_ACCEL 1000

#define NORMAL_SPEED 200*8*8 //200*8 = 1 obrot na sekunde 
#define NORMAL_ACCEL 200*8*20 // 20 obrotow / sek2

AccelStepper stepperX(AccelStepper::DRIVER, STEP_X, DIR_X); // DIR = HIGH - forward
AccelStepper stepperY(AccelStepper::DRIVER, STEP_Y, DIR_Y); // DIR = HIGH - forward
AccelStepper stepperZ(AccelStepper::DRIVER, STEP_Z, DIR_Z); // DIR = HIGH - forward

int d = -200 * 8 * 3; //3 obroty
// 1/2 obrotu = 15.7 mm 
// S2209 domyslenie micorstep 1/8 (LOW, LOW);

//0 - wyjsciowa sytuacja badanie nie zaczete patyk po kalibracji nie aktywny - nie reaguje na ruchy
//1 - paryk aktywny - faza badania synchronizacji
//2 - trust game - patyk nie aktywny
//2 < powrt do stanu 0 - patyk nie aktywny wyswietlacz wygaszony 
int researchStage = 0;

// 1 - bez ruchu (czyli kiwa sie lewo prawo caly czas tak samo)
// 2 - ruch losowy
// 3 - nasladowanie ruchu badanego
// 4 - rownanie logistyczbe (to narazie nie)
int researchCondition = 0;

//YAW PITC HROLL
float rY = 0;
float rP = 0;
float rR = 0;

unsigned long lastTimeRndMove = 0;

unsigned long resetPositionTime = 0;

float t_noMove = 0; //podstawa czasu dla ruchu w warunku 1

bool resetPosition = false;

long nextRndMoveTime = 0;

void serialEv() { 
  
  //komenda od kontrolera arduino
  //C1 C2 C3 - warunek badania
  //E - koniec badania synchronizacji gotowy do trust
  //T0 T1 ... T5 - liczba wrzuconych monet
  
  char command = Serial2.read();

  //musi być cos na koncu int np. # bo jak jest to koniec transmicji 
  //to parseInt() czeka na timeout
  //Serial.print("E1#") a nie Serial.print("E1") 

  if('C' == command) 
  {
    researchStage = 1;
    researchCondition = Serial2.parseInt();
     //wyslij warunek na SD (czas timestamp robi odbiornik)
    dataToSend.messType = command;
    dataToSend.value = researchCondition;
    sendToCAM();

    t_noMove = 0;
  }
  else if('E' == command) 
  {
    researchStage = 2;
    //wyslij czas na SD (czas timestamp robi odbiornik)
    dataToSend.messType = command;
    dataToSend.value = 0;
    sendToCAM();

    resetPosition = true; //patyk do pionu
    resetPositionTime = millis();
  }
  else if('T' == command) 
  {
    researchStage = 0;
    //wyslij wynik na SD
    dataToSend.messType = command;
    dataToSend.value = Serial2.parseInt();
    sendToCAM();
  }
}

void setup() {
  //do komunikacji z UNO :: na CNC 39 = SCL; 4 = Hold
  Serial2.begin(115200, SERIAL_8N1, 39, 4, false, 50);
  while (!Serial2);
  //do debug
  Serial.begin(115200);
  while (!Serial);
  
  pinMode(LED_BUILTIN, OUTPUT);

  setupEspNow();

  delay(100); 

  pinMode(END_X, INPUT_PULLUP);
  pinMode(END_Y, INPUT_PULLUP);
  pinMode(END_Z, INPUT_PULLUP);
 
  //X homing
  stepperX.enableOutputs();
  stepperX.setMaxSpeed(HOMING_SPEED);
  stepperX.setAcceleration(HOMING_ACCEL);
  stepperX.moveTo(d);
  while(digitalRead(END_X)) {
    stepperX.run();
  }
  stepperX.setCurrentPosition(0);
  stepperX.runToNewPosition(MAX_MAX); // 1 1/10 obrotu

  //Y homing
  stepperY.enableOutputs();
  stepperY.setMaxSpeed(HOMING_SPEED);
  stepperY.setAcceleration(HOMING_ACCEL);
  stepperY.moveTo(d);
  while(digitalRead(END_Y)) {
    stepperY.run();
  }
  stepperY.setCurrentPosition(0);
  stepperY.runToNewPosition(MAX_MAX); // 1 1/10 obrotu

  //Z homing
  stepperZ.enableOutputs();
  stepperZ.setMaxSpeed(HOMING_SPEED);
  stepperZ.setAcceleration(HOMING_ACCEL);
  stepperZ.moveTo(d);
  while(digitalRead(END_Z)) {
    stepperZ.run();
  }
  stepperZ.setCurrentPosition(0);
  stepperZ.runToNewPosition(MAX_MAX); // 1 1/10 obrotu

  //stan wyjsciowy, wszyski elinki napiete
  stepperX.runToNewPosition(MAX_MAX-MAX_X); 
  stepperY.runToNewPosition(MAX_MAX-MAX_Y); 
  stepperZ.runToNewPosition(MAX_MAX-MAX_Z); 

  //===============
  stepperX.setMaxSpeed(NORMAL_SPEED);
  stepperY.setMaxSpeed(NORMAL_SPEED);
  stepperZ.setMaxSpeed(NORMAL_SPEED);
  stepperX.setAcceleration(NORMAL_ACCEL);
  stepperY.setAcceleration(NORMAL_ACCEL);
  stepperZ.setAcceleration(NORMAL_ACCEL);
  
  lastTimeRndMove = millis();
}


#define RR 500

#define AA 30.0

void loop() {
  if (Serial2.available()) {
    serialEv();
  }

  if (1 == researchStage || resetPosition) 
  {

    if(newSensorData && !resetPosition) {
      newSensorData = false;

      dataToSend.messType = 'S'; //resend sensor data to SD
      dataToSend.value = 0;
      dataToSend.YPR[0] = incomingData.YPR[0];
      dataToSend.YPR[1] = incomingData.YPR[1];
      dataToSend.YPR[2] = incomingData.YPR[2];

      sendToCAM();

      // 3 - nasladowanie ruchu badanego
      if(3 == researchCondition) {
        rR = incomingData.YPR[2];
        rP = incomingData.YPR[1];
        rY = incomingData.YPR[0];
      } 
      // 2 - ruch losowy
      else if(2 == researchCondition) {
        float dR = random(-20, 20) / 10.0;
        float dP = random(-20, 20) / 10.0;
        float dY = random(-30, 30) / 10.0;
        rR += dR;
        rP += dP;
        rY += dY;
        rR = constrain(rR, -15.0, 15.0);
        rP = constrain(rP, -15.0, 15.0);
        rY = constrain(rY, 0.0, 360.0);
      }
      /// 1 - bez ruchu (czyli kiwa sie lewo prawo caly czas tak samo)
      else if(1 == researchCondition) {
        rR = 15.0*sin(t_noMove);
        rP = 0.0;
        rY = 0.0;
        //2*pi/20 - pelen cykl w 1 sek bo co 50ms czyli 20 Hz
        t_noMove += (PI/50.0); //pelen cykl w 5 sek,
      }

      dataToSend.messType = 'R'; //reakcja maszyny to SD
      dataToSend.value = 0;
      dataToSend.YPR[0] = stepperX.currentPosition() / 100.0;
      dataToSend.YPR[1] = stepperY.currentPosition() / 100.0;
      dataToSend.YPR[2] = stepperZ.currentPosition() / 100.0;

      sendToCAM();
    }

    if(resetPosition) {
      rR = 0.0;
      rP = 0.0;
      rY = 0.0;

      if(millis() - resetPositionTime > 3000) resetPosition = false;
    }
  
    //przeniesc obliczenia a i rr do nadajnika
    //zoptymalizowac nadajnik i odbiornik na transmisje binarna a nie ascii
    float dx = 2*RR*sin(2*radians(-rR));
    float dy = 2*RR*sin(2*radians(rP));
    float rr = sqrt(sq(dx) + sq(dy));
    if(rr>RR) rr = RR;
    float a = atan2(dy,dx) + radians(rY);

    float h1 = RR+rr*cos(a);
    float h2 = RR+rr*cos(a-radians(120));
    float h3 = RR+rr*cos(a+radians(120));

    #define MMM 3 //dla zwiekszenia precyzji przeliczenia z h na steps
    stepperX.moveTo(map(MMM*h1, 0, MMM*2*RR, KR, MAX_RANGE-KR)); 
    stepperY.moveTo(map(MMM*h2, 0, MMM*2*RR, KR, MAX_RANGE-KR));
    stepperZ.moveTo(map(MMM*h3, 0, MMM*2*RR, KR, MAX_RANGE-KR));


    stepperX.run();
    stepperY.run();
    stepperZ.run();
  }

}






