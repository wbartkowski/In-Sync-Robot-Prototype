/****
*
* IMU senspr
* Colectiong data form IMU sensor and sanding to robot controler 
* (c) 2022 Wieslaw Bartkowski, University of Warsaw, Poland
*
****/

#include <Arduino.h>

#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//========ESP NOW================================================
#include <esp_now.h>
#include <WiFi.h>

//MAC addres of the robot (ESP32 controling robot movments)
uint8_t totterAddress[] = {0xE0, 0xE2, 0xE6, 0x0B, 0x81, 0x44};

typedef struct struct_sensor_message {
    float YPR[3];
} struct_sensor_message;

// Create a struct_message to hold data to send
struct_sensor_message dataToSend;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setupEspNow() {
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  // Once ESPNow is successfully Init, we will register for Send CB
  // to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, totterAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
}
//========ESP NOW END=============================================

void sendToTotter() {

  // Set values to send
  //dataToSend.time = (millis() - startTime) / 1000.0; //sekundy
  //dataToSend.value = state;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(totterAddress, (uint8_t *) &dataToSend, sizeof(dataToSend));
   
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
  }
}

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50) //100 - sprawdzic czy nie jest domyslnie 10Hz

Adafruit_BNO055 bno = Adafruit_BNO055();

bool blink = false;

void setup(void)
{
  Serial.begin(115200);
  while (!Serial);
  
  setupEspNow();
  
  Wire.setPins(4,5);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print("no BNO055 detected!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  pinMode(LED_BUILTIN, OUTPUT);
}

uint8_t sys, gyro, accel, mag = 0;

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  //pelna kalibracja jak wszysko jest rowne 3
  //procedura kalibracji: https://youtu.be/Bw0WuAyGsnY
  if(sys!=3 || gyro!=3 || accel!=3 || mag!=3) {
    /* calibration data for each sensor. */
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    
    Serial.print(F("Calibration: "));
    Serial.print(sys, DEC);
    Serial.print(F(" "));
    Serial.print(gyro, DEC);
    Serial.print(F(" "));
    Serial.print(accel, DEC);
    Serial.print(F(" "));
    Serial.println(mag, DEC);  

    blink = !blink;
    digitalWrite(LED_BUILTIN, blink);
    //koniec kalibracji
    if(sys==3 && gyro==3 && accel==3 && mag==3) {
      for (int i = 0; i < 3; i++)
      {
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
      }
    }
  } else {
    dataToSend.YPR[0] = event.orientation.x; //YAW (HEADING)
    dataToSend.YPR[1] = -event.orientation.z; //PITCH minus zeby bylo jak w mpu6050
    dataToSend.YPR[2] = -event.orientation.y; //ROLL minus zeby bylo jak w mpu6050

    sendToTotter();
    
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}



