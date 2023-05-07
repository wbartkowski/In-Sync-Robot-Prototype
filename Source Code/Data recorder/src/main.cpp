/****
*
* Data recorder is colecting data from experiment procedure and IMU sensor data
* (c) 2022 Wieslaw Bartkowski, University of Warsaw, Poland
*
****/

#include <Arduino.h>

#include "FS.h"
#include "SD_MMC.h"

#include <esp_now.h>
#include <WiFi.h>

unsigned long startTime = 0;
int participantNR = 1;

//====FS=========================================
// Read a file in SD card
void readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while (file.available())
    {
        Serial.write(file.read());
    }
}

// Write a file in SD card
void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }

    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
}

// Append to the end of file in SD card
void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("Message appended");
    }
    else
    {
        Serial.println("Append failed");
    }
}
//====FS END======================================

//========ESP NOW================================================
//Structure to receive data
// Must match the sender structure
// messType jak w serial z UNO plus 'S' dla danych z sensora
typedef struct struct_message {
    char messType;
    float value;
    float YPR[3];
} struct_message;

// Create a struct_message to hold incoming data
struct_message incomingData;

#define DSTR_W 9
#define DSTR_P 2
// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));

  if('C' == incomingData.messType) {
    startTime = millis(); 
    float time = (millis() - startTime) / 1000.0; //sekundy
    String s = "C ";
    char buffer[10]; // Enough room for the digits you want and more to be safe
    dtostrf(time, DSTR_W, DSTR_P, buffer);
    s = s+buffer;
    dtostrf(incomingData.value, DSTR_W, DSTR_P, buffer);
    s = s+buffer;
    dtostrf(millis() / 1000.0, DSTR_W, DSTR_P, buffer); //Global Time
    s = s+buffer;
    dtostrf(participantNR, DSTR_W, 0, buffer); //Numer badanego w danym dniu o ile nie padnie zasilanie
    s = s+buffer+'\n';
    appendFile(SD_MMC, "/data.txt", s.c_str());
    participantNR++;
  } 
  else if('E' == incomingData.messType) {
    float time = (millis() - startTime) / 1000.0; //sekundy
    String s = "E ";
    char buffer[10]; // Enough room for the digits you want and more to be safe
    dtostrf(time, DSTR_W, DSTR_P, buffer);
    s = s+buffer;
    dtostrf(millis() / 1000.0, DSTR_W, DSTR_P, buffer); //Global Time
    s = s+buffer+'\n';;
    appendFile(SD_MMC, "/data.txt", s.c_str());
  } 
  else if('T' == incomingData.messType) {
    float time = (millis() - startTime) / 1000.0; //sekundy
    String s = "T ";
    char buffer[10]; // Enough room for the digits you want and more to be safe
    dtostrf(time, DSTR_W, DSTR_P, buffer);
    s = s+buffer;
    dtostrf(incomingData.value, DSTR_W, DSTR_P, buffer);
    s = s+buffer;
    dtostrf(millis() / 1000.0, DSTR_W, DSTR_P, buffer); //Global Time
    s = s+buffer+'\n';;
    appendFile(SD_MMC, "/data.txt", s.c_str());
  } 
  else if('S' == incomingData.messType) {
    float time = (millis() - startTime) / 1000.0; //sekundy
    String s = "S ";
    char buffer[10]; // Enough room for the digits you want and more to be safe
    dtostrf(time, DSTR_W, DSTR_P, buffer);
    s = s+buffer;
    for(int i = 0; i<3; i++) {
      dtostrf(incomingData.YPR[i], DSTR_W, DSTR_P, buffer);
      s = s+buffer;
    }
    s = s+'\n';
    appendFile(SD_MMC, "/data.txt", s.c_str());
  }
  else if('R' == incomingData.messType) {
    float time = (millis() - startTime) / 1000.0; //sekundy
    String s = "R ";
    char buffer[10]; // Enough room for the digits you want and more to be safe
    dtostrf(time, DSTR_W, DSTR_P, buffer);
    s = s+buffer;
    for(int i = 0; i<3; i++) {
      dtostrf(incomingData.YPR[i], DSTR_W, DSTR_P, buffer);
      s = s+buffer;
    }
    s = s+'\n';
    appendFile(SD_MMC, "/data.txt", s.c_str());
  }
  //  Serial.print(s);
}

void setupEspNow() {
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
//========ESP NOW END=============================================

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    delay(1000);

    setupEspNow();

    delay(1000);

    Serial.println("SDcard Testing....");

    if (!SD_MMC.begin())
    {
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD_MMC.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println("No SD_MMC card attached");
        return;
    }

    if(!SD_MMC.exists("/data.txt")) {
        writeFile(SD_MMC, "/data.txt", "");
    }
}

void loop()
{
    delay(10000);
}