/****
*
* Experiment controler 
* Controling experiment stages and coin acceptor and returner
* (c) 2022 Wieslaw Bartkowski, University of Warsaw, Poland
*
****/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

#define LED_PIN 6
#define BUTTON_PIN 4
#define SERVO_PIN 9
#define COIN_PIN 2

#define LED_COUNT 4
#define BRIGHTNESS 20

#define buttonReactionTime 1000 //1s nie czesciej niz co tyle reaguje buton na wcisniecie
#define trustGameTimeLimit 10000 //10s

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);
Servo pusherServo;

int coinNum = 0;
volatile bool newCoin = false;
int lastCoinNum = 0;
int trustGameResult = 0;

// 1 - bez ruchu (czyli kiwa sie lewo prawo caly czas tak samo)
// 2 - ruch losowy
// 3 - nasladowanie ruchu badanego
// 4 - rownanie logistyczbe (to narazie nie)
int conditionNR = 0; //TODO zapamietanie w EPROM ktory warunek byl ostatnio badany i zaczyanien od niego 
//0 - wyjsciowa sytuacja badanie nie zaczete patyk po kalibracji nie aktywny - nie reaguje na ruchy
//1 - paryk aktywny - faza badania synchronizacji
//2 - trust game - patyk nie aktywny
//2 < powrt do stanu 0 - patyk nie aktywny wyswietlacz wygaszony 
int researchStage = 0;

unsigned long lastTimeButtonPressed = 0;
unsigned long lastCoinTime = 0;


void coinImpuls()
{
  newCoin = true;;
}

void pushCoins(int N) 
{
  pusherServo.attach(SERVO_PIN);

  for (int i = 0; i < N; i++) 
  {
    //wysuniete
    pusherServo.write(165);
    delay(1200);
    //wsuniete
    pusherServo.write(10);
    delay(1200);
  }

  pusherServo.detach();
}

void showConditionNR(int N)
{
  strip.clear();
  if (N>0) strip.fill(strip.Color(255, 120, 50, 155), 0, N);
  strip.show();
}

void setup()
{
  Serial.begin(115200); //9600

  pinMode(COIN_PIN, INPUT); //INPUT_PULLUP
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pusherServo.attach(SERVO_PIN);
  delay(100);
  attachInterrupt(0,coinImpuls, FALLING); //digitalPinToInterrupt(COIN_PIN)

  //wsuniecie - pozycja spoczynkowa
  pusherServo.write(10);
  delay(1000);
  //zeby nie brzeczalo
  pusherServo.detach();

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.clear();
  strip.show();
}

void loop()
{
  if(digitalRead(BUTTON_PIN) == LOW && millis()-lastTimeButtonPressed > buttonReactionTime) 
  {
    lastTimeButtonPressed = millis();
    if(0 == researchStage) 
    {
      //zaczyna sie stage 1
      researchStage++;
      conditionNR = conditionNR % 3;
      showConditionNR(conditionNR+1);
      //informacja o warunku do ESP32
      Serial.println("C"+String(conditionNR+1)+"#");
      conditionNR++; //jaki nastepny condution
    } 
    else if (1 == researchStage) 
    {
      //zaczyna sie stage 2
      researchStage++;
      //faza trust game oznaczona na zielono
      strip.fill(strip.Color(0, 255, 0, 0));
      strip.show();
      //sygnal konca fazy synchronizacji
      Serial.println("E"); 
      coinNum = 0;
    }
    //coinNum musi byc zero bo inaczej badany w trakcie trust jeszcze
    //jak nic nie wrzuci to tez mozna przerwac po coinNum zero
    else if (2 == researchStage && 0 == coinNum) 
    {
      // koniec badania - reset do stage 0
      researchStage = 0;
      strip.clear();
      strip.show();
      //wyslij wynik trust game
      Serial.println("T"+String(trustGameResult)+"#");
      trustGameResult = 0;
    }
  }

//===== obsluga przyjmowania i wyrzucania monet - działa niezaleznie =================  
  if (newCoin)
  {
    lastCoinTime = millis();
    coinNum++;
    delay(500);
    newCoin = false; //po delay bo przerwanie i tak robi newCoin=true 
    //(to obejscie inaczej czasami się myli
    //bo wiecej impulsow czasami niz wrzucnych monet 
    //TODO sprawdzic jak inczej to rozwiązac)
  }
  //jak nic wiecej przez trustGameTimeLimit to zwraca monety wedlug wzoru
  if(2 == researchStage && coinNum > 0 && millis()-lastCoinTime > trustGameTimeLimit) 
  {
    pushCoins(coinNum+1);
    trustGameResult = coinNum;
    coinNum = 0;
  }
//=====================================================================================

}
