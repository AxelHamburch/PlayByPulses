#include <Arduino.h>
#include "DFRobotDFPlayerMini.h"
#include "HardwareSerial.h"

#define PULSE_PIN 15
#define PULSE_TIMEOUT 500

HardwareSerial SerialPort2(2); // use UART2
DFRobotDFPlayerMini myDFPlayer;

// Touch PIN 4
const int touchPin = 4;
const int threshold = 20;
int touchValue;


void setup()
{
  Serial.begin(115200);

  pinMode(PULSE_PIN, INPUT_PULLUP);                          // pulses input

  // Initialisiere DFPlayer
  SerialPort2.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini"));
  Serial.println(F("Initialisiere DFPlayer ... (Dauert bis zu 5 Sek)"));
  if (!myDFPlayer.begin(SerialPort2)) {  //Use serial to communicate with mp3.
    Serial.println(F("Kann nicht starten: "));
    Serial.println(F("1. Kabelverbindung pruefen"));
    Serial.println(F("2. SD Karte korrekt vorbereitet und eingesteckt"));
    while (true) {
      delay(0); // Zeile wird nur für ESP8266 benötigt
    }
  }
  Serial.println(F("DFPlayer Mini Startklar"));
  myDFPlayer.volume(30);  //Einstellung Lautstärke zwischen 0 bis max 30
}


// DFPlayer Diagnose
void printDetail(uint8_t type, int value) {
  switch (type) {
  case TimeOut:
    Serial.println(F("Time Out!"));
    break;
  case WrongStack:
    Serial.println(F("Stack Wrong!"));
    break;
  case DFPlayerCardInserted:
    Serial.println(F("Card Inserted!"));
    break;
  case DFPlayerCardRemoved:
    Serial.println(F("Card Removed!"));
    break;
  case DFPlayerCardOnline:
    Serial.println(F("Card Online!"));
    break;
  case DFPlayerUSBInserted:
    Serial.println("USB Inserted!");
    break;
  case DFPlayerUSBRemoved:
    Serial.println("USB Removed!");
    break;
  case DFPlayerPlayFinished:
    Serial.print(F("Number:"));
    Serial.print(value);
    Serial.println(F(" Play Finished!"));
    break;
  case DFPlayerError:
    Serial.print(F("DFPlayerError:"));
    switch (value) {
    case Busy:
      Serial.println(F("Card not found"));
      break;
    case Sleeping:
      Serial.println(F("Sleeping"));
      break;
    case SerialWrongStack:
      Serial.println(F("Get Wrong Stack"));
      break;
    case CheckSumNotMatch:
      Serial.println(F("Check Sum Not Match"));
      break;
    case FileIndexOut:
      Serial.println(F("File Index Out of Bound"));
      break;
    case FileMismatch:
      Serial.println(F("Cannot Find File"));
      break;
    case Advertise:
      Serial.println(F("In Advertise"));
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}


// Pulserkennung
unsigned int detect_pulses()
{
  unsigned long last_pulse;
  unsigned int pulses;
  bool prev_value;
  bool read_value;
  unsigned long entering_time;
  unsigned long current_time;

  Serial.println("Starting pulse detection...");

  pulses = 0;
  read_value = 1;
  prev_value = !digitalRead(PULSE_PIN);
  entering_time = millis();
  while (true)
  {
    read_value = !digitalRead(PULSE_PIN);
    if (read_value != prev_value && !read_value)
    {
      delay(35);
      read_value = !digitalRead(PULSE_PIN);
      if (!read_value)
      {
        pulses++;
        last_pulse = millis();
        Serial.println("pulses++");
      }
    }
    prev_value = read_value;
    current_time = millis();
    if (pulses > 0 && (current_time - last_pulse > PULSE_TIMEOUT))
    {
      break;
    }
  }
  return (pulses);
}



void loop()
{
  // für dflplayer - Touch PIN 4 Start
  touchValue = touchRead(touchPin);
  if (touchValue < threshold) {
    myDFPlayer.play(1);  //Spiele mp3 Datei Nummer 1
    delay(2000); // Touch Entpreller 
  }
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }


  unsigned int pulses = 0;

  pulses = detect_pulses(); // is a loop to detect pulses, will return the amount of pulses
  Serial.println("Pulses: " + String(pulses));
  if (pulses >= 1 && pulses <= 1)
  {
    myDFPlayer.play(1);  //Spiele mp3 Datei Nummer 1
    Serial.println("Start DFPlayer mit Nr.1");
  }
  else if (pulses >= 2 && pulses <= 10)
  {
    myDFPlayer.play(2);  //Spiele mp3 Datei Nummer 2
    Serial.println("Start DFPlayer mit Nr.2");
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
}



