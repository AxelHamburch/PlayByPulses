#include <Arduino.h>
#include "DFRobotDFPlayerMini.h"
#include "HardwareSerial.h"

#define PULSE_PIN 15
#define PULSE_TIMEOUT 200

#define RXD2 16
#define TXD2 17

HardwareSerial SerialPort1(2); // use UART1 (UART0 is used for terminal, UART2 ist occupied)
DFRobotDFPlayerMini myDFPlayer;

// Touch PIN 4
const int touchPin = 4;
const int threshold = 20;
int touchValue;

// Zustände der Pulserkennung
enum PulseDetectionState {
  WAIT_FOR_PULSE,
  DEBOUNCE,
  PULSE_RECEIVED
};
// Variablen für die Pulserkennung
PulseDetectionState currentState = WAIT_FOR_PULSE;
unsigned long last_pulse = 0;
unsigned int pulses = 0;
unsigned int final_pulse_count = 0;
bool prev_value = !digitalRead(PULSE_PIN);
unsigned long debounce_start_time = 0;


void setup()
{
  Serial.begin(115200);
  delay(500);
  pinMode(PULSE_PIN, INPUT_PULLUP);                          // pulses input

  // Initialisiere DFPlayer
  SerialPort1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd0 is on pin: " + String(TX));
  Serial.println("Serial Rxd0 is on pin: " + String(RX));
  Serial.println("Serial Txd1 is on pin: " + String(TX1));
  Serial.println("Serial Rxd1 is on pin: " + String(RX1));
  Serial.println("Serial Txd2 is on pin: " + String(TX2));
  Serial.println("Serial Rxd2 is on pin: " + String(RX2));
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini"));
  Serial.println(F("Initialisiere DFPlayer ... (Dauert bis zu 5 Sek)"));
  if (!myDFPlayer.begin(SerialPort1)) {  //Use serial to communicate with mp3.
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



void loop()
{
  // DFLPlayer - Touch PIN 4 Start
  touchValue = touchRead(touchPin);
  if (touchValue < threshold) {
    myDFPlayer.play(1);  //Spiele mp3 Datei Nummer 1
    delay(2000); // Touch Entpreller 
  }

  // Pulserkennung
  bool read_value = !digitalRead(PULSE_PIN);
  unsigned long current_time = millis();
  switch (currentState) {
  case WAIT_FOR_PULSE:
    if (read_value != prev_value && !read_value) {
      debounce_start_time = current_time;
      currentState = DEBOUNCE;
    }
    break;
  case DEBOUNCE:
    if (current_time - debounce_start_time >= 35) {
      read_value = !digitalRead(PULSE_PIN);
      if (!read_value) {
        pulses++;
        last_pulse = current_time;
        Serial.println("pulse+");
        currentState = PULSE_RECEIVED;
      }
      else {
        currentState = WAIT_FOR_PULSE;
      }
    }
    break;
  case PULSE_RECEIVED:
    if (current_time - last_pulse > PULSE_TIMEOUT) {
      final_pulse_count = pulses;
      Serial.print("Total pulses: ");
      Serial.println(final_pulse_count);
      // Reset for next pulse detection sequence
      pulses = 0;
      currentState = WAIT_FOR_PULSE;
    }
    break;
  }
  prev_value = read_value;


  if (final_pulse_count >= 1 && final_pulse_count <= 1)
  {
    myDFPlayer.play(1);  //Spiele mp3 Datei Nummer 1
    Serial.println("Start DFPlayer mit Nr.1");
    Serial.println("Pulses: " + String(final_pulse_count));
    unsigned int final_pulse_count = 0;
  }
  else if (final_pulse_count >= 2 && final_pulse_count <= 10)
  {
    myDFPlayer.play(2);  //Spiele mp3 Datei Nummer 2
    Serial.println("Start DFPlayer mit Nr.2");
    Serial.println("Pulses: " + String(final_pulse_count));
    unsigned int final_pulse_count = 0;
  }

}



