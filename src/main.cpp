#include <Arduino.h>
#include "DFRobotDFPlayerMini.h"
#include "HardwareSerial.h"

#define PULSE_PIN 15
#define PULSE_TIMEOUT 200

#define COIN_PIN 13
#define COIN_TIMEOUT 200

HardwareSerial SerialPort2(2); // use UART2 (UART0 is used for terminal, UART1 is difficult)
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

// Variablen für die Münzprüfung
PulseDetectionState coinState = WAIT_FOR_PULSE;
unsigned long last_coin_pulse = 0;
unsigned int coin_pulses = 0;
unsigned int final_coin_pulse_count = 0;
bool prev_coin_value = digitalRead(COIN_PIN); // Invertiertes Signal
unsigned long coin_debounce_start_time = 0;

void setup()
{
  Serial.begin(115200);
  delay(500);
  pinMode(PULSE_PIN, INPUT_PULLUP); // pulses input
  pinMode(COIN_PIN, INPUT_PULLUP);  // coins input

  // Initialisiere DFPlayer
  SerialPort2.begin(9600, SERIAL_8N1, 16, 17); // RX, TX (UART2)
  Serial.println("Serial Txd0 is on pin: " + String(TX));
  Serial.println("Serial Rxd0 is on pin: " + String(RX));
  Serial.println("Serial Txd1 is on pin: " + String(TX1));
  Serial.println("Serial Rxd1 is on pin: " + String(RX1));
  Serial.println("Serial Txd2 is on pin: " + String(TX2));
  Serial.println("Serial Rxd2 is on pin: " + String(RX2));
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini"));
  Serial.println(F("Initialisiere DFPlayer ... (Dauert bis zu 5 Sek)"));
  if (!myDFPlayer.begin(SerialPort2)) {  // Use serial to communicate with mp3.
    Serial.println(F("Kann nicht starten: "));
    Serial.println(F("1. Kabelverbindung prüfen"));
    Serial.println(F("2. SD Karte korrekt vorbereitet und eingesteckt"));
    while (true) {
      delay(0); // Zeile wird nur für ESP8266 benötigt
    }
  }
  Serial.println(F("DFPlayer Mini Startklar"));
  myDFPlayer.volume(30);  // Einstellung Lautstärke zwischen 0 bis max 30
}

void loop()
{
  // DFPlayer - Touch PIN 4 Start
  touchValue = touchRead(touchPin);
  if (touchValue < threshold) {
    myDFPlayer.play(4);  // Spiele mp3 Datei Nummer 4
    Serial.print("Touch recognized, play mp3");
    delay(1000); // Touch Entpreller 
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
        currentState = WAIT_FOR_PULSE; // Zurück zu WAIT_FOR_PULSE, um weitere Impulse zu erfassen
      }
      else {
        currentState = WAIT_FOR_PULSE;
      }
    }
    break;
  }
  prev_value = read_value;

  // Münzpulserkennung
  bool coin_read_value = digitalRead(COIN_PIN); // Invertiertes Signal
  unsigned long coin_current_time = millis();
  switch (coinState) {
  case WAIT_FOR_PULSE:
    if (coin_read_value != prev_coin_value && coin_read_value) {
      coin_debounce_start_time = coin_current_time;
      coinState = DEBOUNCE;
    }
    break;
  case DEBOUNCE:
    if (coin_current_time - coin_debounce_start_time >= 35) {
      coin_read_value = digitalRead(COIN_PIN);
      if (coin_read_value) {
        coin_pulses++;
        last_coin_pulse = coin_current_time;
        Serial.println("coin pulse+");
        coinState = WAIT_FOR_PULSE; // Zurück zu WAIT_FOR_PULSE, um weitere Impulse zu erfassen
      }
      else {
        coinState = WAIT_FOR_PULSE;
      }
    }
    break;
  }
  prev_coin_value = coin_read_value;

  // Auswertung der Pulse, spiel File des DFPlayer
  if (pulses > 0 && (current_time - last_pulse > PULSE_TIMEOUT)) {
    final_pulse_count = pulses;
    Serial.print("Total pulses: ");
    Serial.println(final_pulse_count);
    pulses = 0; // Reset für die nächste Impulsreihe
    // Trigger DFPlayer actions based on final_pulse_count
    if (final_pulse_count == 1) {
      myDFPlayer.play(1);
      Serial.println("Start DFPlayer mit Nr.1");
    }
    else if (final_pulse_count == 2) {
      myDFPlayer.play(2);
      Serial.println("Start DFPlayer mit Nr.2");
    }
    else if (final_pulse_count == 3) {
      myDFPlayer.play(3);
      Serial.println("Start DFPlayer mit Nr.3");
    }
    final_pulse_count = 0;
  }

  // Auswertung der Münzpulse, spiel File des DFPlayer
  if (coin_pulses > 0 && (coin_current_time - last_coin_pulse > COIN_TIMEOUT)) {
    final_coin_pulse_count = coin_pulses;
    Serial.print("Total coin pulses: ");
    Serial.println(final_coin_pulse_count);
    coin_pulses = 0; // Reset für die nächste Impulsreihe
    // Trigger DFPlayer actions based on final_coin_pulse_count
    if (final_coin_pulse_count == 2) {
      myDFPlayer.play(4);
      Serial.println("Start DFPlayer mit Nr.4");
    }
    else if (final_coin_pulse_count == 3) {
      myDFPlayer.play(5);
      Serial.println("Start DFPlayer mit Nr.5");
    }
    else if (final_coin_pulse_count == 4) {
      myDFPlayer.play(6);
      Serial.println("Start DFPlayer mit Nr.6");
    }
    final_coin_pulse_count = 0;
  }
}
