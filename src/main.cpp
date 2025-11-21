#include <Arduino.h>
#include <Wire.h>
#include "DHT20.h"

// --- PINS ---
#define RED_PIN 15
#define GREEN_PIN 2
#define BUTTON_PIN 4
#define BUZZER_PIN 13

// --- SENSOR CONFIG ---
DHT20 dht;
unsigned long last_sensor_read = 0;
#define SENSOR_INTERVAL 2000 // Read every 2 seconds

// --- STATES ---
#define START_STATE 0
#define STUDY_STATE 1
#define BREAK_STATE 2

// --- BUZZER CONFIG (PWM) ---
#define BUZZER_CHANNEL 0
#define BUZZER_FREQ 1500 // 1500Hz Tone
#define BUZZER_RES 8

// --- CONFIG ---
// Set to 'true' for quick 5-second tests. Set 'false' for real 25-min timers.
#define TEST_MODE true 

// *** NEW: Study Time Tracking ***
unsigned long total_study_time_ms = 0; // Holds cumulative study time

unsigned long STUDY_MILLIS;
unsigned long BREAK_MILLIS;

// --- VARIABLES ---
int state = START_STATE;
unsigned long state_start_time;

void setup() {
  Serial.begin(115200);

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT); 
  
  // Setup Buzzer Tone
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0); // Ensure silent

  // 3. Initialize Sensor
  Wire.begin();
  dht.begin();
  Serial.println("DHT20 Sensor Initialized.");

  if (TEST_MODE) {
    STUDY_MILLIS = 5000; // 5 seconds
    BREAK_MILLIS = 3000; // 3 seconds
  } else {
    STUDY_MILLIS = 25 * 60 * 1000;
    BREAK_MILLIS = 5 * 60 * 1000;
  }

  state = START_STATE;
}

// Helper: Read Temperature & Humidity (Non-blocking)
void readEnvironment() {
  unsigned long current_time = millis();
  
  // Only read every 2 seconds (reading too fast causes errors)
  if (current_time - last_sensor_read >= SENSOR_INTERVAL) {
    last_sensor_read = current_time;
    
    // Request data from DHT20
    dht.read();
    float temperature = dht.getTemperature();
    float humidity = dht.getHumidity();

    // Check for errors
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Error reading sensor!");
    } else {
      Serial.print("Room Env -> Temp: ");
      Serial.print(temperature, 1);
      Serial.print(" °C | Humidity: ");
      Serial.print(humidity, 1);
      Serial.println(" %");
    }
  }
}

// Generic Buzz Function
void buzz() {
  Serial.println("Timer Up! Buzzing...");
  
  // 1. Turn off all lights during the buzzer transition
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);

  // 2. Turn Buzzer ON (1500Hz)
  ledcWrite(BUZZER_CHANNEL, 128); 
  
  // 3. Wait 5 seconds
  delay(2000); 
  
  // 4. Turn Buzzer OFF
  ledcWrite(BUZZER_CHANNEL, 0);
  
  Serial.println("Buzzing Finished. Switching Mode.");
}

void printTotalTime() {
  Serial.println("--------------------------------");
  Serial.print("TOTAL STUDY TIME: ");
  Serial.print(total_study_time_ms / 1000);
  Serial.println(" seconds");
  Serial.println("--------------------------------");
}

void loop() {
  unsigned long current_time = millis();

  // 1. READ SENSOR (Runs in background)
  readEnvironment();
  
  // 1. READ BUTTON (Active High)
  bool button_pressed = (digitalRead(BUTTON_PIN) == HIGH);

  // 2. STATE MACHINE
  switch(state){
    
    // --- START (Both ON) ---
    case START_STATE:
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, HIGH);

      if (button_pressed) {
        Serial.println("STARTING SESSION...");
        while(digitalRead(BUTTON_PIN) == HIGH); 
        state = STUDY_STATE;
        state_start_time = millis();
      }
      break;

    // --- STUDY (Red ON) ---
    case STUDY_STATE:
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);

      // CASE A: MANUALLY STOPPED
      if (button_pressed) {
        Serial.println("PAUSED/STOPPED.");
        
        // Calculate partial time
        unsigned long elapsed = current_time - state_start_time;
        total_study_time_ms += elapsed;
        
        // Print Total
        printTotalTime();

        while(digitalRead(BUTTON_PIN) == HIGH);
        state = START_STATE;
      }
      // CASE B: TIMER FINISHED
      else if (current_time - state_start_time >= STUDY_MILLIS) {
        Serial.println("Study Time Finished.");
        
        // Add full duration
        total_study_time_ms += STUDY_MILLIS;
        
        // Print Total
        printTotalTime();

        buzz(); 
        
        // Start Break
        state = BREAK_STATE;
        state_start_time = millis();
      }
      break;

    // --- BREAK (Green ON) ---
    case BREAK_STATE:
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH);

      if (button_pressed) {
        Serial.println("STOPPING...");
        while(digitalRead(BUTTON_PIN) == HIGH);
        state = START_STATE;
      }
      else if (current_time - state_start_time >= BREAK_MILLIS) {
        // *** BREAK OVER ***
        // New: Buzz here too!
        Serial.println("Break Time Finished.");
        buzz(); 
        
        // Back to Study
        state = STUDY_STATE;
        state_start_time = millis();
      }
      break;
  }
}