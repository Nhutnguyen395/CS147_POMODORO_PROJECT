#include <Arduino.h>
#include <Wire.h>
#include "DHT20.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

// --- PINS ---
#define RED_PIN 15
#define GREEN_PIN 2
#define BUTTON_PIN 4
#define BUZZER_PIN 13

// WIFI and AZURE CONFIG
#define WIFI_SSID "alvis"
#define WIFI_PASSWORD "abcdefgh"

// Azure Configuration
String iothubName = "cs147group11IoTHub";
String deviceName = "146esp32";
String url = "https://" + iothubName + ".azure-devices.net/devices/" + deviceName + "/messages/events?api-version=2021-04-12";
String sasToken = "SharedAccessSignature sr=cs147group11IoTHub.azure-devices.net&sig=t45FKi44kYSmYE3y5xze3PzC2ToNeFRdc0BP1%2BkZg1w%3D&se=1764120276&skn=iothubowner";

// DigiCert Global Root G2 (Standard for Azure)
const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIEtjCCA56gAwIBAgIQCv1eRG9c89YADp5Gwibf9jANBgkqhkiG9w0BAQsFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
"MjAeFw0yMjA0MjgwMDAwMDBaFw0zMjA0MjcyMzU5NTlaMEcxCzAJBgNVBAYTAlVT\n" \
"MR4wHAYDVQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xGDAWBgNVBAMTD01TRlQg\n" \
"UlMyNTYgQ0EtMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMiJV34o\n" \
"eVNHI0mZGh1Rj9mdde3zSY7IhQNqAmRaTzOeRye8QsfhYFXSiMW25JddlcqaqGJ9\n" \
"GEMcJPWBIBIEdNVYl1bB5KQOl+3m68p59Pu7npC74lJRY8F+p8PLKZAJjSkDD9Ex\n" \
"mjHBlPcRrasgflPom3D0XB++nB1y+WLn+cB7DWLoj6qZSUDyWwnEDkkjfKee6ybx\n" \
"SAXq7oORPe9o2BKfgi7dTKlOd7eKhotw96yIgMx7yigE3Q3ARS8m+BOFZ/mx150g\n" \
"dKFfMcDNvSkCpxjVWnk//icrrmmEsn2xJbEuDCvtoSNvGIuCXxqhTM352HGfO2JK\n" \
"AF/Kjf5OrPn2QpECAwEAAaOCAYIwggF+MBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYD\n" \
"VR0OBBYEFAyBfpQ5X8d3on8XFnk46DWWjn+UMB8GA1UdIwQYMBaAFE4iVCAYlebj\n" \
"buYP+vq5Eu0GF485MA4GA1UdDwEB/wQEAwIBhjAdBgNVHSUEFjAUBggrBgEFBQcD\n" \
"AQYIKwYBBQUHAwIwdgYIKwYBBQUHAQEEajBoMCQGCCsGAQUFBzABhhhodHRwOi8v\n" \
"b2NzcC5kaWdpY2VydC5jb20wQAYIKwYBBQUHMAKGNGh0dHA6Ly9jYWNlcnRzLmRp\n" \
"Z2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RHMi5jcnQwQgYDVR0fBDswOTA3\n" \
"oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0R2xvYmFsUm9v\n" \
"dEcyLmNybDA9BgNVHSAENjA0MAsGCWCGSAGG/WwCATAHBgVngQwBATAIBgZngQwB\n" \
"AgEwCAYGZ4EMAQICMAgGBmeBDAECAzANBgkqhkiG9w0BAQsFAAOCAQEAdYWmf+AB\n" \
"klEQShTbhGPQmH1c9BfnEgUFMJsNpzo9dvRj1Uek+L9WfI3kBQn97oUtf25BQsfc\n" \
"kIIvTlE3WhA2Cg2yWLTVjH0Ny03dGsqoFYIypnuAwhOWUPHAu++vaUMcPUTUpQCb\n" \
"eC1h4YW4CCSTYN37D2Q555wxnni0elPj9O0pymWS8gZnsfoKjvoYi/qDPZw1/TSR\n" \
"penOgI6XjmlmPLBrk4LIw7P7PPg4uXUpCzzeybvARG/NIIkFv1eRYIbDF+bIkZbJ\n" \
"QFdB9BjjlA4ukAg2YkOyCiB8eXTBi2APaceh3+uBLIgLk8ysy52g2U3gP7Q26Jlg\n" \
"q/xKzj3O9hFh/g==\n" \
"-----END CERTIFICATE-----\n"; \

// --- SENSOR CONFIG ---
DHT20 dht;
unsigned long lastTelemetryTime = 0;
#define TELEMETRY_INTERVAL 3000

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

  // 1. Hardware init
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT); 
  
  // Setup Buzzer Tone
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ, BUZZER_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0); // Ensure silent

  // 2. Initialize Sensor
  Wire.begin();
  dht.begin();
  Serial.println("Hardware Initialized.");

    // 3. Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Block until connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  if (TEST_MODE) {
    STUDY_MILLIS = 5000; // 5 seconds
    BREAK_MILLIS = 10000; // 3 seconds
  } else {
    STUDY_MILLIS = 25 * 60 * 1000;
    BREAK_MILLIS = 5 * 60 * 1000;
  }

  state = START_STATE;
}

// Send data to Azure
void sendTelemetry(){
  dht.read();
  float temp = dht.getTemperature();
  float hum = dht.getHumidity();

  // Create JSON Payload
  ArduinoJson::JsonDocument doc;

  // 1. Environment Data
  if (!isnan(temp) && !isnan(hum)) {
    doc["temperature"] = temp;
    doc["humidity"] = hum;
  } else {
    doc["temperature"] = "null"; 
  }

  // 2. Study Time Data (Seconds)
  // This counts up as you complete sessions or pause them.
  doc["total_study_seconds"] = total_study_time_ms / 1000;

  // 3. Optional Status (Helpful for filtering on Dashboard)
  if(state == STUDY_STATE){
    doc["status"] = "studying";
  } else if(state == BREAK_STATE){
    doc["status"] = "break";
  }

  char buffer[256];
  serializeJson(doc, buffer, sizeof(buffer));

  // Send HTTPS POST to Azure IoT Hub
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClientSecure client;
    client.setCACert(root_ca);
    HTTPClient http;
    
    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", sasToken);
    
    int httpCode = http.POST(buffer);
    
    if (httpCode == 204) {
      Serial.print("Azure Sent: ");
      Serial.println(buffer);
    } else {
      Serial.print("Azure Error: ");
      Serial.println(httpCode);
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected!");
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
  
  // 3. Wait 2 seconds
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

  // 1. Send Telemetry (Every 3 seconds in background)
  if (current_time - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    sendTelemetry();
    lastTelemetryTime = current_time;
  }
  
   // 2. Read Button
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
        sendTelemetry();
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

        // update azure
        sendTelemetry();
      }
      // CASE B: TIMER FINISHED
      else if (current_time - state_start_time >= STUDY_MILLIS) {
        Serial.println("Study Time Finished.");
        
        // Add full duration
        total_study_time_ms += STUDY_MILLIS;
        
        // Print Total
        printTotalTime();

        buzz(); 
        digitalWrite(RED_PIN, LOW);
        digitalWrite(GREEN_PIN, HIGH);
        // Start Break
        state = BREAK_STATE;
        state_start_time = millis();
        sendTelemetry();
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
        Serial.println("Break Time Finished.");
        buzz(); 
        digitalWrite(RED_PIN, HIGH);
        digitalWrite(GREEN_PIN, LOW);
        state = STUDY_STATE;
        state_start_time = millis();
        sendTelemetry();
      }
      break;
  }
}