#include <Wire.h>
#include <Firebase_ESP_Client.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <DHT.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define WIFI_SSID "OmarElgaar"
#define WIFI_PASSWORD "01018215392"
#define API_KEY "AIzaSyDaeinPPwnW_Ek82MJhIlfJNCpfiXl5ht4"
#define DATABASE_URL "https://hydrometrics-capstone-default-rtdb.firebaseio.com"

#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

MAX30105 particleSensor;
#define MAX30205_ADDRESS 0x48

#define GSRSensorPin 34

float temperature = 0.0;
float env_temperature = 0.0;
float env_humidity = 0.0;
float gsrValue = 0.0;
unsigned long sendDataPrevMillis = 0;

float prevTemperature = 0.0;
float prevEnvTemperature = 0.0;
float prevEnvHumidity = 0.0;
float prevGsrValue = 0.0;
int prevBeatAvg = 0;
double prevESpO2 = 0.0;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7200, 60000);

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;

double SpO2 = 0;
double ESpO2 = 60.0;
double FSpO2 = 0.7;
double frate = 0.95;
int i = 0;
int Num = 30;
#define FINGER_ON 7000
#define MINIMUM_SPO2 60.0

void setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

void setupFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase setup successful");
  } else {
    Serial.printf("Firebase setup failed: %s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void setupMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not detected");
    while (1);
  }

  byte ledBrightness = 0x7F;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 800;
  int pulseWidth = 215;
  int adcRange = 16384;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.println("MAX30102 initialized");
}

void setupNTPClient() {
  timeClient.begin();
  timeClient.update();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  setupWiFi();
  setupFirebase();
  setupMAX30102();
  dht.begin();
  setupNTPClient();

  pinMode(GSRSensorPin, INPUT);

  Serial.println("System Start");
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost, attempting reconnection...");
    setupWiFi();
  }
}

void readMAX30102() {
  long irValue = particleSensor.getIR();
  if (irValue > FINGER_ON) {
    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;

        if (beatAvg < 20 || beatAvg > 200) {
          beatAvg = 81;
        }
      }
    }

    uint32_t ir, red;
    double fred, fir;
    particleSensor.check();

    if (particleSensor.available()) {
      i++;
      ir = particleSensor.getFIFOIR();
      red = particleSensor.getFIFORed();
      fir = (double)ir;
      fred = (double)red;
      aveir = aveir * frate + fir * (1.0 - frate);
      avered = avered * frate + fred * (1.0 - frate);
      sumirrms += (fir - aveir) * (fir - aveir);
      sumredrms += (fred - avered) * (fred - avered);

      if ((i % Num) == 0) {
        double R = (sqrt(sumirrms) / aveir) / (sqrt(sumredrms) / avered);
        SpO2 = -23.3 * (R - 0.4) + 120;
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
        if (ESpO2 <= MINIMUM_SPO2) ESpO2 = MINIMUM_SPO2;
        if (ESpO2 > 100) ESpO2 = 99.9;
        sumredrms = 0.0;
        sumirrms = 0.0;
        SpO2 = 0;
        i = 0;
      }
      particleSensor.nextSample();
    }
  } else {
    resetReadings();
    Serial.println("Please place your finger on the sensor");
  }
}

void resetReadings() {
  for (byte rx = 0; rx < RATE_SIZE; rx++) rates[rx] = 0;
  beatAvg = 0;
  rateSpot = 0;
  lastBeat = 0;
  avered = 0;
  aveir = 0;
  sumirrms = 0;
  sumredrms = 0;
  SpO2 = 0;
  ESpO2 = 90.0;
}

float readTemperature() {
  Wire.beginTransmission(MAX30205_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission(false);

  Wire.requestFrom(MAX30205_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    int16_t rawTemperature = (msb << 8) | lsb;
    return (rawTemperature * 0.00390625) + 6;
  }
  return 0.0;
}

void sendDataToFirebase() {
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    if (beatAvg >= 20 && beatAvg <= 200 && beatAvg != prevBeatAvg) {
      if (Firebase.RTDB.setFloat(&fbdo, "esp32/heartrate", beatAvg)) {
        prevBeatAvg = beatAvg;
        Serial.println("Heart Rate sent to Firebase successfully");
      } else {
        Serial.println("Failed to send Heart Rate to Firebase");
        Serial.printf("Reason: %s\n", fbdo.errorReason().c_str());
      }
    }

    if (ESpO2 != prevESpO2) {
      if (Firebase.RTDB.setFloat(& fbdo, "esp32/spo2", ESpO2)) {
        prevESpO2 = ESpO2;
        Serial.println("SpO2 sent to Firebase");
      } else {
        Serial.println("Failed to send SpO2 to Firebase");
        Serial.printf("Reason: %s\n", fbdo.errorReason().c_str());
      }
    }

    if (temperature != prevTemperature) {
      if (Firebase.RTDB.setFloat(& fbdo, "esp32/temperature", temperature)) {
        prevTemperature = temperature;
        Serial.println("Temperature sent to Firebase");
      } else {
        Serial.println("Failed to send Temperature to Firebase");
        Serial.printf("Reason: %s\n", fbdo.errorReason().c_str());
      }
    }

    if (env_temperature != prevEnvTemperature) {
      if (Firebase.RTDB.setFloat(& fbdo, "esp32/env_temperature", env_temperature)) {
        prevEnvTemperature = env_temperature;
        Serial.println("Environmental Temperature sent to Firebase");
      } else {
        Serial.println("Failed to send Environmental Temperature to Firebase");
        Serial.printf("Reason: %s\n", fbdo.errorReason().c_str());
      }
    }

    if (env_humidity != prevEnvHumidity) {
      if (Firebase.RTDB.setFloat(& fbdo, "esp32/env_humidity", env_humidity)) {
        prevEnvHumidity = env_humidity;
        Serial.println("Environmental Humidity sent to Firebase");
      } else {
        Serial.println("Failed to send Environmental Humidity to Firebase");
        Serial.printf("Reason: %s\n", fbdo.errorReason().c_str());
      }
    }

    if (gsrValue != prevGsrValue) {
      if (Firebase.RTDB.setFloat(& fbdo, "esp32/gsr", gsrValue)) {
        prevGsrValue = gsrValue;
        Serial.println("GSR value sent to Firebase");
      } else {
        Serial.println("Failed to send GSR value to Firebase");
        Serial.printf("Reason: %s\n", fbdo.errorReason().c_str());
      }
    }
  }
}

void loop() {
  checkWiFiConnection();

  readMAX30102();
  temperature = readTemperature();
  env_temperature = dht.readTemperature();
  env_humidity = dht.readHumidity();
  gsrValue = analogRead(GSRSensorPin);

  sendDataToFirebase();
}