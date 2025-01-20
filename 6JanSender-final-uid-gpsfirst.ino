//LORA SENDER ( uji coba user id dengan kode gps first then lora, 06 januari 2024 )
#include "MAX30105.h"
#include "heartRate.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

#define REPORTING_PERIOD_MS 1000
#define ONE_WIRE_BUS 4
#define FINGER_ON 7000
#define MINIMUM_SPO2 60.0
#define ss 5
#define rst 14
#define dio0 26

MAX30105 particleSensor;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2 pada ESP32 untuk GPS NEO6M

// Heart rate calculation variables
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// Blood oxygen calculation variables
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

// DS18B20 temperature sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);
float suhuDS18B20;

// Voltage and battery capacity variables
float Volt1;
float Volt;
float capacity;

// Voltage limits
const float V_min = 6.4;  // Minimum voltage (battery almost empty)
const float V_max = 8.4;  // Maximum voltage (battery full)

// Timing variables
unsigned long lastVitalReport = 0;
unsigned long lastTempRequest = 0;
unsigned long lastTempRead = 0;
unsigned long lastGPSReport = 0;
unsigned long lastBatteryCheck = 0;  // Time tracking for battery reading
bool tempRequestSent = false;

//VARIABLE USER ID
const char* userID = "User2";

void setup() {
    Serial.begin(115200);
    Serial.println("System Start");

    // Initialize MAX30102
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found");
        while (1);
    }

    // Configure MAX30102
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

    // Initialize DS18B20
    sensor.begin();
    sensor.setResolution(10);

    // Initialize GPS NEO6M
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX pada pin 16, TX pada pin 17

    // Initialize LoRa
    LoRa.setPins(ss, rst, dio0);
    while (!LoRa.begin(865.0625E6)) {
        Serial.println("Connecting LoRa...");
        delay(500);
    }
    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa Initialized Successfully");
}

void sendLoRaData(float bpm, double spo2, float suhu, double lat, double lng, float batteryCapacity) {
    LoRa.beginPacket();
    LoRa.print("ID: ");
    LoRa.print(userID);
    LoRa.print("BPM: ");
    LoRa.print(bpm);
    LoRa.print(", SpO2: ");
    LoRa.print(spo2);
    LoRa.print(", Suhu: ");
    LoRa.print(suhu);
    LoRa.print("C, Lat: ");
    LoRa.print(lat, 6);
    LoRa.print(", Lng: ");
    LoRa.print(lng, 6);
    LoRa.print(", Battery: ");
    LoRa.print(batteryCapacity);
    LoRa.print("%");
    LoRa.endPacket();
}

void checkVitalSigns() {
    long irValue = particleSensor.getIR();

    if (irValue > FINGER_ON) {
        if (checkForBeat(irValue)) {
            long delta = millis() - lastBeat;
            lastBeat = millis();
            beatsPerMinute = 60 / (delta / 1000.0);

            if (beatsPerMinute < 255 && beatsPerMinute > 20) {
                rates[rateSpot++] = (byte)beatsPerMinute;
                rateSpot %= RATE_SIZE;
                beatAvg = 0;
                for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
                beatAvg /= RATE_SIZE;
            }
        }

        if (particleSensor.available()) {
            i++;
            uint32_t ir = particleSensor.getFIFOIR();
            uint32_t red = particleSensor.getFIFORed();

            double fir = (double)ir;
            double fred = (double)red;

            aveir = aveir * frate + fir * (1.0 - frate);
            avered = avered * frate + fred * (1.0 - frate);
            sumirrms += (fir - aveir) * (fir - aveir);
            sumredrms += (fred - avered) * (fred - avered);

            if ((i % Num) == 0) {
                double R = (sqrt(sumirrms) / aveir) / (sqrt(sumredrms) / avered);
                SpO2 = 104.0 - 17.0 * R;
                ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;

                if (ESpO2 <= MINIMUM_SPO2) ESpO2 = MINIMUM_SPO2;
                if (ESpO2 > 100) ESpO2 = 99.9;

                sumredrms = 0.0;
                sumirrms = 0.0;
                SpO2 = 0; // Reset SpO2 for the next calculation
                i = 0;
            }
            particleSensor.nextSample();
        }

        if (millis() - lastVitalReport >= REPORTING_PERIOD_MS) {
            Serial.print("Bpm: " + String(beatAvg));
            if (beatAvg > 30) {
                Serial.println(", SPO2: " + String(ESpO2));
            } else {
                Serial.println(", SPO2: ----");
                ESpO2 = 0; // Set ESpO2 to 0 if no valid reading
            }
            lastVitalReport = millis();
        }
    } else {
        if (millis() - lastVitalReport >= REPORTING_PERIOD_MS) {
            resetVitalSigns();
            Serial.println("Finger Please");
            ESpO2 = 0; // Set ESpO2 to 0 if no finger detected
            lastVitalReport = millis();
        }
    }
}

void resetVitalSigns() {
    for (byte rx = 0; rx < RATE_SIZE; rx++) rates[rx] = 0;
    beatAvg = 0;
    rateSpot = 0;
    lastBeat = 0;
    avered = 0;
    aveir = 0;
    sumirrms = 0;
    sumredrms = 0;
    SpO2 = 0;
    ESpO2 = 90.0; // Default value when no readings
}

void handleTemperature() {
    unsigned long currentMillis = millis();

    if (!tempRequestSent && (currentMillis - lastTempRequest >= 1000)) {
        sensor.requestTemperatures();
        tempRequestSent = true;
        lastTempRequest = currentMillis;
        lastTempRead = currentMillis;
    }

    if (tempRequestSent && (currentMillis - lastTempRead >= 750)) {  // 750ms waktu konversi
        suhuDS18B20 = sensor.getTempCByIndex(0);
        Serial.print("Suhu: ");
        Serial.println(suhuDS18B20, 2);
        tempRequestSent = false;
    }
}

void handleGPS() {
    unsigned long currentMillis = millis();

    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid() && (currentMillis - lastGPSReport >= 1000)) {
                Serial.print("Lat: ");
                Serial.print(gps.location.lat(), 6);
                Serial.print(", Lng: ");
                Serial.println(gps.location.lng(), 6);
                
                // Send LoRa data including battery capacity
                sendLoRaData(beatAvg, ESpO2, suhuDS18B20, gps.location.lat(), gps.location.lng(), capacity);
                lastGPSReport = currentMillis;
            }
        }
    }

    if (currentMillis > 5000 && gps.charsProcessed() < 10) {
        Serial.println(F("No GPS data received: check wiring."));
        delay(5000);
    }
}

void readBatteryCapacity() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBatteryCheck >= 5000) { // Baca kapasitas baterai setiap 5 detik
        Volt1 = analogRead(35); // Read from pin 35
        Volt = (Volt1 * 0.00088) * 5; // Calculation for voltage
        
        // Calculate capacity in percentage
        if (Volt >= V_max) {
            capacity = 100.0;  // Full capacity
        } else if (Volt <= V_min) {
            capacity = 0.0;    // Empty capacity
        } else {
            // Percentage capacity from range
            capacity = ((Volt - V_min) / (V_max - V_min)) * 100.0;
        }
        Serial.print("Kapasitas Baterai: ");
        Serial.print(capacity);
        Serial.println(" %");

        lastBatteryCheck = currentMillis; // Update last battery check time
    }
}

void loop() {
    checkVitalSigns();
    handleTemperature();
    handleGPS();
    readBatteryCapacity(); // Membaca kapasitas baterai setiap 5 detik
}