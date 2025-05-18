//LORA SENDER ( uji coba user id dengan kode gps first then lora, improved stability )
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
#define FINGER_ON 7000         // Ambang deteksi jari
#define MINIMUM_SPO2 60.0
#define ss 5
#define rst 14
#define dio0 26

// Tambahkan definisi untuk buffer averaging yang lebih besar
#define MAX_BPM_BUFFER 10          // Ukuran buffer untuk rata-rata BPM
#define MAX_SPO2_BUFFER 10         // Ukuran buffer untuk rata-rata SpO2
#define VALID_READING_THRESHOLD 3  // Jumlah minimum pembacaan valid sebelum menampilkan hasil

MAX30105 particleSensor;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2 pada ESP32 untuk GPS NEO6M

// Heart rate calculation variables - dengan buffer yang lebih besar
const byte RATE_SIZE = 8;  // Perbesar rate size dari 4 ke 8
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
int lastBeatAvg = 0;  // Menyimpan nilai BPM terakhir yang valid

// Filter variabel tambahan untuk BPM
float bpmBuffer[MAX_BPM_BUFFER];
int bpmBufferIndex = 0;
int validBpmReadings = 0;
float filteredBPM = 0;
bool bpmIsValid = false;

// Blood oxygen calculation variables
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
double SpO2 = 0;
double ESpO2 = 90.0;    // Nilai awal yang lebih realistis
double FSpO2 = 0.7;
double frate = 0.95;    // IR filtering rate - meningkatkan untuk filtering yang lebih agresif
int i = 0;
int Num = 50;           // Perbesar interval sampling untuk SpO2 dari 30 ke 50

// Filter variabel tambahan untuk SpO2
float spo2Buffer[MAX_SPO2_BUFFER];
int spo2BufferIndex = 0;
int validSpo2Readings = 0;
float filteredSpO2 = 0;
bool spo2IsValid = false;
float lastValidSpO2 = 95.0;  // Nilai SpO2 terakhir yang valid

// DS18B20 temperature sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);
float suhuDS18B20; // Deklarasi variabel

// Fungsi kalibrasi suhu - pindahkan ke atas sebelum digunakan
float calibrateTemperature(float rawTemp) {
    // Faktor kalibrasi berdasarkan selisih
    const float CALIBRATION_OFFSET = 0.75; // Selisih yang perlu ditambahkan
    const float CALIBRATION_FACTOR = 1.0;  // Faktor pengali jika diperlukan
    
    // Aplikasikan kalibrasi
    float calibratedTemp = (rawTemp * CALIBRATION_FACTOR) + CALIBRATION_OFFSET;
    
    // Pembulatan ke 0.1°C
    // Cara kerja: kalikan dengan 10, bulatkan ke bilangan bulat, lalu bagi lagi dengan 10
    calibratedTemp = round(calibratedTemp * 10.0) / 10.0;
    
    return calibratedTemp;
}

// Voltage and battery capacity variables
float Volt1;
float Volt;
float capacity;

// Voltage limits
const float V_min = 3.0;  // Minimum voltage (battery almost empty)
const float V_max = 4.2;  // Maximum voltage (battery full)

// Timing variables
unsigned long lastVitalReport = 0;
unsigned long lastTempRequest = 0;
unsigned long lastTempRead = 0;
unsigned long lastGPSReport = 0;
unsigned long lastBatteryCheck = 0;  // Time tracking for battery reading
bool tempRequestSent = false;

// Variabel untuk cek stabilitas
unsigned long fingerPresentTime = 0;
bool fingerStable = false;
const unsigned long STABILITY_DELAY = 2000; // Tunggu 2 detik sebelum mulai pembacaan

// Variabel untuk deteksi anomali
const float MAX_BPM_CHANGE_RATE = 20.0;     // Maksimum perubahan BPM yang diperbolehkan antara pembacaan
const float MAX_SPO2_CHANGE_RATE = 5.0;     // Maksimum perubahan SpO2 yang diperbolehkan antara pembacaan

//VARIABLE USER ID
const char* userID = "User3";

void setup() {
    Serial.begin(115200);
    Serial.println("System Start");

    // Initialize MAX30102
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found");
        while (1);
    }

    // Konfigurasi MAX30102 yang dioptimalkan
    byte ledBrightness = 0x1F;          // Kurangi brightness menjadi 60 (dari 0x7F) untuk mengurangi noise
    byte sampleAverage = 8;             // Tingkatkan sample averaging dari 4 ke 8
    byte ledMode = 2;                   // Mode 2 = Red + IR
    int sampleRate = 400;               // Turunkan dari 800 ke 400 Hz untuk noise yang lebih rendah
    int pulseWidth = 411;               // Tingkatkan pulse width dari 215 ke 411 untuk sinyal yang lebih kuat
    int adcRange = 4096;                // Turunkan dari 16384 ke 4096 untuk noise yang lebih rendah
    
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    particleSensor.enableDIETEMPRDY();
    
    // Sesuaikan amplitudo pulsa untuk pembacaan yang lebih kuat
    particleSensor.setPulseAmplitudeRed(0x0A);     // 0x0A = 10mA
    particleSensor.setPulseAmplitudeGreen(0);
    particleSensor.setPulseAmplitudeIR(0x0A);      // 0x0A = 10mA
    
    // Inisialisasi buffer BPM dan SpO2
    for (int i = 0; i < MAX_BPM_BUFFER; i++) bpmBuffer[i] = 0;
    for (int i = 0; i < MAX_SPO2_BUFFER; i++) spo2Buffer[i] = 0;

    // Initialize DS18B20
    sensor.begin();
    sensor.setResolution(12);

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
    LoRa.print(" BPM: ");
    LoRa.print(bpm, 1);
    LoRa.print(", SpO2: ");
    LoRa.print(spo2, 1);
    LoRa.print(", Suhu: ");
    LoRa.print(suhu, 1);
    LoRa.print("C, Lat: ");
    LoRa.print(lat, 6);
    LoRa.print(", Lng: ");
    LoRa.print(lng, 6);
    LoRa.print(", Battery: ");
    LoRa.print(batteryCapacity, 1);
    LoRa.print("%");
    LoRa.endPacket();
}

// Fungsi untuk filter BPM menggunakan rata-rata
float filterBPM(float newBPM) {
    // Cek apakah pembacaan baru berada dalam batas yang masuk akal
    if (newBPM < 40 || newBPM > 220) {
        return filteredBPM; // Pertahankan nilai terakhir jika di luar batas normal
    }
    
    // Cek apakah pembacaan baru terlalu jauh berbeda dari pembacaan sebelumnya
    if (validBpmReadings > 0 && abs(newBPM - filteredBPM) > MAX_BPM_CHANGE_RATE) {
        return filteredBPM; // Pertahankan nilai terakhir jika perubahan terlalu besar
    }
    
    // Tambahkan nilai baru ke buffer
    bpmBuffer[bpmBufferIndex] = newBPM;
    bpmBufferIndex = (bpmBufferIndex + 1) % MAX_BPM_BUFFER;
    
    // Tambah penghitung pembacaan valid
    if (validBpmReadings < MAX_BPM_BUFFER) {
        validBpmReadings++;
    }
    
    // Hitung rata-rata dari seluruh buffer
    float sum = 0;
    for (int i = 0; i < validBpmReadings; i++) {
        sum += bpmBuffer[i];
    }
    
    // Perbarui nilai BPM terfilter
    filteredBPM = sum / validBpmReadings;
    return filteredBPM;
}

// Fungsi untuk filter SpO2 menggunakan rata-rata dan deteksi anomali
// Fungsi filter SpO2 yang direvisi
float filterSpO2(float newSpO2) {
    // Pendekatan kalibrasi proporsional alih-alih offset tetap
    // Berdasarkan data pengujian: 86->98, 87->99, 80->97
    
    // Faktor kalibrasi menggunakan pendekatan proporsional
    float calibrationFactor = 1.12;  // Faktor pengali (~12% lebih tinggi)
    float calibratedSpO2 = newSpO2 * calibrationFactor;
    
    // Batasi nilai maksimum pada 100%
    if (calibratedSpO2 > 100.0) {
        calibratedSpO2 = 100.0;
    }
    
    // Cek apakah pembacaan baru berada dalam batas yang masuk akal
    if (calibratedSpO2 < MINIMUM_SPO2 || calibratedSpO2 > 100) {
        return filteredSpO2; // Pertahankan nilai terakhir jika di luar batas normal
    }
    
    // Cek apakah pembacaan baru terlalu jauh berbeda dari pembacaan sebelumnya
    if (validSpo2Readings > 0 && abs(calibratedSpO2 - filteredSpO2) > MAX_SPO2_CHANGE_RATE) {
        return filteredSpO2; // Pertahankan nilai terakhir jika perubahan terlalu besar
    }
    
    // Tambahkan nilai baru ke buffer
    spo2Buffer[spo2BufferIndex] = calibratedSpO2;
    spo2BufferIndex = (spo2BufferIndex + 1) % MAX_SPO2_BUFFER;
    
    // Tambah penghitung pembacaan valid
    if (validSpo2Readings < MAX_SPO2_BUFFER) {
        validSpo2Readings++;
    }
    
    // Hitung rata-rata dari seluruh buffer
    float sum = 0;
    for (int i = 0; i < validSpo2Readings; i++) {
        sum += spo2Buffer[i];
    }
    
    // Perbarui nilai SpO2 terfilter
    filteredSpO2 = sum / validSpo2Readings;
    lastValidSpO2 = filteredSpO2; // Simpan nilai terakhir yang valid
    return filteredSpO2;
}

void checkVitalSigns() {
    long irValue = particleSensor.getIR();

    // Cek keberadaan jari
    if (irValue > FINGER_ON) {
        // Cek stabilitas, tunggu hingga jari stabil
        if (!fingerStable) {
            if (fingerPresentTime == 0) {
                fingerPresentTime = millis();
            } else if (millis() - fingerPresentTime > STABILITY_DELAY) {
                fingerStable = true;
                Serial.println("Finger detected and stable. Starting measurements...");
            }
        }
        
        // Hanya proses data jika jari sudah stabil
        if (fingerStable) {
            // Cek heart beat
            if (checkForBeat(irValue)) {
                long delta = millis() - lastBeat;
                lastBeat = millis();
                
                // Aplikasikan filter untuk menghilangkan spike
                beatsPerMinute = 60 / (delta / 1000.0);
                
                if (beatsPerMinute < 220 && beatsPerMinute > 40) {
                    rates[rateSpot++] = (byte)beatsPerMinute;
                    rateSpot %= RATE_SIZE;
                    
                    // Hitung rata-rata BPM
                    beatAvg = 0;
                    byte validValues = 0;
                    for (byte x = 0; x < RATE_SIZE; x++) {
                        if (rates[x] > 0) {
                            beatAvg += rates[x];
                            validValues++;
                        }
                    }
                    
                    if (validValues > 0) {
                        beatAvg /= validValues;
                        
                        // Aplikasikan filter tambahan untuk BPM
                        beatAvg = filterBPM(beatAvg);
                        bpmIsValid = (validBpmReadings >= VALID_READING_THRESHOLD);
                        
                        // Jika perubahan drastis, pakai nilai terakhir yang valid
                        if (abs(beatAvg - lastBeatAvg) > MAX_BPM_CHANGE_RATE && lastBeatAvg > 0) {
                            beatAvg = lastBeatAvg;
                        } else {
                            lastBeatAvg = beatAvg;
                        }
                    }
                }
            }

            // Proses data untuk SpO2
            if (particleSensor.available()) {
                i++;
                uint32_t ir = particleSensor.getFIFOIR();
                uint32_t red = particleSensor.getFIFORed();

                double fir = (double)ir;
                double fred = (double)red;

                // Aplikasikan averaging filter dengan faktor alpha yang lebih kecil untuk stabilitas
                aveir = aveir * frate + fir * (1.0 - frate);
                avered = avered * frate + fred * (1.0 - frate);
                
                // Kumpulkan data untuk perhitungan RMS
                sumirrms += (fir - aveir) * (fir - aveir);
                sumredrms += (fred - avered) * (fred - avered);

                if ((i % Num) == 0) {
                // Hindari pembagian dengan nol
                if (avered > 0 && aveir > 0 && sumredrms > 0 && sumirrms > 0) {
                    double R = (sqrt(sumirrms) / aveir) / (sqrt(sumredrms) / avered);
                    
                    // Kembalikan ke rumus awal dengan sedikit modifikasi
                    SpO2 = 104.0 - 18.0 * R; // Sedikit modifikasi dari rumus asli untuk akurasi lebih baik
                    
                    // Filter nilai SpO2
                    if (SpO2 >= MINIMUM_SPO2 && SpO2 <= 100) {
                        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
                        
                        // Terapkan filter tambahan pada SpO2
                        ESpO2 = filterSpO2(ESpO2);
                        spo2IsValid = (validSpo2Readings >= VALID_READING_THRESHOLD);
                    }
                }
                
                // Reset variabel untuk perhitungan berikutnya
                sumredrms = 0.0;
                sumirrms = 0.0;
                i = 0;
                }
                particleSensor.nextSample(); // Lanjut ke sampel berikutnya
            }

            // Laporan data tanda vital
            if (millis() - lastVitalReport >= REPORTING_PERIOD_MS) {
                // Laporan data hanya jika sudah mencapai jumlah pembacaan valid
                Serial.print("BPM: ");
                if (bpmIsValid) {
                    Serial.print(beatAvg);
                } else {
                    Serial.print("----");
                }
                
                Serial.print(", SpO2: ");
                if (spo2IsValid) {
                    Serial.print(ESpO2, 1);
                    Serial.print("%");
                } else {
                    Serial.print("----");
                }
                
                Serial.print(" (IR: ");
                Serial.print(irValue);
                Serial.println(")");
                
                lastVitalReport = millis();
            }
        }
    } else {
        // Reset saat tidak ada jari
        if (millis() - lastVitalReport >= REPORTING_PERIOD_MS) {
            resetVitalSigns();
            Serial.println("Place your finger on the sensor");
            lastVitalReport = millis();
        }
    }
}

void resetVitalSigns() {
    // Reset semua variabel ke nilai awal
    for (byte rx = 0; rx < RATE_SIZE; rx++) rates[rx] = 0;
    beatAvg = 0;
    rateSpot = 0;
    lastBeat = 0;
    
    // Reset variabel SpO2
    avered = 0;
    aveir = 0;
    sumirrms = 0;
    sumredrms = 0;
    SpO2 = 0;
    ESpO2 = 90.0; // Default value yang realistis saat tidak ada pembacaan
    
    // Reset variabel stabilitas
    fingerStable = false;
    fingerPresentTime = 0;
    
    // Reset filter buffers
    for (int i = 0; i < MAX_BPM_BUFFER; i++) bpmBuffer[i] = 0;
    for (int i = 0; i < MAX_SPO2_BUFFER; i++) spo2Buffer[i] = 0;
    bpmBufferIndex = 0;
    spo2BufferIndex = 0;
    validBpmReadings = 0;
    validSpo2Readings = 0;
    filteredBPM = 0;
    filteredSpO2 = 0;
    bpmIsValid = false;
    spo2IsValid = false;
}

void handleTemperature() {
    unsigned long currentMillis = millis();

    if (!tempRequestSent && (currentMillis - lastTempRequest >= 1000)) {
        sensor.requestTemperatures();
        tempRequestSent = true;
        lastTempRequest = currentMillis;
        lastTempRead = currentMillis;
    }

    if (tempRequestSent && (currentMillis - lastTempRead >= 800)) {  // Tambahkan waktu konversi menjadi 800ms untuk 12-bit
        float rawTemp = sensor.getTempCByIndex(0);
        // Terapkan kalibrasi hanya jika pembacaan valid
        if (rawTemp != -127.00) {
            suhuDS18B20 = calibrateTemperature(rawTemp);
        }
        Serial.print("Suhu: ");
        Serial.println(suhuDS18B20, 1); // Tampilkan 1 digit desimal untuk presisi 0.1°C
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
                
                // Kirim data LoRa hanya jika BPM dan SpO2 valid
                float bpmToSend = bpmIsValid ? beatAvg : 0;
                float spo2ToSend = spo2IsValid ? ESpO2 : 0;
                
                sendLoRaData(bpmToSend, spo2ToSend, suhuDS18B20, 
                             gps.location.lat(), gps.location.lng(), capacity);
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
    if (currentMillis - lastBatteryCheck >= 5000) {
        // Raw reading
        Volt1 = analogRead(35); // Read from pin 35
        
        // Calculate uncalibrated voltage (raw calculation)
        float rawVolt = Volt1 * 0.00088 * 5;
        
        // Apply calibration factor - using direct ratio between multimeter and sensor
        // 3.80V (multimeter) / 3.71V (sensor) = 1.0243
        float calibrationFactor = 0.6868; // 0.662 * 1.0243 ≈ 0.678
        
        // Calculate calibrated voltage
        Volt = rawVolt * calibrationFactor;
        
        // Calculate capacity in percentage
        if (Volt >= V_max) {
            capacity = 100.0;  // Full capacity
        } else if (Volt <= V_min) {
            capacity = 0.0;    // Empty capacity
        } else {
            // Percentage capacity from range
            capacity = ((Volt - V_min) / (V_max - V_min)) * 100.0;
        }
        
        Serial.print("Tegangan (Raw): ");
        Serial.print(rawVolt, 3);
        Serial.println(" V");
        
        Serial.print("Tegangan (Calibrated): ");
        Serial.print(Volt, 2);
        Serial.println(" V");
        
        Serial.print("Kapasitas Baterai: ");
        Serial.print(capacity);
        Serial.println(" %");

        lastBatteryCheck = currentMillis;
    }
}

void loop() {
    checkVitalSigns();
    handleTemperature();
    handleGPS();
    readBatteryCapacity(); // Membaca kapasitas baterai setiap 5 detik
}