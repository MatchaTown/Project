#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Define LoRa pins
#define ss 5
#define rst 14
#define dio0 26

// WiFi & MQTT configuration
const char* ssid = "X";
const char* password = "beraktakcebok";
const char* mqtt_server = "test.mosquitto.org";

// Initialize WiFi and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Variables for sensor data
String receivedData = "";
float heartRate = 0;
float spo2 = 0;
float suhuDS18B20 = 0;
String gpsLocation = "";  // Variable for GPS data
float batteryCapacity = 0; // Variable for battery capacity

// Function to connect to WiFi
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// Function to reconnect to MQTT broker if disconnected
void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("LoRa Receiver");

    // Setup LoRa transceiver module
    LoRa.setPins(ss, rst, dio0);
    
    while (!LoRa.begin(865.0625E6)) {
        Serial.println("Connecting to LoRa...");
        delay(500);
    }

    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa Initialized");

    // Setup WiFi and MQTT
    setup_wifi();
    client.setServer(mqtt_server, 1883);
}

void loop() {
    // Ensure MQTT connection
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Check if LoRa has received any data
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        receivedData = "";

        // Read LoRa packet data
        while (LoRa.available()) {
            receivedData += (char)LoRa.read();
        }

        Serial.println("Received: " + receivedData);

        // Parse the received data
        parseData(receivedData);

        // Publish data to MQTT broker
        String heartRateStr = String(heartRate);
        String spo2Str = String(spo2);
        String suhuStr = String(suhuDS18B20);
        
        Serial.println("Heart Rate: " + heartRateStr);
        Serial.println("SpO2: " + spo2Str);
        Serial.println("Suhu: " + suhuStr);
        Serial.println("Location: " + gpsLocation);
        Serial.println("Battery Capacity: " + String(batteryCapacity));

        client.publish("sensor/heart_rateBpkHaji", heartRateStr.c_str());
        client.publish("sensor/spo2BpkHaji", spo2Str.c_str());
        client.publish("sensor/suhBpkHaji", suhuStr.c_str());
        
        // Publish GPS data to MQTT if valid
        if (gpsLocation != "") {
            client.publish("lokasi/BpkHaji", gpsLocation.c_str());
        }

        // Publish battery capacity
        String batteryCapacityStr = String(batteryCapacity);
        client.publish("kapasitas/bpkHaji", batteryCapacityStr.c_str());

        delay(500); // Delay before reading the next data
    }
}

// Function to parse received data from LoRa
void parseData(String data) {
    int hrIndex = data.indexOf("BPM: ");
    int spo2Index = data.indexOf(", SpO2: ");
    int suhuIndex = data.indexOf(", Suhu: ");
    int latIndex = data.indexOf(", Lat: ");
    int longIndex = data.indexOf(", Lng: ");
    int batteryIndex = data.indexOf(", Battery: ");

    if (hrIndex != -1 && spo2Index != -1 && suhuIndex != -1 && latIndex != -1 && longIndex != -1 && batteryIndex != -1) {
        heartRate = data.substring(hrIndex + 5, spo2Index).toFloat();
        spo2 = data.substring(spo2Index + 8, suhuIndex).toFloat();
        suhuDS18B20 = data.substring(suhuIndex + 8, latIndex).toFloat();

        // Parsing GPS data
        float latitude = data.substring(latIndex + 6, longIndex).toFloat();
        float longitude = data.substring(longIndex + 6, batteryIndex).toFloat();
        batteryCapacity = data.substring(batteryIndex + 10).toFloat(); // Parsing battery capacity
        
        // Store GPS location in format Lat,Long
        gpsLocation = String(latitude, 6) + "," + String(longitude, 6);

        // Display parsed data on Serial Monitor
        Serial.print("Heart Rate: "); Serial.println(heartRate);
        Serial.print("SpO2: "); Serial.println(spo2);
        Serial.print("Suhu: "); Serial.println(suhuDS18B20);
        Serial.print("Location: "); Serial.println(gpsLocation);
        Serial.print("Battery Capacity: "); Serial.println(batteryCapacity);
    } else {
        Serial.println("Invalid data received");
    }
}
