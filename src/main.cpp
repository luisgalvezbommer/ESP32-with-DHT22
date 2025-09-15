#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_wifi.h"
#include "esp_task_wdt.h"

// DHT22 Setup
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// LED Setup
#define LED_PIN 2

// WiFi Credentials
const char* ssid = "SSID";
const char* password = "PASSWORD";

// MQTT Setup
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_DHT22_luisgalvezbommer";

WiFiClient espClient;
PubSubClient client(espClient);

// Watchdog-freundliche LED-Funktion
void blinkLED(int delayMs = 250) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    esp_task_wdt_reset(); // Watchdog füttern während Blinken
    digitalWrite(LED_PIN, LOW);
}

void errorBlink() {
    for(int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        esp_task_wdt_reset(); // Wichtig!
        digitalWrite(LED_PIN, LOW);
        delay(100);
        esp_task_wdt_reset(); // Wichtig!
    }
}

void setup_wifi() {
    Serial.println("=== WiFi Setup gestartet ===");
    
    // Watchdog während WiFi-Setup verlängern
    esp_task_wdt_reset();
    
    WiFi.disconnect(true);
    delay(1000);
    esp_task_wdt_reset();
    
    WiFi.mode(WIFI_OFF);
    delay(1000);
    esp_task_wdt_reset();
    
    esp_wifi_restore();
    delay(500);
    esp_task_wdt_reset();
    
    WiFi.mode(WIFI_STA);
    delay(1000);
    esp_task_wdt_reset();

    Serial.print("Verbinde mit WiFi: ");
    Serial.println(ssid);
    
    // WiFi-Scan mit Watchdog-Reset
    Serial.println("Scanne verfügbare WiFi-Netzwerke...");
    esp_task_wdt_reset(); // Vor dem Scan!
    
    int n = WiFi.scanNetworks();
    esp_task_wdt_reset(); // Nach dem Scan!
    
    Serial.print("Gefundene Netzwerke: ");
    Serial.println(n);
    
    // Schnellere Anzeige ohne alle Details
    for (int i = 0; i < min(n, 5); ++i) { // Nur erste 5 zeigen
        Serial.print(WiFi.SSID(i));
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.println(" dBm)");
        esp_task_wdt_reset();
    }

    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { // Reduziert von 30
        delay(1000);
        esp_task_wdt_reset(); // Jeden Versuch!
        
        Serial.print(".");
        digitalWrite(LED_PIN, attempts % 2);
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("✅ WiFi verbunden!");

        WiFi.setSleep(false);
        Serial.print("IP: ");
        Serial.print(WiFi.localIP());
        Serial.print(", RSSI: ");
        Serial.println(WiFi.RSSI());
        
        // Erfolgs-LED
        for(int i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            esp_task_wdt_reset();
            digitalWrite(LED_PIN, LOW);
            delay(200);
            esp_task_wdt_reset();
        }
    } else {
        Serial.println("❌ WiFi fehlgeschlagen!");
        digitalWrite(LED_PIN, HIGH);
    }
}

void reconnect() {
    int attempts = 0;
    while (!client.connected() && attempts < 3) { // Reduziert von 5
        esp_task_wdt_reset();
        
        Serial.print("MQTT Versuch ");
        Serial.print(attempts + 1);
        Serial.print("/3... ");
        
        if (client.connect(mqtt_client_id)) {
            Serial.println("✅ OK!");
            
            // Status "online" senden bei Verbindung
            client.publish("luis/esp32/dht22/status", "online");
            esp_task_wdt_reset();
            
            // WiFi SSID senden (einmalig bei Verbindung)
            client.publish("luis/esp32/dht22/wifi/ssid", WiFi.SSID().c_str());
            esp_task_wdt_reset();
            
            blinkLED(100);
        } else {
            Serial.print("❌ Fehler ");
            Serial.println(client.state());
            attempts++;
            delay(1000); // Reduziert von 2000
            esp_task_wdt_reset();
        }
    }
    
    if (!client.connected()) {
        Serial.println("MQTT tot - NEUSTART!");
        ESP.restart();
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Watchdog Timer - LÄNGER für WiFi-Scans
    esp_task_wdt_init(60, true);  // 60 Sekunden statt 30!
    esp_task_wdt_add(NULL);
    
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    esp_task_wdt_reset();
    
    Serial.println("🚀 ESP32 gestartet!");
    
    dht.begin();
    setup_wifi();
    
    if (WiFi.status() == WL_CONNECTED) {
        client.setServer(mqtt_server, mqtt_port);
        Serial.println("✅ Setup OK!");
    } else {
        Serial.println("❌ Setup Fehler!");
        ESP.restart();
    }
}

void loop() {
    // Watchdog GANZ am Anfang füttern
    esp_task_wdt_reset();
    
    Serial.println("=== Loop Start ===");
    
    // WiFi prüfen
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("❌ WiFi weg - NEUSTART!");
        digitalWrite(LED_PIN, HIGH);
        delay(2000);
        ESP.restart();
        return;
    }

    Serial.print("📡 WiFi OK (RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(")");
    
    // Keep-Alive
    WiFi.RSSI();
    esp_task_wdt_reset();
    
    // MQTT prüfen
    if (!client.connected()) {
        Serial.println("🔗 MQTT reconnect...");
        reconnect();
    }
    client.loop();
    esp_task_wdt_reset();

    // DHT22 lesen
    Serial.println("📊 DHT22 lesen...");
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    esp_task_wdt_reset();

    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("❌ DHT Fehler!");
        errorBlink();
        
        // Sensor Error Status senden
        if (client.connected()) {
            client.publish("luis/esp32/dht22/status", "sensor_error");
            esp_task_wdt_reset();
        }
        
        delay(2000);
        return;
    }

    Serial.print("🌡️  ");
    Serial.print(temperature, 1);
    Serial.print("°C, 💧 ");
    Serial.print(humidity, 1);
    Serial.print("% -> ");

    // MQTT senden - ALLE Topics gleichzeitig
    if (client.connected()) {
        Serial.println("📤 MQTT senden...");
        
        bool all_ok = true;
        
        // 1. Temperatur
        bool temp_ok = client.publish("luis/esp32/dht22/temperature", String(temperature, 1).c_str());
        esp_task_wdt_reset();
        all_ok &= temp_ok;
        
        // 2. Luftfeuchtigkeit  
        bool hum_ok = client.publish("luis/esp32/dht22/humidity", String(humidity, 1).c_str());
        esp_task_wdt_reset();
        all_ok &= hum_ok;
        
        // 3. WiFi RSSI
        bool rssi_ok = client.publish("luis/esp32/dht22/wifi/rssi", String(WiFi.RSSI()).c_str());
        esp_task_wdt_reset();
        all_ok &= rssi_ok;
        
        // 4. System Uptime (in Sekunden)
        unsigned long uptime_seconds = millis() / 1000;
        bool uptime_ok = client.publish("luis/esp32/dht22/system/uptime", String(uptime_seconds).c_str());
        esp_task_wdt_reset();
        all_ok &= uptime_ok;
        
        // 5. Freier Heap-Speicher
        bool heap_ok = client.publish("luis/esp32/dht22/system/heap", String(ESP.getFreeHeap()).c_str());
        esp_task_wdt_reset();
        all_ok &= heap_ok;
        
        // 6. Status OK
        bool status_ok = client.publish("luis/esp32/dht22/status", "ok");
        esp_task_wdt_reset();
        all_ok &= status_ok;
        
        // Zusätzliche Debug-Ausgaben
        Serial.print("📊 Uptime: ");
        Serial.print(uptime_seconds);
        Serial.print("s, Heap: ");
        Serial.print(ESP.getFreeHeap());
        Serial.print(" bytes, RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        
        if (all_ok) {
            Serial.println("✅ Alle MQTT Topics gesendet!");
            blinkLED(100); // Kurzes Blinken
        } else {
            Serial.println("❌ Einige MQTT Topics fehlgeschlagen!");
            errorBlink();
        }
    } else {
        Serial.println("❌ MQTT nicht verbunden!");
    }

    Serial.println("⏱️  Warte 5 Sekunden...");
    
    // Aufgeteilte Wartezeit mit Watchdog-Reset
    for(int i = 0; i < 5; i++) {
        delay(1000);
        esp_task_wdt_reset();
    }
    
    Serial.println("=== Loop Ende ===\n");
}