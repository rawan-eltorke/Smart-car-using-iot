//ESP32 DONEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE


#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <WiFiClientSecure.h>
#include <ESP32PWM.h>
#include <DHT.h>

// Replace these with your WiFi and MQTT broker details
const char* ssid = "Shahd’s iphone";
const char* password = "00000000";
const char* mqttServer = "9295ba44a3014ae68d67fc8c4d721df0.s1.eu.hivemq.cloud";
const int mqttPort = 8883;  // Make sure this is the correct port for your broker
const char* mqttUser = "Mahmoud"; // Leave empty if not used
const char* mqttPassword = "M123456m"; // Leave empty if not used

// Pins
const int servoPin = 5;          // Pin connected to the servo
const int flameSensorPin = 35;   // Analog pin connected to the flame sensor
const int irSensorPin = 15;      // Digital pin connected to the IR sensor
const int buzzerPin = 18;        // Pin connected to the buzzer
const int dhtPin = 4;            // Pin connected to the DHT sensor

const int DHT_TYPE = DHT11;      // Define DHT type (DHT11 or DHT22)
const int flameThreshold = 200;  // Default threshold for flame detection (adjust as needed)

// Variables
unsigned long previousMillis = 0;
const long interval = 1000; // Interval at which to read the sensors and publish
unsigned long startTime;

// Variables for non-blocking servo movement
unsigned long servoPreviousMillis = 0;
const long servoInterval = 500; // Half a second

WiFiClientSecure espClient;
PubSubClient client(espClient);

// Create objects for the servo, DHT sensor, and LCD
Servo myServo;
DHT dht(dhtPin, DHT_TYPE);

// Set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// Set LCD address, number of columns and rows
// If you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

void connectToMqtt()
{
    while (!client.connected())
    {
        Serial.print("Connecting to MQTT...");

        if (client.connect("ESP32Client", mqttUser, mqttPassword))
        {
            Serial.println("Connected");
            client.subscribe("sensor/reading"); // Subscribe to the IR sensor topic
            client.subscribe("flame/detected"); // Subscribe to the flame detected topic
            client.subscribe("temp/reading");
            client.subscribe("humidity/reading");
            client.subscribe("ultrasonic/distance");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup()
{
    Serial.begin(9600); // Start serial communication
    Serial2.begin(9600, SERIAL_8N1, 16, 17); // Initialize Serial2 (TX=17, RX=16 on ESP32)

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    int wifiAttempts = 0;
    const int maxWifiAttempts = 30; // Max number of attempts
    while (WiFi.status() != WL_CONNECTED && wifiAttempts < maxWifiAttempts)
    {
        delay(100);
        Serial.print(".");
        wifiAttempts++;
    }
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("Failed to connect to WiFi");
        while (true); // Stop execution if WiFi connection fails
    }
    Serial.println("WiFi connected");
    espClient.setInsecure(); // For security, use proper certificate validation in production

    // Set MQTT server
    client.setServer(mqttServer, mqttPort);

    // Connect to MQTT broker
    connectToMqtt();

    pinMode(flameSensorPin, INPUT);
    pinMode(irSensorPin, INPUT);
    pinMode(buzzerPin, OUTPUT);

    myServo.attach(servoPin);
    myServo.write(90); // Set the servo to 90 degrees

    dht.begin(); // Initialize the DHT sensor

    lcd.init();        // Initialize the LCD
    lcd.backlight();   // Turn on the LCD backlight
    lcd.clear();       // Clear any previous text
}

void loop()
{
    if (!client.connected())
    {
        connectToMqtt();
    }
    client.loop();

    myServo.write(0);    // Move to 0 degrees
    delay(500);          // Wait for half a second
    myServo.write(90);   // Move to 90 degrees
    delay(500);          // Wait for half a second

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;

        // Read flame sensor value
        int flameSensorValue = analogRead(flameSensorPin);

        // Read IR sensor value
        int irSensorValue = digitalRead(irSensorPin);

        // Check flame sensor value
        bool flameDetected = flameSensorValue < flameThreshold; // Adjust threshold as needed

        // Check IR sensor value
        bool motionDetected = irSensorValue == LOW;

        // Print sensor values for debugging
        Serial.print("Flame Sensor Value: ");
        Serial.println(flameSensorValue);
        Serial.print("IR Sensor Value: ");
        Serial.println(irSensorValue);

        // Receive distance data from Arduino
         if (Serial2.available()) {
    String receivedData = Serial2.readStringUntil('\n');
    Serial.print("Received from Arduino: ");
    Serial.println(receivedData);
  
            // Publish distance data to MQTT
            client.publish("ultrasonic/distance", receivedData.c_str());

            // Update LCD with received distance
            lcd.clear();           // Clear the previous display
            lcd.setCursor(0, 0);   // Set cursor to first row
            lcd.print("Distance: ");
            lcd.print(receivedData);
        }

        // Update the LCD and buzzer based on sensor readings
        if (flameDetected && motionDetected) {
            digitalWrite(buzzerPin, LOW); // Turn off buzzer
            delay(50);                   // Delay to prevent immediate reactivation
            lcd.clear();                  // Clear the previous display
            lcd.setCursor(0, 0);          // Set cursor to the first row
            lcd.print("Working");         // Display "Working" on the LCD
            Serial.println("Flame and motion detected! Buzzer OFF");
        } else if (flameDetected) {
            digitalWrite(buzzerPin, HIGH); // Turn on buzzer
            delay(50);                    // Delay to prevent immediate reactivation
            lcd.clear();                   // Clear the previous display
            lcd.setCursor(0, 0);           // Set cursor to the first row
            lcd.print("FIRE DETECTED!");   // Display "FIRE DETECTED!" on the LCD
            Serial.println("Flame detected! Buzzer ON");
        } else if (motionDetected) {
            digitalWrite(buzzerPin, HIGH); // Turn on buzzer
            delay(50);                    // Delay to prevent immediate reactivation
            lcd.clear();                   // Clear the previous display
            lcd.setCursor(0, 0);           // Set cursor to the first row
            lcd.print("MOTION DETECTED!"); // Display "MOTION DETECTED!" on the LCD
            Serial.println("Motion detected! Buzzer ON");
        } else {
            digitalWrite(buzzerPin, LOW);  // Turn off buzzer
            delay(50);                    // Delay to prevent immediate reactivation
            lcd.clear();                   // Clear the previous display
            lcd.setCursor(0, 0);           // Set cursor to the first row
            lcd.print("No Alert");         // Display "No Alert" on the LCD
            Serial.println("No alerts. Buzzer OFF");
        }

        // Publish sensor data to MQTT
        client.publish("flame/detected", flameDetected ? "1" : "0");
        client.publish("sensor/reading", motionDetected ? "1" : "0");

        // Read and publish temperature and humidity
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        if (!isnan(temperature) && !isnan(humidity)) {
            client.publish("temp/reading", String(temperature).c_str());
            client.publish("humidity/reading", String(humidity).c_str());
            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.println("°C");
            Serial.print("Humidity: ");
            Serial.print(humidity);
            Serial.println("%");
        } else {
            Serial.println("Failed to read from DHT sensor!");
        }
    }

    // Non-blocking servo movement
    if (millis() - servoPreviousMillis >= servoInterval)
    {
        servoPreviousMillis = millis();
        myServo.write(myServo.read() == 0 ? 90 : 0);
    }
}
