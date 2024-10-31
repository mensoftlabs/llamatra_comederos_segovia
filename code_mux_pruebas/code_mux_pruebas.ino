#include <HardwareSerial.h>

// Define control pins for multiplexer
#define PIN_A 18
#define PIN_B 5
#define PIN_C 22

// Define serial communication settings
#define NUM_SENSE 5  // Number of sensors
#define BUF_SIZE 64  // Buffer size

int canal = 0;
HardwareSerial SensorSerial(1);  // Use Serial1 on ESP32

// Function to set the multiplexer channel
void setMultiplexerChannel(int channel) {
    digitalWrite(PIN_A, channel & 0x01);
    digitalWrite(PIN_B, (channel >> 1) & 0x01);
    digitalWrite(PIN_C, (channel >> 2) & 0x01);
}

// Function to initialize UART and multiplexer pins
void setup() {
    Serial.begin(9600);
    SensorSerial.begin(9600, SERIAL_8N1, 19, 13); // RX on GPIO 16, TX on GPIO 17
    delay(200);
    Serial.println("");
    Serial.println("NUEVA PRUEBA");

    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    pinMode(PIN_C, OUTPUT);
}

// Function to read sensor and display distance
void readSensor() {
    setMultiplexerChannel(canal);  // Set the current sensor channel
    delay(10);  // Short delay for stable reading

    if (SensorSerial.available() >= 4) {  // Assuming data is 4 bytes
        uint8_t header = SensorSerial.read();
        if (header == 0xFF) {
            uint8_t Data_H = SensorSerial.read();
            uint8_t Data_L = SensorSerial.read();
            uint8_t checksum = SensorSerial.read();

            if ((0xFF + Data_H + Data_L) == checksum) {
                int distance = (Data_H << 8) | Data_L;
                Serial.print("Sensor ");
                Serial.print(canal + 1);
                Serial.print(": ");
                Serial.print(distance);
                Serial.println(" mm");
            }
        }
    }
    canal = (canal + 1) % NUM_SENSE;  // Move to the next sensor
}

void loop() {
    readSensor();
    delay(3000);  // Wait 3 seconds before reading next sensor
}
