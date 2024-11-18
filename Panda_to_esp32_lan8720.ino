#include <ETH.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFiUdp.h>

// Ethernet Settings for LAN8720
#define ETH_PHY_ADDR 1       
#define ETH_PHY_POWER -1     
#define ETH_PHY_MDC 23       
#define ETH_PHY_MDIO 18      
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN 

WiFiUDP udp;
const int localPort = 6000;  

// Static IP Configuration for Ethernet
IPAddress local_IP(192,168,0,125);    
IPAddress gateway(192,168,0,1);       
IPAddress subnet(255,255,255,0);      

static bool eth_connected = false;  

// I2C Settings for PCA9685
const int SDA_PIN = 4;  
const int SCL_PIN = 16; 

// PCA9685 Setup
Adafruit_PWMServoDriver pwmControllers[4] = {
    Adafruit_PWMServoDriver(0x40),
    Adafruit_PWMServoDriver(0x41),
    Adafruit_PWMServoDriver(0x42),
    Adafruit_PWMServoDriver(0x43)
};
const int freq = 1526; 

// DMX/PWM Channel Values
const int totalChannels = 64;
int pwmValues[totalChannels];  

void EthernetEvent(WiFiEvent_t event);

// Task handles
TaskHandle_t udpTaskHandle = NULL;
TaskHandle_t pwmTaskHandle = NULL;

// Ethernet Event Handler
void EthernetEvent(WiFiEvent_t event) {
    switch (event) {
    case ARDUINO_EVENT_ETH_START:
        Serial.println("ETH Started");
        ETH.setHostname("esp32-ethernet");
        if (!ETH.config(local_IP, gateway, subnet)) {
            Serial.println("Failed to configure static IP");
        }
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        Serial.println("ETH Connected");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        Serial.print("ETH MAC: ");
        Serial.print(ETH.macAddress());
        Serial.print(", IPv4: ");
        Serial.print(ETH.localIP());
        Serial.print(", Speed: ");
        Serial.print(ETH.linkSpeed());
        Serial.println(" Mbps");
        eth_connected = true;
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        Serial.println("ETH Disconnected");
        eth_connected = false;
        break;
    case ARDUINO_EVENT_ETH_STOP:
        Serial.println("ETH Stopped");
        eth_connected = false;
        break;
    default:
        break;
    }
}

// Function to process incoming UDP data
void processUDPData(byte* packetBuffer, int length) {
    Serial.print("Processing packet of length: ");
    Serial.println(length);

    int numValues = 0;
    char *token = strtok((char *)packetBuffer, " ");
    while (token != NULL && numValues < totalChannels) {
        pwmValues[numValues] = atoi(token);
        numValues++;
        token = strtok(NULL, " ");
    }

    if (numValues == totalChannels) {
        Serial.println("PWM values ready for update.");
        // Notify PWM Task to update values
        xTaskNotifyGive(pwmTaskHandle);
    } else {
        Serial.printf("Error parsing values. Parsed %d values.\n", numValues);
    }
}

// Task to listen for incoming UDP traffic
void UDPTrafficTask(void *pvParameters) {
    while (1) {
        if (eth_connected) {
            int packetSize = udp.parsePacket();
            if (packetSize > 0) {
                byte packetBuffer[512];  
                udp.read(packetBuffer, packetSize);  
                processUDPData(packetBuffer, packetSize);  
            }
        } else {
            Serial.println("Ethernet is not connected.");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Yield task for 10ms
    }
}

// Task to update PWM values on PCA9685
void PWMUpdateTask(void *pvParameters) {
    while (1) {
        // Wait for notification to update PWM
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Block task until notification is received

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 16; j++) {
                int pwmValue = pwmValues[i * 16 + j];
                pwmControllers[i].setPWM(j, 0, pwmValue);
            }
        }
        Serial.println("PWM values updated.");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Setup started");

    // I2C Settings for PCA9685
    pinMode(SDA_PIN, INPUT_PULLUP); // SDA
    pinMode(SCL_PIN, INPUT_PULLUP); // SCL

    Wire.begin(SDA_PIN, SCL_PIN);
    for (int i = 0; i < 4; i++) {
        pwmControllers[i].begin();
        pwmControllers[i].setPWMFreq(freq);
    }

    // Start Ethernet with LAN8720
    WiFi.onEvent(EthernetEvent);  
    if (!ETH.begin(ETH_PHY_LAN8720, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE)) {
        Serial.println("Ethernet failed to start");
    } else {
        Serial.println("Ethernet started");
        udp.begin(localPort);  
        Serial.println("UDP listener started on port " + String(localPort));
    }

    // Create UDP Traffic Task
    xTaskCreate(UDPTrafficTask, "UDP Traffic Task", 4096, NULL, 1, &udpTaskHandle);

    // Create PWM Update Task
    xTaskCreate(PWMUpdateTask, "PWM Update Task", 2048, NULL, 1, &pwmTaskHandle);
}

void loop() {
    // The loop can be kept empty as tasks handle everything.
}
