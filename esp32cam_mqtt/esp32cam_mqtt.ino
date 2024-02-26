#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Used to disable brownout detection
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp32cam.h"

int loop_counter = 0;

const char* ssid = "yourssid";
const char* password = "yourpw";
const char* mqtt_server = "192.168.0.1"; //adress of your broker
int myIPAdress = 50; //ip adress of this device 192.168.0.xxx
String prepend = "esp32auto/" + String(myIPAdress);

bool STREAM = false;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

//motor pins
int ENA = 2; // Enable pin for motor A
int ENB = 3; // Enable pin for motor B
int IN1 = 12; // Input 1 for motor A
int IN2 = 13; // Input 2 for motor A
int IN3 = 15; // Input 4 for motor B
int IN4 = 14; // Input 3 for motor B
int pwmChannelA = 5;
int pwmChannelB = 3;
int pwmResolutionBit = 13;
int pwmMax = (2 << pwmResolutionBit) - 1;
// kill timer for motor if connection is lost
long deadman_time = 0;
long deadman_cutoff = 1 * 1000; // if we dont get a deadman heartbeat in this timeframe motor will stop

//will return the average FPS in a set interval
float measureFps(int frame_interval = 13) {
  static long count = 0;
  static long total_time = 0;
  static long last_time = 0;
  static float average_fps = 0;

  long current_time = millis();
  if (last_time == 0) {
    last_time = current_time;
  }
  total_time += current_time - last_time;
  count++;
  last_time = current_time;

  if (count % frame_interval == 0) {
    long average_time = total_time / count;
    if (average_time > 0) {
      average_fps = frame_interval / average_time;
    } else {
      average_fps = 0;
    }
    Serial.print("average_fps ");
    Serial.println(average_fps);
    count = 0;
    total_time = 0;
  }
  return average_fps;
}

// receiving a message
void callback(char* topic, byte* payload, unsigned int length) {
  // Print topic
  Serial.print("Received message on topic: ");
  Serial.println(topic);

  //convert message to string and print
  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("Message: ");
  Serial.println(msg);
  if (strcmp(topic, (prepend + "/set/resolution").c_str()) == 0) {
    setResolution(msg);
  } else if (strcmp(topic, (prepend + "/set/quality").c_str()) == 0) {
    setQuality(msg.toInt());
  } else if (strcmp(topic, (prepend + "/set/fps").c_str()) == 0) {
    FPS = msg.toFloat();
  } else if (strcmp(topic, (prepend + "/set/fps").c_str()) == 0) {
    FPS = msg.toFloat();
  } else if (strcmp(topic, (prepend + "/set/led").c_str()) == 0) {
    double brightness = msg.toDouble();
    setLEDBrightness(brightness);
  } else if (strcmp(topic, (prepend + "/camera/shot").c_str()) == 0) {
    sendMqttFrame(&mqtt_client);
  } else if (strcmp(topic, (prepend + "/camera/stream/start").c_str()) == 0) {
    STREAM = true;
  } else if (strcmp(topic, (prepend + "/camera/stream/stop").c_str()) == 0) {
    STREAM = false;
  } else if (strcmp(topic, (prepend + "/motor/speed/a").c_str()) == 0) {
    controlMotors(1,  msg.toFloat());
  } else if (strcmp(topic, (prepend + "/motor/speed/b").c_str()) == 0) {
    controlMotors(2,  msg.toFloat());
  } else if (strcmp(topic, (prepend + "/motor/deadman/heartbeat").c_str()) == 0) {
    deadman_time  = millis(); // update the deadmanswitch
    // TODO implement deadman switch for motors
  }
}


/*
   speed_norm = [-1.0; 1.0]
   motor_pin = [1; 2]
*/
void controlMotors(int motor_pin, float speed_norm) {

  Serial.print("input ");
  Serial.print(speed_norm);
  // map the values from 0 to 1 to 0 to pwmMax
  int speed_int = constrain(abs(speed_norm) * pwmMax/2, 0, pwmMax/2);// for whatever reason esp wants 1/2

  Serial.print(" output ");
  Serial.print(speed_int);
  Serial.println();

  // which motor
  if (motor_pin == 1) {
    // Determine the direction of the motors
    if (speed_norm >= 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
    // Set the speed of the motor
    ledcWrite(pwmChannelA, speed_int);
  }

  else if (motor_pin == 2) {
    if (speed_norm >= 0) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    // Set the speed of the motor
    ledcWrite(pwmChannelB, speed_int);
  }
}

void setIPAddress(uint8_t ipAddress) {
  // Try each IP address until one is set successfully
  IPAddress staticIP(192, 168, 0, ipAddress);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);

  // Set the static IP address, gateway, and subnet mask for the ESP32
  if (!WiFi.config(staticIP, gateway, subnet)) {
    Serial.print("ERROR setting IP address ");
    Serial.println(staticIP.toString());
  }
}

void checkMQTTConnection(PubSubClient *mqttClient) {
  // Check if the client is connected
  if (!mqtt_client.connected()) {
    // If not connected, attempt to connect to the MQTT broker
    Serial.println("Connecting to MQTT broker...");
    mqttClient->setServer(mqtt_server, 1883);
    while (!mqttClient->connected()) {
      if (mqttClient->connect("ESP32-CAM")) {
        Serial.println("Connected to broker");

        // Set the callback function
        mqttClient->setCallback(callback);

        // Subscribe to the esp32cam/set topic
        mqttClient->subscribe((prepend + "/#").c_str());

        // Publish message to MQTT topic, including the local WiFi IP address
        String message = "online on ip " + WiFi.localIP().toString() + " topic: " + prepend;
        mqttClient->publish("esp32cam", message.c_str());
        Serial.println("Connected to broker SUCCESSFULLY");

      } else {
        Serial.print("Failed to connect, status code = ");
        Serial.println(mqttClient->state());
        Serial.println("Retrying in 3 seconds...");
        delay(3000);
      }
    }
  }
  //  Serial.println("Connected to broker");
}

void checkWiFiConnection() {
  // Check if the ESP32-CAM is connected to WiFi
  if (WiFi.status() != WL_CONNECTED) {
    // Attempt to reconnect to WiFi in a loop
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin();
      Serial.print("Connecting to WiFi... ");
      delay(3000);
    }
    // If the connection is successful, print the local IP address
    Serial.println("Connected to WiFi");
    setIPAddress(myIPAdress);
    Serial.println("IP address: " + WiFi.localIP().toString());
  } else {
    //    Serial.println("Connected to WiFi");
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);     // Turn-off the 'brownout detector'



  Serial.begin(115200);  // Initialize serial communication at 115200 baud
  Serial.println("\n\n\n\n\n\n\n\n\n\n");
  Serial.println("booting...\n");

//  setup motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcSetup(pwmChannelA, 5000, pwmResolutionBit);
  ledcAttachPin(ENA, pwmChannelA);
  ledcSetup(pwmChannelB, 5000, pwmResolutionBit);
  ledcAttachPin(ENB, pwmChannelB);

  // FLASHLIGHT setup
  pinMode(ledPin, OUTPUT);  // Set the LED pin as an output
  ledcSetup(ledcChannel, ledcFrequency, led_bit);  // Initialize the LEDC channel
  ledcAttachPin(ledPin, ledcChannel);  // Attach the LED pin to the LEDC channel


  // setup camera
  cam_setup();
  //  setLEDBrightness(0);  // Turn off the LED

  // Connect to WiFi
  checkWiFiConnection();

  // Connect to MQTT broker
  checkMQTTConnection(&mqtt_client);

}

void loop() {
  
  // Check connections and update mqtt loop
  if (loop_counter % 60 == 0) {
    checkWiFiConnection();
    checkMQTTConnection(&mqtt_client);
  }
  mqtt_client.loop();
  if (STREAM) {
    sendMqttFrame(&mqtt_client);
  } else {
    delay(100); // maybe replace with a sleep?
  }
  loop_counter++;
}
