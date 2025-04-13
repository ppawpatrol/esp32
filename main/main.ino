#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Advait's S21 Ultra";
const char* password = "12345678";

const char* mqtt_server = "192.168.112.204";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/sensors";

const int pirPin = 14;

const int trigPin = 12;
const int echoPin = 13;

const int aqiPin = 34;

WiFiClient espClient;
PubSubClient client(espClient);

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

  Serial.println();
  Serial.print("WiFi connected - IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" - trying again in 5 seconds");
      delay(5000);
    }
  }
}

long readUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  
  long distance = duration * 0.034 / 2;
  return distance;
}

int readAQI() {
  return analogRead(34);
}

void setup() {
  Serial.begin(115200);

  pinMode(pirPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(aqiPin, INPUT);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int motionDetected = digitalRead(pirPin);
  long distance = readUltrasonicDistance();
  int aqi = readAQI();

  Serial.print("PIR Sensor: ");
  Serial.print(motionDetected);
  Serial.print(" | Ultrasonic Distance: ");
  Serial.print(distance);
  Serial.print(" cm | AQI: ");
  Serial.println(aqi);

  // edge filtering
  bool publishData = false;
  const int DISTANCE_THRESHOLD = 50;
  if (motionDetected == HIGH || distance < DISTANCE_THRESHOLD || aqi > 1400) {
    publishData = true;
  }

  if (publishData) {
    String payload = "{";
    payload += "\"motion\":" + String(motionDetected) + ",";
    payload += "\"distance_cm\":" + String(distance) + ",";
    payload += "\"aqi\":" + String(aqi);
    payload += "}";
    
    Serial.print("Publishing payload: ");
    Serial.println(payload);

    if (client.publish(mqtt_topic, payload.c_str())) {
      Serial.println("Message published successfully");
    } else {
      Serial.println("Message publish failed");
    }
  }

  delay(2000);
}
