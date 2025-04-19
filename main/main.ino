#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/i2s.h>
#include "driver/adc.h"

// WiFi credentials
const char* ssid = "Advait's S21 Ultra";
const char* password = "12345678";

// Server
const char* mqtt_server = "192.168.173.204";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/sensors";

// Sensor pins
#define PIR_PIN     14
#define TRIG_PIN    12
#define ECHO_PIN    13
#define AQI_PIN     34  // Using GPIO34 for AQI sensor (ADC1 channel 6)

// I2S mic pins
#define I2S_WS  25
#define I2S_SD  33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0
#define BUFFER_LEN 256

int16_t sBuffer[BUFFER_LEN];
WiFiClient espClient;
PubSubClient client(espClient);

// 🔧 I2S Config
void setupI2S() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_start(I2S_PORT);
}

// 📡 WiFi Setup
void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

// 🔁 MQTT Setup
void connectMQTT() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
            Serial.println("MQTT Connected");
        } else {
            Serial.print("MQTT failed, rc=");
            Serial.print(client.state());
            Serial.println(" - trying again in 5 seconds");
            delay(5000);
        }
    }
}

// 🎤 AUDIO TASK
void audioTask(void *param) {
  WiFiClient audioClient;
  while (!audioClient.connect("192.168.173.204", 8080)) {
    Serial.println("i WILL kms");
    delay(1000);
  }

  while (1) {
    size_t bytesRead;
    if (i2s_read(I2S_PORT, sBuffer, sizeof(sBuffer), &bytesRead, portMAX_DELAY) == ESP_OK && bytesRead > 0) {
      audioClient.write((uint8_t *)sBuffer, bytesRead);
    }
  }
}

// 📈 SENSOR TASK
void sensorTask(void *param) {
  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Setup ADC for AQI Sensor (GPIO 34 -> ADC1 channel 6)
  adc1_config_width(ADC_WIDTH_BIT_12);  // Set ADC width to 12 bits
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);  // Set ADC attenuation to 0 dB for 0-1V range

  while (1) {
    if (!client.connected()) connectMQTT();
    client.loop();

    int motion = digitalRead(PIR_PIN);

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    long distance = duration * 0.034 / 2;

    // Read AQI sensor using ADC API (on GPIO34 -> ADC1 channel 6)
    int aqi = adc1_get_raw(ADC1_CHANNEL_6);  // Use ADC1 API to get raw data

    if (motion || distance < 50 || aqi > 1400) {
      String payload = "{";
      payload += "\"motion\":" + String(motion) + ",";
      payload += "\"distance_cm\":" + String(distance) + ",";
      payload += "\"aqi\":" + String(aqi);
      payload += "}";

      client.publish(mqtt_topic, payload.c_str());
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Delay here is OK
  }
}

// 🛠 SETUP
void setup() {
  Serial.begin(115200);
  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  setupI2S();

xTaskCreatePinnedToCore(audioTask, "Audio Task", 4096, NULL, 1, NULL, 0);
Serial.println("Audio Task Created");

xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 4096, NULL, 1, NULL, 1);
Serial.println("Sensor Task Created");
}

void loop() {
}