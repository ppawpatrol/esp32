#include "esp_camera.h"
#include <WiFi.h>

// WiFi credentials – update with your network information:
const char* ssid = "Advait's S21 Ultra";
const char* password = "12345678";

// Flask server settings – update with your Flask server IP and port:
#define FLASK_SERVER "192.168.173.204"
#define FLASK_PORT 5000
#define FLASK_PATH "/upload"

// Boundary string used to separate form-data parts in the HTTP request:
const char* boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";

// Pin configuration for AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Initialize and configure the camera:
void initCamera() {
  camera_config_t config;
  
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  // Use higher resolution and two frame buffers if PSRAM is available:
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    // Stop further execution if camera init fails.
    while (true) { delay(1000); }
  }
}

// Connect to the WiFi network:
void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// This function sends the captured image to the Flask server:
void sendPhoto(camera_fb_t * fb) {
  if (fb == NULL) {
    Serial.println("No frame buffer to send.");
    return;
  }

  WiFiClient client;
  Serial.printf("Connecting to %s:%d\n", FLASK_SERVER, FLASK_PORT);
  if (!client.connect(FLASK_SERVER, FLASK_PORT)) {
    Serial.println("Connection to Flask server failed");
    return;
  }
  
  // Construct multipart/form-data parts
  String startPart = "";
  startPart += "--";
  startPart += boundary;
  startPart += "\r\n";
  startPart += "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n";
  startPart += "Content-Type: image/jpeg\r\n\r\n";
  
  String endPart = "\r\n--";
  endPart += boundary;
  endPart += "--\r\n";
  
  int contentLength = startPart.length() + fb->len + endPart.length();
  
  // Construct the HTTP POST request header:
  String httpRequest = "";
  httpRequest += "POST ";
  httpRequest += FLASK_PATH;
  httpRequest += " HTTP/1.1\r\n";
  httpRequest += "Host: ";
  httpRequest += FLASK_SERVER;
  httpRequest += "\r\n";
  httpRequest += "Content-Type: multipart/form-data; boundary=";
  httpRequest += boundary;
  httpRequest += "\r\n";
  httpRequest += "Content-Length: ";
  httpRequest += String(contentLength);
  httpRequest += "\r\n";
  httpRequest += "Connection: close\r\n\r\n";
  
  // Send header and multipart data:
  client.print(httpRequest);
  client.print(startPart);
  client.write(fb->buf, fb->len);
  client.print(endPart);
  
  // Optionally, read and print the server's response:
  unsigned long timeout = millis();
  while (client.available() == 0 && millis() - timeout < 5000) {
    delay(10);
  }
  while (client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  Serial.println("\nPhoto sent successfully.");
  client.stop();
}

void setup() {
  Serial.begin(115200);
  // Initialize the camera module
  initCamera();
  // Connect to WiFi
  connectWiFi();
}

void loop() {
  // Capture a frame from the camera
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(5000);
    return;
  }
  
  // Send the captured image to the Flask server
  sendPhoto(fb);
  
  // Return the frame buffer back to the driver for reuse
  esp_camera_fb_return(fb);
  
  // Wait for 5 seconds before capturing and sending the next image
  delay(5000);
}
