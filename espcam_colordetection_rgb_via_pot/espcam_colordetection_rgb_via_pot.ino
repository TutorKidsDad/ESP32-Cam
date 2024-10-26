#include "esp_camera.h"
#include "img_converters.h"

#define CAMERA_MODEL_AI_THINKER // Use the AI-Thinker ESP32-CAM model

const int ledPin = 4; // Define the LED pin
const int redPotPin = 34; // Define the red potentiometer pin
const int greenPotPin = 35; // Define the green potentiometer pin
const int bluePotPin = 32; // Define the blue potentiometer pin

int redThreshold = 128; // Initialize the red threshold
int greenThreshold = 128; // Initialize the green threshold
int blueThreshold = 128; // Initialize the blue threshold

void setup() {
  pinMode(ledPin, OUTPUT); // Set the LED pin as an output
  digitalWrite(ledPin, HIGH); // Turn on the LED
  pinMode(redPotPin, INPUT); // Set the red potentiometer pin as an input
  pinMode(greenPotPin, INPUT); // Set the green potentiometer pin as an input
  pinMode(bluePotPin, INPUT); // Set the blue potentiometer pin as an input
  Serial.begin(115200); // Initialize serial communication
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void loop() {
  camera_fb_t * fb = esp_camera_fb_get(); // Get a frame from the camera
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  uint8_t *image = fb->buf; // Get the image data
  int redSum = 0; // Initialize the red sum variable
  int greenSum = 0; // Initialize the green sum variable
  int blueSum = 0; // Initialize the blue sum variable
  for (int i = 0; i < fb->len; i += 2) {
    redSum += image[i]; // Add the red pixel value to the red sum
    greenSum += image[i + 1]; // Add the green pixel value to the green sum
    blueSum += image[i + 2]; // Add the blue pixel value to the blue sum
  }
  int redAverage = redSum / (fb->len / 2); // Calculate the average red pixel value
  int greenAverage = greenSum
/ (fb->len / 2); // Calculate the average green pixel value
int blueAverage = blueSum / (fb->len / 2); // Calculate the average blue pixel value
if (redAverage > redThreshold && greenAverage > greenThreshold && blueAverage > blueThreshold) {
digitalWrite(ledPin, LOW); // Turn off the LED
} else {
digitalWrite(ledPin, HIGH); // Turn on the LED
}
Serial.printf("Red: %d Green: %d Blue: %d\n", redAverage, greenAverage, blueAverage); // Print the average pixel values
delay(100); // Wait for 100ms
redThreshold = map(analogRead(redPotPin), 0, 4095, 0, 255); // Map the red potentiometer reading to the red threshold value
greenThreshold = map(analogRead(greenPotPin), 0, 4095, 0, 255); // Map the green potentiometer reading to the green threshold value
blueThreshold = map(analogRead(bluePotPin), 0, 4095, 0, 255); // Map the blue potentiometer reading to the blue threshold value
}
