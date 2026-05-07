#include <WiFiS3.h>
#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long lastTime = 0;
float dt;

float x, y, z;
float vx, vy, vz;

// Dynamic offsets
float ax_offset = 0;
float ay_offset = 0;
float az_offset = 0;

bool calibrated = false;

// WiFi
char ssid[] = "Kenti’s iPhone";
char pass[] = "12345678";

char server[] = "172.20.10.7";
int port = 8080;

WiFiClient client;

void connectWiFi() {
  Serial.println("Connecting to WiFi...");

  while (true) {
    WiFi.begin(ssid, pass);

    int attempts = 0;

    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected!");

      delay(3000);
      IPAddress ip = WiFi.localIP();

      if (ip != IPAddress(0, 0, 0, 0)) {
        Serial.print("Got IP: ");
        Serial.println(ip);
        return;
      } else {
        Serial.println("No IP assigned. Retrying...");
      }
    } else {
      Serial.println("\nConnection failed. Retrying...");
    }

    WiFi.disconnect();
    delay(2000);
  }
}

void calibrateMPU() {
  const int samples = 10;
  float sum_ax = 0;
  float sum_ay = 0;
  float sum_az = 0;

  Serial.println("Calibrating... keep still");

  for (int i = 0; i < samples; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    delay(10);
  }

  ax_offset = sum_ax / samples;
  ay_offset = sum_ay / samples;
  az_offset = sum_az / samples;

  Serial.println("Calibration done:");
  Serial.print("ax_offset: "); Serial.println(ax_offset);
  Serial.print("ay_offset: "); Serial.println(ay_offset);
  Serial.print("az_offset: "); Serial.println(az_offset);

  calibrated = true;
}

void setup() {
  Serial.begin(9600);
  delay(2000);

  Wire.begin();
  mpu.initialize();

  calibrateMPU();
  connectWiFi();

  lastTime = millis();
}

void loop() {
  if (!calibrated) return;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost. Reconnecting...");
    connectWiFi();
  }

  if (!client.connected()) {
    Serial.println("Connecting to server...");
    if (client.connect(server, port)) {
      Serial.println("Connected to server!");
    } else {
      Serial.println("Server connection failed");
      delay(1000);
      return;
    }
  }

  mpu.getAcceleration(&ax, &ay, &az);

  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float new_ax = ax - ax_offset;
  float new_ay = ay - ay_offset;
  float new_az = az - az_offset;

  if (abs(new_ax) < 300) {
    new_ax = 0;
  }
  if (abs(new_ay) < 300) {
    new_ay = 0;
  }
  if (abs(new_az) < 300) {
    new_az = 0;
  }
  // Integrate
  vx += new_ax * dt;
  vy += new_ay * dt;
  vz += new_az * dt;

  if (new_ax == 0) {
    vx = 0;
  }
  if (new_ay == 0) {
    vy = 0;
  }
  if (new_az == 0) {
    vz = 0;
  }

  x += vx * dt;
  y += vy * dt;
  z += vz * dt;

  client.print(x); 
  client.print("\n");
  client.print(y); 
  client.print("\n");
  client.print(z); 
  client.print("\n");
  Serial.print("Sent x: "); 
  Serial.println(x);
  Serial.print("Sent y: "); 
  Serial.println(y);
  Serial.print("Sent z: "); 
  Serial.println(z);
  delay(5);
}