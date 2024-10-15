#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <PID_v1_bc.h>

const char* ssid = "PTIT.HCM_CanBo";
const char* password = "";

#define MIN_PULSE_LENGTH 1520 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

#define I2C_SDA_1 21
#define I2C_SCL_1 22
#define I2C_SDA_2 3
#define I2C_SCL_2 1

Servo motA, motB, motC, motD;
AsyncWebServer server(80);

TwoWire I2C_2 = TwoWire(1); 

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp(&I2C_2); // I2C

void setupWiFi();
void setupMotors();
void setupSensors();
String controlPage();
String sensorPage();
String getSensorJSON();

double pitch, roll, yaw;
double altitude;

double setpointPitch = 0, inputPitch, outputPitch;
double setpointRoll = 0, inputRoll, outputRoll;
double setpointAltitude = 0, inputAltitude, outputAltitude;

double Kp = 1, Ki = 0.05, Kd = 0.01;

PID pidPitch(&inputPitch, &outputPitch, &setpointPitch, Kp, Ki, Kd, DIRECT);
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, Kp, Ki, Kd, DIRECT);
PID pidAltitude(&inputAltitude, &outputAltitude, &setpointAltitude, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);

  setupWiFi();
  setupMotors();
  setupSensors();

  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetMode(AUTOMATIC);
  pidAltitude.SetMode(AUTOMATIC);

  // Motor control page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", controlPage());
  });

  // Sensor page
  server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", sensorPage());
  });

  // JSON endpoint for sensor data
  server.on("/sensor-data", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getSensorJSON());
  });

  // Motor update page
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    int valueA = request->hasParam("motA") ? request->getParam("motA")->value().toInt() : 1520;
    int valueB = request->hasParam("motB") ? request->getParam("motB")->value().toInt() : 1520;
    int valueC = request->hasParam("motC") ? request->getParam("motC")->value().toInt() : 1520;
    int valueD = request->hasParam("motD") ? request->getParam("motD")->value().toInt() : 1520;
    double userAltitude = request->hasParam("altitude") ? request->getParam("altitude")->value().toDouble() : 0;

    // Set the altitude setpoint from user input
    setpointAltitude = userAltitude;

    motA.writeMicroseconds(valueA);
    motB.writeMicroseconds(valueB);
    motC.writeMicroseconds(valueC);
    motD.writeMicroseconds(valueD);

    request->send(200, "text/plain", "Motors and altitude updated");
  });

  server.begin();
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  roll = atan2(-a.acceleration.x, a.acceleration.z) * 180 / PI;
  altitude = bmp.readAltitude(1013.25);
  inputPitch = pitch;
  inputRoll = roll;
  inputAltitude = altitude;

  pidPitch.Compute();
  pidRoll.Compute();
  pidAltitude.Compute();

  updateMotorSpeeds(outputPitch, outputRoll, outputAltitude);
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
}

void setupMotors() {
  motA.attach(13, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(14, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motD.attach(27, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

  motA.writeMicroseconds(1520);
  motB.writeMicroseconds(1520);
  motC.writeMicroseconds(1520);
  motD.writeMicroseconds(1520);
}

void setupSensors() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip on I2C_1");
  }

  I2C_2.begin(I2C_SDA_2, I2C_SCL_2);
  if (!bmp.begin(0x77)) {
    Serial.println("Could not find a valid BMP280 sensor on I2C_2");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, 
                  Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, 
                  Adafruit_BMP280::STANDBY_MS_500);
}

void updateMotorSpeeds(double pitchOutput, double rollOutput, double altitudeOutput) {
  int pwmA = 1520 + pitchOutput - rollOutput + altitudeOutput;
  int pwmB = 1520 - pitchOutput - rollOutput + altitudeOutput;
  int pwmC = 1520 + pitchOutput + rollOutput + altitudeOutput;
  int pwmD = 1520 - pitchOutput + rollOutput + altitudeOutput;

  motA.writeMicroseconds(constrain(pwmA, 1520, 2000));
  motB.writeMicroseconds(constrain(pwmB, 1520, 2000));
  motC.writeMicroseconds(constrain(pwmC, 1520, 2000));
  motD.writeMicroseconds(constrain(pwmD, 1520, 2000));
}

// HTML page to display sensor data with auto-refresh
String sensorPage() {
  String page = "<html><body><h1>Sensor Data</h1>";
  page += "<div id='sensorData'></div>";
  page += "<script>";
  page += "setInterval(function() {";
  page += "fetch('/sensor-data').then(response => response.json()).then(data => {";
  page += "document.getElementById('sensorData').innerHTML = `";
  page += "<h2>MPU6050</h2>";
  page += "Accel X: ${data.accelX} m/s^2<br>";
  page += "Accel Y: ${data.accelY} m/s^2<br>";
  page += "Accel Z: ${data.accelZ} m/s^2<br>";
  page += "Gyro X: ${data.gyroX} rad/s<br>";
  page += "Gyro Y: ${data.gyroY} rad/s<br>";
  page += "Gyro Z: ${data.gyroZ} rad/s<br>";
  page += "Temperature (MPU6050): ${data.tempMPU} &deg;C<br>";
  page += "<h2>BMP280</h2>";
  page += "Temperature: ${data.tempBMP} &deg;C<br>";
  page += "Pressure: ${data.pressure} hPa<br>";
  page += "Altitude: ${data.altitude} m<br>";
  page += "`;";
  page += "});";
  page += "}, 100);"; // Refresh every second
  page += "</script>";
  page += "</body></html>";
  return page;
}

// JSON response for sensor data
String getSensorJSON() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  String json = "{";
  json += "\"accelX\": " + String(a.acceleration.x) + ",";
  json += "\"accelY\": " + String(a.acceleration.y) + ",";
  json += "\"accelZ\": " + String(a.acceleration.z) + ",";
  json += "\"gyroX\": " + String(g.gyro.x) + ",";
  json += "\"gyroY\": " + String(g.gyro.y) + ",";
  json += "\"gyroZ\": " + String(g.gyro.z) + ",";
  json += "\"tempMPU\": " + String(t.temperature) + ",";
  json += "\"tempBMP\": " + String(bmp.readTemperature()) + ",";
  json += "\"pressure\": " + String(bmp.readPressure() / 100.0) + ",";
  json += "\"altitude\": " + String(bmp.readAltitude(1013.25));
  json += "}";
  return json;
}

// HTML control page for motor control
String controlPage() {
  String page = "<html><body><h1>Motor Control</h1>";
  page += "<form action='/update' method='get'>";
  page += "Motor A: <input type='number' name='motA'><br>";
  page += "Motor B: <input type='number' name='motB'><br>";
  page += "Motor C: <input type='number' name='motC'><br>";
  page += "Motor D: <input type='number' name='motD'><br>";
  page += "Altitude: <input type='number' name='altitude' step='0.1'><br>";
  page += "<input type='submit' value='Update'>";
  page += "</form>";
  page += "</body></html>";
  return page;
}
