#include <QTRSensors.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WebSerial.h>

// -------- LINE SENSOR SETUP --------
QTRSensors qtr;
const uint8_t SensorCount = 3;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {26, 27, 14};
const int weights[3] = {-1, 0, 1};   // Precomputed weights

// -------- MOTOR PINS --------
int IN1 = 32;
int IN2 = 33;
int IN3 = 25;
int IN4 = 12;

// -------- BUTTON AND LED --------
int startButton = 4;
int whiteLED = 18;
int blackLED = 19;

// -------- CALIBRATION --------
uint16_t whiteValues[SensorCount] = {0,0,0};
uint16_t blackValues[SensorCount] = {0,0,0};
uint16_t thresholds[SensorCount];

// -------- MOTOR CONTROL --------
int baseSpeed = 255;
float Kp = 0.6;
float Kd = 1.0;

float previousError = 0;
int lastDirection = 0;

// -------- ROBOT STATE --------
bool robotRunning = true;

// -------- DEBUG TIMER --------
unsigned long lastDebug = 0;

// -------- WIFI + WEBSERIAL --------
const char* ssid = "2sDIZONRobot";
const char* password = "12345678";
AsyncWebServer server(80);

// -------- FUNCTIONS --------
void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN3, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void recvMsg(uint8_t *data, size_t len) {

  String message = "";
  for(int i = 0; i < len; i++) message += char(data[i]);
  message.trim();

  Serial.print("Received: ");
  Serial.println(message);
  WebSerial.print("Received: ");
  WebSerial.println(message);

  // -------- STOP --------
  if(message == "STOP"){
    robotRunning = false;
    stopMotors();
    WebSerial.println("Robot Stopped");
  }

  // -------- START --------
  else if(message == "START"){
    robotRunning = true;
    WebSerial.println("Robot Running");
  }

  // -------- CHANGE KP --------
  else if(message.startsWith("KP")){
    float newKp = message.substring(3).toFloat();
    Kp = newKp;

    WebSerial.print("New Kp: ");
    WebSerial.println(Kp);
  }

  // -------- CHANGE KD --------
  else if(message.startsWith("KD")){
    float newKd = message.substring(3).toFloat();
    Kd = newKd;

    WebSerial.print("New Kd: ");
    WebSerial.println(Kd);
  }

  // -------- CHANGE BASE SPEED --------
  else if(message.startsWith("BASE")){
    int newBase = message.substring(5).toInt();
    baseSpeed = newBase;

    WebSerial.print("New Base Speed: ");
    WebSerial.println(baseSpeed);
  }

  // -------- SHOW CURRENT VALUES --------
  else if(message == "PD"){
    WebSerial.print("Kp: ");
    WebSerial.println(Kp);

    WebSerial.print("Kd: ");
    WebSerial.println(Kd);

    WebSerial.print("BaseSpeed: ");
    WebSerial.println(baseSpeed);
  }

  else{
    WebSerial.println("Unknown command");
  }
}
// -------- SETUP --------
void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  Serial.println();
  Serial.println("WiFi Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  WebSerial.begin(&server);
  WebSerial.onMessage(recvMsg);
  server.begin();
  WebSerial.println("ESP32 Robot Online");

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(startButton, INPUT_PULLUP);
  pinMode(whiteLED, OUTPUT);
  pinMode(blackLED, OUTPUT);

  Serial.println("Press the button to start calibration...");
  WebSerial.println("Press the button to start calibration...");
  while(digitalRead(startButton) == HIGH);
  delay(300);

  // WHITE CALIBRATION
  digitalWrite(whiteLED, HIGH);
  WebSerial.println("White calibration...");
  delay(3000);

  for(int i = 0; i < 20; i++){
    qtr.read(sensorValues);
    for(int j = 0; j < SensorCount; j++)
      whiteValues[j] += sensorValues[j];
    delay(50);
  }

  for(int j = 0; j < SensorCount; j++)
    whiteValues[j] /= 20;

  digitalWrite(whiteLED, LOW);

  // BLACK CALIBRATION
  digitalWrite(blackLED, HIGH);
  WebSerial.println("Black calibration...");
  delay(3000);

  for(int i = 0; i < 20; i++){
    qtr.read(sensorValues);
    for(int j = 0; j < SensorCount; j++)
      blackValues[j] += sensorValues[j];
    delay(50);
  }

  for(int j = 0; j < SensorCount; j++)
    blackValues[j] /= 20;

  digitalWrite(blackLED, LOW);

  for(int j = 0; j < SensorCount; j++)
    thresholds[j] = (whiteValues[j] + blackValues[j]) / 2;

  Serial.println("Calibration Complete!");
  WebSerial.println("Calibration Complete!");
}

// -------- LOOP --------
void loop() {

  if(!robotRunning){
    stopMotors();
    return;
  }

  qtr.read(sensorValues);

  bool lineDetected = false;

  for(int i = 0; i < SensorCount; i++){
    if(sensorValues[i] > thresholds[i]){
      lineDetected = true;
      break;
    }
  }

  // -------- LINE LOST RECOVERY --------
  if(!lineDetected){

    int recoverSpeed = 230;

    if(lastDirection >= 0){
      analogWrite(IN1,200);
      digitalWrite(IN2,HIGH);
      analogWrite(IN3,recoverSpeed);
      digitalWrite(IN4,LOW);
    }
    else{
      analogWrite(IN1,recoverSpeed);
      digitalWrite(IN2,LOW);
      analogWrite(IN3,200);
      digitalWrite(IN4,HIGH);
    }

    if(millis() - lastDebug > 120){
      lastDebug = millis();
      Serial.println("Line lost - recovering");
      WebSerial.println("Line lost - recovering");
    }

    return;
  }

  // -------- ERROR CALCULATION --------
  float position = 0;
  float total = 0;

  for(int i = 0; i < SensorCount; i++){
    int value = sensorValues[i] - thresholds[i];
    if(value < 0) value = 0;

    position += value * weights[i];
    total += value;
  }

  static float filteredError = 0;
  float error = (total > 0) ? position / total : 0;

  filteredError = (0.7 * filteredError) + (0.3 * error);
  error = filteredError;

  if ((error > 0 && previousError < 0) || (error < 0 && previousError > 0)) {
    previousError = error * 0.5;
  }
  
  if(error > 0.2) lastDirection = 1;
  else if(error < -0.2) lastDirection = -1;

  // -------- PD CONTROL --------
  float derivative = error - previousError;
  float correction = (Kp * error) + (Kd * derivative);
  previousError = error;

  float turnFactor = (error > 0) ? error : -error;

  int dynamicBaseSpeed = baseSpeed;

  int adjust = correction * 170;

  int speedLeft  = dynamicBaseSpeed - adjust;
  int speedRight = dynamicBaseSpeed + adjust;

  speedLeft  = constrain(speedLeft,150,255);
  speedRight = constrain(speedRight,150,255);

  // -------- MOTOR DRIVE --------
  analogWrite(IN1,speedLeft);
  digitalWrite(IN2,LOW);

  analogWrite(IN3,speedRight);
  digitalWrite(IN4,LOW);

  // -------- DEBUG (RATE LIMITED) --------
  if(millis() - lastDebug > 80){

    lastDebug = millis();

    Serial.print("Error: "); Serial.print(error);
    Serial.print(" Speeds: "); Serial.print(speedLeft);
    Serial.print(","); Serial.println(speedRight);

    WebSerial.print("Error: "); WebSerial.print(error);
    WebSerial.print(" Speeds: "); WebSerial.print(speedLeft);
    WebSerial.print(","); WebSerial.println(speedRight);
  }
}
