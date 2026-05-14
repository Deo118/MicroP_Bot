#include <QTRSensors.h>

// QTR sensor

QTRSensors qtr;

const uint8_t SensorCount = 8;

const uint8_t sensorPin[8] =
{A0,A1,A2,A3,A4,A5,A6,A7};

int sensorVal[8];

// TB6612FNG

const int PWMA = 5;
const int PWMB = 6;

const int AIN1 = 2;
const int AIN2 = 3;

const int BIN1 = 8;
const int BIN2 = 7;

const int STBY = 4;

// Buttons and buzzer

const int BTN_CALI = 12;
const int BTN_RUN  = 11;

const int BUZZER = 13;

// Calibration

uint16_t whiteValues[8] = {0};
uint16_t blackValues[8] = {0};

// Position

long position = 3500;
long sensorSum = 0;

// PID

long lastError = 0;

// Normal PID

float Kp_normal = 0.055;
float Kd_normal = 3.5;

// Straight PID

float Kp_straight = 0.030;
float Kd_straight = 1.5;

// Turn PID

float Kp_turn = 0.095;
float Kd_turn = 2.0;

// Diamond PID

float Kp_diamond = 0.025;
float Kd_diamond = 6.5;

// Intersection PID

float Kp_intersection = 0.020;
float Kd_intersection = 7.0;

// Speed

int speed_straight  = 255;
int speed_normal    = 220;
int speed_turn      = 170;
int speed_diamond   = 190;
int speed_intersect = 200;

// Lost line

#define LOST_LINE_THRESHOLD 1500

int lastDir = 0;

int recoverSpeed = 180;
int recoverReverse = 170;

// Robot state

bool calibrated = false;
bool running = false;

// Beep

void beep(int times){

  for(int i=0;i<times;i++){

    tone(BUZZER,2500);
    delay(100);

    noTone(BUZZER);
    delay(100);
  }
}

void longBeep(int duration){

  tone(BUZZER,2500);
  delay(duration);

  noTone(BUZZER);
}

// Motor

void motor(int left, int right){

  if(left >= 0){

    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);

    analogWrite(PWMA,left);
  }
  else{

    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);

    analogWrite(PWMA,-left);
  }

  if(right >= 0){

    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);

    analogWrite(PWMB,right);
  }
  else{

    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);

    analogWrite(PWMB,-right);
  }
}

// Stop motor

void stopMotor(){

  analogWrite(PWMA,0);
  analogWrite(PWMB,0);

  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);

  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,LOW);
}

// Read sensor

void readSensor(){

  long weighted = 0;
  long sum = 0;

  for(int i=0;i<8;i++){

    int v = analogRead(sensorPin[i]);

    v = map(v,
            whiteValues[i],
            blackValues[i],
            0,
            1000);

    v = constrain(v,0,1000);

    if(v < 50)
      v = 0;

    sensorVal[i] = v;

    sum += v;

    weighted +=
    (long)v * (i * 1000);
  }

  sensorSum = sum;

  if(sum > 0){

    position = weighted / sum;
  }
}

// Lost line

bool lostLine(){

  return(sensorSum < LOST_LINE_THRESHOLD);
}

// Line follow

void lineFollow(){

  long target = 3500;

  long error =
  position - target;

  long derivative =
  error - lastError;

  bool straight =
  (abs(error) < 120 &&
   abs(derivative) < 80);

  bool sharpTurn =
  (abs(error) > 2500);

  bool zigzag =
  (abs(error) > 1200 &&
   abs(error) < 2500);

  bool fullBlack =
  (sensorVal[0] > 600 &&
  sensorVal[1] > 600 &&
  sensorVal[2] > 600 &&
  sensorVal[3] > 600 &&
  sensorVal[4] > 600 &&
  sensorVal[5] > 600 &&
  sensorVal[6] > 600 &&
  sensorVal[7] > 600);

  bool diamond =
  (!fullBlack &&
  sensorVal[2] > 600 &&
  sensorVal[3] > 600 &&
  sensorVal[4] > 600 &&
  sensorVal[5] > 600);

  bool intersection =
  (!fullBlack &&
  sensorVal[0] > 600 &&
  sensorVal[7] > 600);

  float useKp;
  float useKd;

  int baseSpeed;

  if(straight){

    useKp = Kp_straight;
    useKd = Kd_straight;

    baseSpeed = speed_straight;
  }

  else if(sharpTurn){

    useKp = Kp_turn;
    useKd = Kd_turn;

    baseSpeed = speed_turn;
  }

  else if(zigzag){

    useKp = Kp_turn;
    useKd = Kd_turn;

    baseSpeed = speed_normal;
  }

  else{

    useKp = Kp_normal;
    useKd = Kd_normal;

    baseSpeed = speed_normal;
  }

  if(fullBlack){

    useKp = Kp_intersection;
    useKd = Kd_intersection;

    baseSpeed = 230;

    target = 3500;
  }

  else if(diamond){

    useKp = Kp_diamond;
    useKd = Kd_diamond;

    baseSpeed = speed_diamond;

    if(lastDir < 0){

      target = 2500;
    }
    else if(lastDir > 0){

      target = 4500;
    }
  }

  else if(intersection){

    useKp = Kp_intersection;
    useKd = Kd_intersection;

    baseSpeed = speed_intersect;
  }

  error =
  position - target;

  derivative =
  error - lastError;

  if(error > 100)
    lastDir = 1;

  else if(error < -100)
    lastDir = -1;

  long correction =
  (useKp * error) +
  (useKd * derivative);

  lastError = error;

  int leftPWM =
  constrain(baseSpeed - correction,
            -255,255);

  int rightPWM =
  constrain(baseSpeed + correction,
            -255,255);

  if(sharpTurn){

    if(error > 0){

      leftPWM  = speed_turn;
      rightPWM = -speed_turn;
    }
    else{

      leftPWM  = -speed_turn;
      rightPWM = speed_turn;
    }
  }

  motor(leftPWM,rightPWM);
}

// Setup

void setup(){

  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);

  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);

  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);

  pinMode(STBY,OUTPUT);

  digitalWrite(STBY,HIGH);

  pinMode(BTN_CALI,INPUT_PULLUP);
  pinMode(BTN_RUN,INPUT_PULLUP);

  pinMode(BUZZER,OUTPUT);

  qtr.setTypeAnalog();

  qtr.setSensorPins(sensorPin,
                    SensorCount);

  beep(1);
}

// Loop

void loop(){

  if(!calibrated &&
     digitalRead(BTN_CALI) == LOW){

    delay(300);

    beep(1);

    delay(3000);

    for(int i=0;i<40;i++){

      for(int j=0;j<8;j++){

        whiteValues[j] +=
        analogRead(sensorPin[j]);
      }

      delay(50);
    }

    for(int j=0;j<8;j++){

      whiteValues[j] /= 40;
    }

    beep(2);

    delay(1000);

    longBeep(300);

    delay(3000);

    for(int i=0;i<40;i++){

      for(int j=0;j<8;j++){

        blackValues[j] +=
        analogRead(sensorPin[j]);
      }

      delay(50);
    }

    for(int j=0;j<8;j++){

      blackValues[j] /= 40;
    }

    longBeep(700);

    delay(700);

    beep(1);
    delay(120);

    beep(1);
    delay(120);

    longBeep(900);

    calibrated = true;
  }

  if(calibrated &&
     !running &&
     digitalRead(BTN_RUN) == LOW){

    delay(300);

    running = true;

    beep(3);
  }

  if(running){

    readSensor();

    if(lostLine()){

      if(lastDir >= 0){

        motor(-recoverReverse,
               recoverSpeed);
      }
      else{

        motor(recoverSpeed,
             -recoverReverse);
      }

      return;
    }

    lineFollow();
  }
}