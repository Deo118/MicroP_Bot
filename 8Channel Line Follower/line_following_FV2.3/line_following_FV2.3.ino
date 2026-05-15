const int sensorPin[8] = {A0,A1,A2,A3,A4,A5,A6,A7};

const int BUZZER = 13;

const int PWMA = 5;
const int PWMB = 6;
const int AIN1 = 2;
const int AIN2 = 3;
const int BIN1 = 8;
const int BIN2 = 7;
const int STBY = 4;

const int BTN_CALI = 12;
const int BTN_RUN  = 11;

const unsigned long CALI_TIME = 5000;
const unsigned long RUN_DELAY = 1000;

unsigned long caliStartTime = 0;
unsigned long runDelayTimer = 0;

#define CALI_AUTO 1

int sensorMin[8];
int sensorMax[8];
int sensorVal[8];

bool runPending = false;
bool caliMode = false;
bool runMode = false;
bool caliDone = false;

bool buzzerActive = false;
bool debug = false;

int baseSpeed = 250;

long position = 3500;
long lastError = 0;

float Kp = 0.06;
float Kd = 2.0;

float filteredDerivative = 0;

int lastDir = 0;
bool lostLine = false;

long sensorSum = 0;

#define LOST_LINE_THRESHOLD 500

enum RobotState {
  NORMAL,
};

RobotState robotState = NORMAL;

void calibration();
void endCalibration();
void readSensor();
void checkLostLine();
void lineFollow();
void motor(int left, int right);
void triggerBuzzer();
void updateBuzzer();

int buzzerStep = 0;
unsigned long buzzerTimer = 0;

const unsigned long BUZZ_ON = 80;
const unsigned long BUZZ_OFF = 80;

void setup(){

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);

  pinMode(BTN_CALI, INPUT_PULLUP);
  pinMode(BTN_RUN, INPUT_PULLUP);

  pinMode(BUZZER, OUTPUT);

  digitalWrite(BUZZER, LOW);
  digitalWrite(STBY, HIGH);

  for(int i = 0; i < 8; i++){

    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
}

void loop(){

  if(!runMode && digitalRead(BTN_CALI) == LOW){

    delay(200);

    if(!caliMode){

      caliMode = true;
      caliDone = false;

      caliStartTime = millis();
    }

    while(digitalRead(BTN_CALI) == LOW);
  }

  if(caliMode){

    calibration();

    if(CALI_AUTO && millis() - caliStartTime >= CALI_TIME){

      endCalibration();
    }
  }

  if(caliDone && !runMode && !runPending && digitalRead(BTN_RUN) == LOW){

    delay(200);

    runPending = true;

    runDelayTimer = millis();

    while(digitalRead(BTN_RUN) == LOW);
  }

  if(runPending && millis() - runDelayTimer >= RUN_DELAY){

    runPending = false;
    runMode = true;

    lostLine = false;

    robotState = NORMAL;
  }

  if(runMode){

    readSensor();

    checkLostLine();

    updateBuzzer();

    switch(robotState){

      case NORMAL:

        if(lostLine){

          if(lastDir >= 0){

            motor(-180, 180);
          }
          else{

            motor(180, -180);
          }

          return;
        }

        lineFollow();

      break;
    }
  }
}

void readSensor(){

  long sum = 0;
  long weighted = 0;

  for(int i = 0; i < 8; i++){

    int v = analogRead(sensorPin[i]);

    v = constrain(v, sensorMin[i], sensorMax[i]);

    v = map(v, sensorMin[i], sensorMax[i], 0, 1000);

    if(v < 50){
      v = 0;
    }

    sensorVal[i] = v;

    sum += v;

    weighted += (long)v * (i * 1000);
  }

  sensorSum = sum;

  if(sum > 0){

    position = weighted / sum;
  }
}

void checkLostLine(){

  lostLine = (sensorSum < LOST_LINE_THRESHOLD);
}

void lineFollow(){

  long target = 3500;

  long error = position - target;

  /************ DEAD BAND ************/

  if(abs(error) < 25){

    error = 0;
  }

  if(error > 100){

    lastDir = 1;
  }
  else if(error < -100){

    lastDir = -1;
  }

  long derivative = error - lastError;

  /************ DERIVATIVE FILTER ************/

  filteredDerivative =
    (filteredDerivative * 0.7) +
    (derivative * 0.3);

  long correction =
    (Kp * error) +
    (Kd * filteredDerivative);

  correction = constrain(correction, -255, 255);

  lastError = error;

  int leftPWM =
    constrain(baseSpeed - correction, -255, 255);

  int rightPWM =
    constrain(baseSpeed + correction, -255, 255);

  motor(leftPWM, rightPWM);
}

void motor(int left, int right){

  if(debug){

    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    return;
  }

  if(left >= 0){

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    analogWrite(PWMA, left);
  }
  else{

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    analogWrite(PWMA, -left);
  }

  if(right >= 0){

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    analogWrite(PWMB, right);
  }
  else{

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    analogWrite(PWMB, -right);
  }
}

void calibration(){

  if(CALI_AUTO){

    motor(90, -90);
  }

  for(int i = 0; i < 8; i++){

    int v = analogRead(sensorPin[i]);

    if(v < sensorMin[i]) sensorMin[i] = v;

    if(v > sensorMax[i]) sensorMax[i] = v;
  }
}

void endCalibration(){

  caliMode = false;

  caliDone = true;

  motor(0,0);
}

void triggerBuzzer(){

  buzzerActive = true;

  buzzerStep = 0;

  buzzerTimer = millis();
}

void updateBuzzer(){

  if(!buzzerActive) return;

  unsigned long now = millis();

  switch(buzzerStep){

    case 0:

      digitalWrite(BUZZER, HIGH);

      if(now - buzzerTimer >= BUZZ_ON){

        buzzerTimer = now;

        buzzerStep++;
      }

    break;

    case 1:

      digitalWrite(BUZZER, LOW);

      if(now - buzzerTimer >= BUZZ_OFF){

        buzzerTimer = now;

        buzzerStep++;
      }

    break;

    case 2:

      digitalWrite(BUZZER, HIGH);

      if(now - buzzerTimer >= BUZZ_ON){

        buzzerTimer = now;

        buzzerStep++;
      }

    break;

    case 3:

      digitalWrite(BUZZER, LOW);

      buzzerActive = false;

    break;
  }
}
