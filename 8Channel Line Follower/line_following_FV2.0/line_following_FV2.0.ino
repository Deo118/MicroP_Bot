//5.23/4/25/2026
/*********** PIN ***********/
const int sensorPin[8] = {A0,A1,A2,A3,A4,A5,A6,A7};

const int BUZZER = 13;

const int LED_PIN  = 9;
const int LED_PIN2 = 10;

const int PWMA = 5;
const int PWMB = 6;
const int AIN1 = 2;
const int AIN2 = 3;
const int BIN1 = 8;
const int BIN2 = 7;
const int STBY = 4;

const int BTN_CALI = 12;
const int BTN_RUN  = 11;

/*********** CONFIG ***********/

const unsigned long BOOST_TIME = 770;

unsigned long caliStartTime = 0;
#define CALI_AUTO 1

unsigned long finishTimer = 0;

#define FINISH_THRESHOLD 700
#define FINISH_HOLD_TIME 30  

unsigned long finishStartTime = 0;
#define FINISH_DURATION 2000 

unsigned long runStartTime = 0;
const unsigned long LEFT_PRIORITY_DELAY = 11000;

const unsigned long CALI_TIME = 5000;
unsigned long previousMillis = 0;

unsigned long runDelayTimer = 0;
const unsigned long RUN_DELAY = 100;

const unsigned long DIAMOND_TIME = 10000;

/*********** SENSOR ***********/
int sensorMin[8];
int sensorMax[8];
int sensorVal[8];

/*********** BOOL ***********/
bool runPending = false;
bool finishDetected = false;
bool finishConfirmed = false;
bool caliMode = false;
bool runMode  = false;
bool caliDone = false;
bool braking = false;
unsigned long brakeTimer = false;
bool boostActivated = false;
bool finishEnabled = false; 
bool leftPriorityMode = true;
bool buzzerActive = false;
bool debug = 0;
bool print = 0;

/*********** SPEED ***********/
int baseSpeed = 255;
int boostSpeed = 255;
int diamonSpeed = 255;
int slowSpeed = 255;
int biasLeftSpeed = 255;

/*********** PID ***********/
long position = 3500;
long lastError = 0;

float Kp_normal = 0.05; 
float Kd_normal = 4;

float Kp_turn = 10; 
float Kd_turn = 50.0; 

float Kp_boost = 0.005;
float Kd_boost = 1.5;

float Kp_diamond = 0.09;
float Kd_diamond = 7.5;

float Kp_left = 0.1;
float Kd_left = 8.5; 

// int preLaneSpeed = 100;
// int angleSpeed = 230;

/*********** LOST LINE ***********/
int lastDir = 0;
bool lostLine = false;
long sensorSum = 0;
#define LOST_LINE_THRESHOLD 1000

int currentLeftPWM = 0;
int currentRightPWM = 0;
#define BRAKE_STEP 10

/*********** STATE ***********/
enum RobotState {
  NORMAL,
};

RobotState robotState = NORMAL;

/*********** PROTOTYPE ***********/
void calibration();
void endCalibration();
void blinkLED();
void readSensor();
void checkLostLine();
void lineFollow();
void motor(int left, int right);

/*********** BUZZER ***********/

int melody[] = {660, 660, 0, 660, 0, 520, 660, 0, 780};
int noteDur[] = {100,100,100,100,100,100,100,100,200};

int noteIndex = 0;
unsigned long noteTimer = 0;
bool musicPlaying = false;

int buzzerStep = 0;
unsigned long buzzerTimer = 0;

const unsigned long BUZZ_ON  = 80;
const unsigned long BUZZ_OFF = 80;  

/*********** SETUP ***********/
void setup(){

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(BTN_CALI, INPUT_PULLUP);
  pinMode(BTN_RUN , INPUT_PULLUP);

  pinMode(BUZZER, OUTPUT);

  digitalWrite(BUZZER, LOW);
  digitalWrite(STBY, HIGH);

  for(int i=0;i<8;i++){
sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  Serial.println("READY");
}

/*********** LOOP ***********/
void loop(){

  if(print){
  Serial.print("pos=");
  Serial.print(position);
  Serial.print(" | L=");
  // Serial.print(leftPWM);
  // Serial.print(" R=");
  // Serial.print(rightPWM);
  Serial.print(" sensorSum:");
  Serial.print(sensorSum);
  Serial.print(" State:");
  Serial.print(robotState);
  Serial.print(" LastDir: ");
  Serial.print(lastDir);
  Serial.print(leftPriorityMode ? " LEFT ON" : " LEFT OFF");
  Serial.print(" Sensor: ");
  for(int i = 0; i < 8; i++){
  Serial.print(sensorVal[i]);
  Serial.print(" ");
  }
  Serial.println();
  }

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
    blinkLED();
    if(print){
      Serial.println("CALI");
    }
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

    runStartTime = millis();     
    leftPriorityMode = false;
  }
    if(runMode){
      readSensor();
      checkLostLine();
      checkFinish();
      updateBuzzer();

    if(!leftPriorityMode && millis() - runStartTime >= LEFT_PRIORITY_DELAY){
      leftPriorityMode = true;
      triggerBuzzer(); 
      finishEnabled = true; 
    }

  if(finishConfirmed){
  static bool braked = false;
  if(!braked){
    brakeHard();
    braked = true;
  }
  motor(0,0);

  if(millis() - finishStartTime <= FINISH_DURATION){
    playMario();
  } else {
    noTone(BUZZER);
  }

  return;
  }
    /*********** STATE ***********/
    switch(robotState){

      case NORMAL:
       if(lostLine){
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(LED_PIN2, HIGH);
      
        if(lastDir >= 0){
          motor(-slowSpeed, slowSpeed);
        } else {
          motor(slowSpeed, -slowSpeed);
        }
        return;
        }

        if(leftPriorityMode){
            baseSpeed = biasLeftSpeed;
        } else {
            baseSpeed = baseSpeed;
        }
        digitalWrite(LED_PIN, LOW);
        digitalWrite(LED_PIN2, LOW);

        lineFollow();
      break;
    }
  }
}

/*********** SENSOR ***********/
void readSensor(){

  long sum = 0;
  long weighted = 0;

  for(int i=0;i<8;i++){

    int v = analogRead(sensorPin[i]);

    v = constrain(v, sensorMin[i], sensorMax[i]);
    v = map(v, sensorMin[i], sensorMax[i], 0, 1000);

    // FILTER
    if(leftPriorityMode && v < 600){
      v = 0; // bỏ nhiễu
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

/*********** LOST LINE ***********/
void checkLostLine(){
  lostLine = (sensorSum < LOST_LINE_THRESHOLD);
}

/*********** PID ***********/
void lineFollow(){
  float Kp_use;
float Kd_use;
  if(braking){
  brakeHardNonBlocking();
  return; 
  }
// ===== LEFT PRIORITY =====

  if(leftPriorityMode){
    Kp_use = Kp_left;
    Kd_use = Kd_left;
  }
  else{
    if(!boostActivated && millis() - runStartTime > BOOST_TIME){
    boostActivated = true;
    braking = true;
    brakeTimer = millis();
    triggerBuzzer();
  }

  // ===== PID =====
  unsigned long runTime = millis() - runStartTime;
  if(millis() - runTime <= BOOST_TIME){
    baseSpeed = boostSpeed;
    Kp_use = Kp_boost;
    Kd_use = Kd_boost;
  }

  else if(runTime >= DIAMOND_TIME){

  baseSpeed = diamonSpeed;
  triggerBuzzer();
  Kp_use = Kp_diamond;
  Kd_use = Kd_diamond;
  }

  else{
    baseSpeed = 255;
    Kp_use = Kp_normal;
    Kd_use = Kd_normal;
  }
}

  // ===== CHECK NHÁNH =====
  bool left  = (sensorVal[0] > 700 || sensorVal[1] > 700 || sensorVal[2] > 700);
  bool right = (sensorVal[5] > 700 || sensorVal[6] > 700 || sensorVal[7] > 700);

  // ===== SETPOINT =====
  long target = 3500;

  if(leftPriorityMode){
    if(left){
      target = 1650;
      Serial.println("BIAS LEFT");
    }
    else if(right){
      target = 1650;
    }
  }

  // ===== PID =====
  long error = position - target;

  if(error > 100) lastDir = 1;
  else if(error < -100) lastDir = -1;

  long derivative = error - lastError; 
  long correction = Kp_use * error + Kd_use * derivative;

  lastError = error;

  int leftPWM  = constrain(baseSpeed - correction, 0, 255);
  int rightPWM = constrain(baseSpeed + correction, 0, 255);

  motor(leftPWM, rightPWM);
}

/*********** MOTOR ***********/
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
    digitalWrite(AIN1,1);
    digitalWrite(AIN2,0);
    analogWrite(PWMA,left);
  }
  else{
    digitalWrite(AIN1,0);
    digitalWrite(AIN2,1);
    analogWrite(PWMA,-left);
  }

  if(right >= 0){
    digitalWrite(BIN1,1);
    digitalWrite(BIN2,0);
    analogWrite(PWMB,right);
  }
  else{
    digitalWrite(BIN1,0);
    digitalWrite(BIN2,1);
    analogWrite(PWMB,-right);
  }
}

/*********** CALI ***********/
void calibration(){

  if(CALI_AUTO){
    motor(90,-90);
  }

  for(int i=0;i<8;i++){
    int v = analogRead(sensorPin[i]);

    if(v < sensorMin[i]) sensorMin[i] = v;
    if(v > sensorMax[i]) sensorMax[i] = v;
  }
}

void endCalibration(){

  caliMode = false;
  caliDone = true;

  motor(0,0);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PIN2, HIGH);
}

/*********** LED ***********/
void blinkLED(){

  static unsigned long t=0;

  if(millis()-t>300){
    t=millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
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
void checkFinish(){

  if(!finishEnabled) return;

  bool allOnLine = true;

  for(int i = 0; i < 8; i++){
    if(sensorVal[i] < FINISH_THRESHOLD){
      allOnLine = false;
      break;
    }
  }

  if(allOnLine){
    if(!finishDetected){
      finishDetected = true;
      finishTimer = millis();
    }
    else{
      if(millis() - finishTimer >= FINISH_HOLD_TIME){
        finishConfirmed = true;
        finishStartTime = millis();
      }
    }
  }
  else{
    finishDetected = false;
  }
}


void playMario(){

  if(!musicPlaying){
    musicPlaying = true;
    noteIndex = 0;
    noteTimer = millis();
  }

  if(noteIndex >= 9){
    digitalWrite(BUZZER, LOW);
    return;
  }

  if(millis() - noteTimer >= noteDur[noteIndex]){
    noteTimer = millis();

    if(melody[noteIndex] == 0){
      digitalWrite(BUZZER, LOW);
    } else {
      tone(BUZZER, melody[noteIndex]);
    }

    noteIndex++;
  }
}
void brakeHard(){

  motor(-200, -200); 
  delay(60);          

  motor(0,0);
}
void brakeHardNonBlocking(){

  if(millis() - brakeTimer < 1400){ 
    motor(255, 255);
    lastDir = -1;
    slowSpeed = 255;
  }
  else{
    lastDir = 0;
    slowSpeed = 255;
    // motor(0,0);
    braking = false;
  }
}

