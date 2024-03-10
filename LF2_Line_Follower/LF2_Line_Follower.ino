#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
//------------------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = 1;             //keep 1 in case of black line. In case of white line change this to 0
unsigned int lineThickness = 15;  //Enter line thickness in mm. Works best for thickness between 10 & 35
unsigned int numSensors = 5;      // Enter number of sensors as 5 or 7
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 150;
int currentSpeed = 30;
int sensorWeight[7] = { 4, 2, 1, 0, -1, -2, -4 };
int activeSensors;
float Kp = 0.08;
float Kd = 0.15;
float Ki = 0;

int onLine = 1;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorArray[7];

void setup() {

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(11, INPUT_PULLUP);  //Pushbutton
  pinMode(12, INPUT_PULLUP);  //Pushbutton
  pinMode(13, OUTPUT);        //LED

  pinMode(5, OUTPUT);     //standby for older carrier boards
  digitalWrite(5, HIGH);  //enables the motor driver
                          //in case of a newer board with motor enable jumper, you can delete these two lines which frees up digital pin 5


  lineThickness = constrain(lineThickness, 10, 35);
  if (numSensors == 5) {
    sensorWeight[1] = 4;
    sensorWeight[5] = -4;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (1) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  //PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
      if (error > 0) {
        motor1run(-50);
        motor2run(lfSpeed);
      } else {
        motor1run(lfSpeed);
        motor2run(-50);
      }
    }
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  if (numSensors == 7) {
    for (int i = 0; i < 7; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }
  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
      activeSensors += sensorArray[i];
    }
    error = error / activeSensors;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 10000; i++) {
    motor1run(50);
    motor2run(-50);

    for (int i = 0; i < 7; i++) {
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for (int i = 0; i < 7; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;
  if (numSensors == 7) {
    for (int i = 0; i < 7; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;
      if (sensorArray[i]) onLine = 1;

      if (isBlackLine == 1 && sensorArray[i]) onLine = 1;
      if (isBlackLine == 0 && !sensorValue[i]) onLine = 1;
    }
  }
  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      if (isBlackLine) {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
      } else {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
      }
      sensorValue[i] = constrain(sensorValue[i], 0, 1000);
      sensorArray[i] = sensorValue[i] > 500;
      if (sensorArray[i]) onLine = 1;

      if (isBlackLine == 1 && sensorArray[i]) onLine = 1;
      if (isBlackLine == 0 && !sensorValue[i]) onLine = 1;
    }
  }
}
//--------Function to run Motor 1-----------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//--------Function to run Motor 2-----------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}
