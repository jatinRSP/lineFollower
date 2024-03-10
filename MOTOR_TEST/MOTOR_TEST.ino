#include <SparkFun_TB6612.h>

#define PWMA 5
#define AIN2 6
#define AIN1 7
#define STBY 8
#define BIN1 9
#define BIN2 10
#define PWMB 11

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {

}

void loop() {

  motor2.drive(255);
  motor1.drive(255);

  delay(3000);

  motor2.drive(-255);
  motor1.drive(-255);

  delay(3000);

}
