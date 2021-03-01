#include <NewPing.h>

#define TRIG A0
#define ECHO A1
#define MAX_DIST 200

#define TRIG_L A4
#define ECHO_L A5
#define MAX_DIST_L 200

#define TRIG_R A2
#define ECHO_R A3
#define MAX_DIST_R 200

#define RIGHT_PWM 9
#define RIGHT_FOR 5
#define RIGHT_BACK 4
#define LEFT_FOR 8
#define LEFT_BACK 7
#define LEFT_PWM 10
#define STBY 6

unsigned int frontDistance;
unsigned int leftDistance;
unsigned int rightDistance;

const int LeftSpeed = 80;
const int RightSpeed = 80;

int speedLeft = LeftSpeed;
int speedRight = RightSpeed;

NewPing sonar(TRIG, ECHO, MAX_DIST);
NewPing sonar_L(TRIG_L, ECHO_L, MAX_DIST_L);
NewPing sonar_R(TRIG_R, ECHO_R, MAX_DIST_R);

void moveForward();
void moveBackward();
void customTurnLeft();
void customTurnRight();
void stopMoving();

void setup()
{
  pinMode(STBY, OUTPUT);
  pinMode(RIGHT_FOR, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_FOR, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);

  digitalWrite(STBY, HIGH);
  unsigned int frontDistance = sonar.ping_cm();
  unsigned int leftDistance = sonar_L.ping_cm();
  unsigned int rightDistance = sonar_R.ping_cm();
  Serial.begin(9600);
  digitalWrite(RIGHT_PWM, 80);
  digitalWrite(LEFT_PWM, 80);
  moveForward();
  delay(5000);
}

/*-------------------------------------------------*/

void loop()

{
  int currentState = 0;
  
  rightDistance = sonar_R.ping_cm(); //read right Ultrasonic sensor
  if (rightDistance == 0)
    rightDistance = 255;

  frontDistance = sonar.ping_cm(); //read front Ultrasonic sensor
  Serial.println(frontDistance);
  if (frontDistance == 0)
    frontDistance = 255;

//  leftDistance = sonar_L.ping_cm();  //read left Ultrasonic sensor 
//  Serial.println(leftDistance);
//  if (leftDistance == 0)
//    leftDistance = 255;

  if (frontDistance <= 15.0) //the front wall is neal, turn left 90 degrees
  {
    currentState = 1; 
  }
  
  if (frontDistance >= 15.0) //the front wall is far away
  {
    if (rightDistance >= 30.0) //the right wall is far away
    {
      currentState = 3; // turn right 90 degrees
    }
    else if (rightDistance > 11.0 && rightDistance < 19.0) // the robot is in the centerline - go forwad
    {
      currentState = 4;
    }   
    else if (rightDistance < 24.0 && rightDistance >= 19.0) // sharp turn right
    {
      currentState = 5; 
    }
    
    else if (rightDistance > 6.0 && rightDistance <= 11.0) 
    {
      currentState = 6; // Лек завой наляво
    }
    
    else if (rightDistance <= 6.0)
    {
      Serial.println("SET case = 7 ");
      currentState = 7; //Силнен завой наляво
    }
    else if (rightDistance >= 24.0 && rightDistance < 30.0)
    {
      Serial.println("SET case = 8 ");
      currentState = 8;  //Силнен завой надясно
    }
    
     }
  switch (currentState)
  {
  case 1: /* завой на 90 градуса */
    Serial.println("ЛЯВ ЗАВОЙ");
    moveBackward();
    delay(100);
    speedLeft = LeftSpeed * 1.35;
    speedRight = RightSpeed* 1.35;
    customTurnLeft();
    delay(750);
    moveBackward();
    delay(100);
  case 3: /* завой на 90 градуса надясно/наляво */
    Serial.println("ДЕСЕН ЗАВОЙ");
    //moveBackward();
    //delay(100);
    for (size_t i = 0; i < 8; i++)
    {
      speedLeft = LeftSpeed;
      speedRight = RightSpeed;
      moveForward();
      delay(20);
      speedLeft = LeftSpeed * 1.4;
      speedRight = 0;
      moveForward();
      delay(170);
    }
    for (size_t i = 0; i < 7; i++)
    {
      speedLeft = LeftSpeed;
      speedRight = RightSpeed;
      moveForward();
      delay(35);
      stopMoving();
      delay(20);
    }
    moveBackward();
    delay(100);
    //    directionCompensation = false;
    break;
  case 4:
    Serial.println("Движение НАПРАВО");
    break;
  case 5:
    Serial.println("Лек завой надясно");
    speedRight = RightSpeed * 2.55;
    speedLeft = LeftSpeed * 2.55;
    customTurnRight();
    delay(50);
    break;
  case 6:
    Serial.println("Лек завой наляво");
    speedRight = RightSpeed * 2.55;
    speedLeft = LeftSpeed * 2.55;
    customTurnLeft();
    delay(50);
    break;
  case 7:
    Serial.println("Силнен завой наляво");
    speedRight = RightSpeed * 2.55;
    speedLeft = LeftSpeed * 2.55;
    customTurnLeft();
    delay(100);
    break;
  case 8:
    Serial.println("Силнен завой надясно");
    speedRight = RightSpeed * 2.55;
    speedLeft = LeftSpeed * 2.55;
    customTurnRight();
    delay(100);
    break;
  default:
    break;
  }
}
void moveForward()
{
  digitalWrite(LEFT_FOR, HIGH);
  digitalWrite(LEFT_BACK, LOW);
  digitalWrite(RIGHT_FOR, HIGH);
  digitalWrite(RIGHT_BACK, LOW);
  analogWrite(RIGHT_PWM, speedRight);
  analogWrite(LEFT_PWM, speedLeft);
}

void moveBackward()
{
  digitalWrite(LEFT_FOR, LOW);
  digitalWrite(LEFT_BACK, HIGH);
  digitalWrite(RIGHT_FOR, LOW);
  digitalWrite(RIGHT_BACK, HIGH);
  analogWrite(RIGHT_PWM, speedRight);
  analogWrite(LEFT_PWM, speedLeft);
}

void customTurnLeft()
{
  digitalWrite(LEFT_FOR, LOW);
  digitalWrite(LEFT_BACK, HIGH);
  digitalWrite(RIGHT_FOR, HIGH);
  digitalWrite(RIGHT_BACK, LOW);
  analogWrite(RIGHT_PWM, speedRight);
  analogWrite(LEFT_PWM, speedLeft);
}

void customTurnRight()
{
  digitalWrite(LEFT_FOR, HIGH);
  digitalWrite(LEFT_BACK, LOW);
  digitalWrite(RIGHT_FOR, LOW);
  digitalWrite(RIGHT_BACK, HIGH);
  analogWrite(RIGHT_PWM, speedRight);
  analogWrite(LEFT_PWM, speedLeft);
}

void stopMoving()
{
  digitalWrite(LEFT_FOR, LOW);
  digitalWrite(LEFT_BACK, LOW);
  digitalWrite(RIGHT_FOR, LOW);
  digitalWrite(RIGHT_BACK, LOW);
}
