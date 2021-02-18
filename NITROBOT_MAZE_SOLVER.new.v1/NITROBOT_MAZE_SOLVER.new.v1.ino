// The rangefinders work well to show the distance to objects from around
// 1 inch (2 cm) to around 9 feet away (3 meters), but they have trouble when
// they aren't approximatley at a right angle to the object they are detecting.
// If the angle is too great (over about 15 degrees) not enough of the sound
// bounces back for it to get a reliable range.

#include <Arduino.h>

// Тук е секцията за Библиотеки и хедър файлове които се инклудват:

#include <Servo.h>

// PROBEN TEKST 

// Тук е секцията с описание на използваните пинове:

// дефинициите са изцяло с главни букви
#define LEFT_FOR 9    // PWMB
#define LEFT_BACK 5   // DIRB  ---  Left
#define RIGHT_FOR 6   // PWMA
#define RIGHT_BACK 10 // DIRA  ---  Right

// константите започват с главна буква, а всяка следваща дума с главна буква
const int LeftIrAvoidancePin = 12;
const int RightIrAvoidancePin = A5;
const int UltrasonicPin = 3; 
const int RgbPin = 2;
// и тези пинове трябва да са константи - не се променят
const int ServoPin = 13;
const int LedPin = 33;

// Robot parameters:
const int RobotLenght = 24; // TODO Да се сложат реалните размери на робота !!!!
const int robotWidth = 16;  // TODO Да се сложат реалните размери на робота !!!!
// Maze parameters:
const int MazeCorridorWidth = RobotLenght * 1.5; // i.e. Corridor width //? ЗА СЕГА СЛАГАМЕ ШИРОЧИНАТА НА РОБОТА ДА Е 1.5 ОТ ДЪЛЖИНАТА НА РОБОТА 36sm.!!!!

// Tresholds:
const float FrontDistanceTreshold = MazeCorridorWidth - RobotLenght;
const float WallToCorridorMiddle = MazeCorridorWidth / 2;
const float SideCorridorTreshold = MazeCorridorWidth;

//? С теаи параметри правим нещо като ръчен PID алгоритъм за следване на централната линия спрямо измереното разстояние до страничната стена
const float CenterLineTolerance = 1.0; // plus/minus how many cm are acceptable to consider the movement to be on the center line...
                                       // +- 1 cm from centerline is considered straight movement!!!
const float SharpTurnTreshold = 8.0;   // TODO Да се определи спрямо размера робота и коридора !!!!!

//? От тук задаваме дали ще следваме дясна или лява стена за да е универсале алгоритъма
const int WallFollowingSide = -90; //Set: -90 for right wall following or +90 for left wall following
                                   //we will add this value to the servo position i.e. myservo.write(90 + WallFollowingSide);
                                   // in order to set to which side the servo should move (0 or 180 degrees)
//Servo parameters
const int FrontServoAngle = 90;
const int SideServoAngle = FrontServoAngle + WallFollowingSide; //(0 or 180 degrees)
const int FrontServoDelay = 150;
const int SideServoDelay = 150;

const int SpeedLeft = 100;
const int SpeedRight = 100 * 1.00; // corection


// Тук започват ГЛОБАЛНИТЕ променливи, които се използват в кода:
// променливите започват с малка буква, а всяка следваща дума с главна буква
// ....

// Тук инициализираме обектите:
Servo myservo;

// Тук слагаме прототипите на функциите:
void moveForward();
void turnRight();
void moveBackward();
void makeSlightLeftTurn();
void turnSlightRight();
void stopMoving();
float getDistance(int servoAngle, int delayAfterServoMovement); //read the Ultasonic Sensor pointing at the given servo angle
float MaxDistance = 80;
int speedLeft = SpeedLeft;
int speedRight = SpeedRight;
//-----------------------------------------------

void setup()
{
  pinMode(ServoPin, OUTPUT);
  pinMode(LEFT_FOR, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FOR, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  Serial.begin(9600);
  myservo.attach(ServoPin);
  myservo.write(90); //Move the servo to center position
  moveForward();
 //delay(10000); //test !!!!!!! Calibration
}

//---------------------------------------------------------

void loop()
{
  
  float frontDistance, sideDistance;
  // states:
  int currentState; //? Правим проверките универсални - за следване на стена отдясно или отляво в зависимост от избрания WallFollowingSide - параметър по-горе.
  // Състоянието (state) след прочитане на разстоянията може да бъде:
  // 1 Приближили сме стена отпред (<= FrontDistanceTreshold), отдясно/отляво има стена (< SideCorridorTreshold) - завой на 180 градуса
  // 2 Пррближили сме стена отпред (<= FrontDistanceTreshold), има коридор надясно/наляво (>= SideCorridorTreshold) - завой на 90 градуса надясно/наляво
  // 3 Напред стената е далече (> FrontDistanceTreshold), има коридор в дясно/ляво (>= SideCorridorTreshold) - завой на 90 градуса надясно/наляво
  // 4 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), в рамките на +- CenterLineTolerance от централната линия сме - движение право напред
  // 5 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от + SharpTurnTreshold от централната линия (по-близо до лявата/дясната стена) сме - движение напред с остър завой наляво/надясно
  // 6 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от - SharpTurnTreshold от централната линия сме (по-близо до дясната/лявата стена) сме - движение напред с остър завой надясно/наляво
  // 7 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от + tolerance от централната линия (по-близо до лявата/дясната стена) сме - движение напред със завой леко наляво/надясно
  // 8 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от - tolerance от централната линия сме (по-близо до дясната/лявата стена) сме - движение напред със завой леко надясно/наляво

  frontDistance = getDistance(FrontServoAngle, FrontServoDelay);
  sideDistance = getDistance(SideServoAngle, SideServoDelay);

  if (frontDistance <= FrontDistanceTreshold  ) //Стената отпред е близко
  {
    if (sideDistance < SideCorridorTreshold)
    {
      // 1 Приближили сме стена отпред (<= FrontDistanceTreshold), отдясно/отляво има стена (< SideCorridorTreshold)
      // - завой на 90 градуса
      currentState = 1; // turn 90 degrees
    }
    else if (sideDistance >= SideCorridorTreshold)
    {
      // 2 Пррближили сме стена отпред (<= FrontDistanceTreshold), има коридор надясно/наляво (>= SideCorridorTreshold)
      // - завой на 90 градуса надясно/наляво

      currentState = 2; // turn 90 degrees
    }
  }
  else if (frontDistance > FrontDistanceTreshold  ) //Стената отпред е далече
  {
    if (sideDistance >= SideCorridorTreshold)
    {
      // 3 Напред стената е далече (> FrontDistanceTreshold), има коридор в дясно/ляво (>= SideCorridorTreshold)
      // - завой на 90 градуса надясно/наляво

      currentState = 3; // turn 90 degrees
    }
    else if (sideDistance < SideCorridorTreshold) // В коридора сме!
    {
      if (sideDistance < WallToCorridorMiddle + CenterLineTolerance && sideDistance > WallToCorridorMiddle - CenterLineTolerance)
      {
        // 4 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), в рамките на +- CenterLineTolerance
        // от централната линия сме  - движение право напред

        currentState = 4; //Close to the centerline - go forwad
      }
      else if (sideDistance >= WallToCorridorMiddle + SharpTurnTreshold)
      {
        // 5 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от + SharpTurnTreshold от
        // централната линия (по-близо до лявата/дясната стена) сме - движение напред с остър завой наляво/надясно

        currentState = 5; //Close to the other wall - hard turn to centerline
      }
      else if (sideDistance <= WallToCorridorMiddle - SharpTurnTreshold)
      {
        // 6 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от - SharpTurnTreshold
        //  от централната линия сме (по-близо до дясната/лявата стена) сме - движение напред с остър завой наляво/надясно

        currentState = 6; //Close to the wall we are following - hard turn to centerline
      }
      else if (sideDistance < WallToCorridorMiddle + CenterLineTolerance && sideDistance < WallToCorridorMiddle + SharpTurnTreshold)
      {
        // 7 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от + tolerance
        //  от централната линия (по-близо до лявата/дясната стена) сме - движение напред със завой леко наляво/надясно

        currentState = 7; //Close to the wall we are following - hard turn to centerline
      }
    }
    else if (sideDistance > WallToCorridorMiddle - CenterLineTolerance && sideDistance > WallToCorridorMiddle - SharpTurnTreshold)
    {
      // 8 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от - tolerance
      //  от централната линия сме (по-близо до дясната/лявата стена) сме - движение напред със завой леко наляво/надясно

      currentState = 8; //Close to the wall we are following - hard turn to centerline
    }
     if (frontDistance > MaxDistance  && sideDistance > MaxDistance){
      currentState = 9; //FINAL
      }
  }
    switch (currentState)
    {
    case 1:
    stopMoving();
    moveBackward();
    delay(200);
    makeSlightLeftTurn();
    delay(500);
     /* завой на 180 градуса */
    Serial.print(1);
  
      break;
    case 2:
      /* завой на 90 градуса надясно/наляво */
      Serial.print(2);
       stopMoving();
    moveBackward();
    delay(200);
    turnSlightRight();
        delay(500);
      break;
    case 3:
        stopMoving();
    moveBackward();
    delay(200);
    turnSlightRight();
        delay(500);
      /* завой на 90 градуса надясно/наляво */
      break;
    case 4:
       speedLeft = SpeedLeft * 1.0;
       speedRight = SpeedRight * 1.0;
       moveForward();
      /* движение право напред */
      break;
    case 5:
      /* движение напред с остър завой наляво/надясно */
       speedLeft = SpeedLeft * 2.0;
       speedRight = SpeedRight * 1.0;
       moveForward();
      break;
    case 6:
      speedLeft = SpeedLeft *1.0;
       speedRight = SpeedRight *2.0;
       moveForward();
      /* движение напред с остър завой надясно/наляво */
      break;
    case 7:
      /* движение напред със завой леко наляво/надясно */
      break;
    case 8:
      /* движение напред със завой леко надясно/наляво */
      break;
    case 9:
    stopMoving();
    delay(20000);
    default:
      break;
    }

    //   while (distanceRight > treshold)
    // {
    //   digitalWrite(LedPin, HIGH);
    //   turnRight();
    //   distanceRight = readRightUltrasonic();
    // }

    // digitalWrite(LedPin, LOW);

    // while ((distanceFront < treshold / 2.0))
    // { //&& (distanceRight < treshold)
    //   makeSlightLeftTurn();
    //   distanceFront = readFrontUltrasonic();
    // }

    // // Тук сетваш стойностите, след като вече си изпълнил двата уайла, с някакви си предишни стойностти, които си прочел в началото, а и е минало време докато се
    // // Горе си добавил някаква корекция на скоростта int SpeedRight = 100 * 1.25; , която тук елиминираш като задаваш твърди стойности!!!!!!!!!!!!!

    // // ЗА ДА ИМАШ ЕФЕКТ ОТ ТЕЗИ ИФОВЕ, НАЙ ВЕРОЯТНО ТРЯБВА ДА СА ВЪТРЕ В УАЙЛА, В КОЙТО МЕРИШ ДИСТАНЦИЯТА В ДЯСНО.... ПРЕДИ ДА ЗАДАДЕШ ДВИЖЕНИЕТО.

    // if (distanceRight > 0 && distanceRight <= 10)
    // {
    //   SpeedRight = 135;
    //   SpeedLeft = 100;
    // }
    // else if (distanceRight > 10 && distanceRight <= 20)
    // {
    //   SpeedRight = 125;
    //   SpeedLeft = 100;
    // }
    // else
    // {
    //   SpeedRight = 125;
    //   SpeedLeft = 170;
    // }

    // if (distanceRightTemp == 0)
    // {
    //   makeSlightLeftTurn();
    //   delay(150); // тоест завиваш наляво за 150 милисекунди????
    // }

    // moveForward();
   
    stopMoving();
    moveForward();
}
//==================================== VOID =====================================================

void moveForward()
{
  analogWrite(LEFT_FOR, abs(speedLeft));
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, abs(speedRight));
  analogWrite(RIGHT_BACK, LOW);
}

void turnRight()
{
  analogWrite(LEFT_FOR, 255);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, 60);
}



void moveBackward()
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, abs(speedLeft));
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, abs(speedRight));
}

void makeSlightLeftTurn()
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, 150);
  analogWrite(RIGHT_FOR, 150);
  analogWrite(RIGHT_BACK, LOW);
}




void turnSlightRight()
{
  //myservo.write(0); // ЗАЩО ТУК?????????????????
  analogWrite(LEFT_FOR, 150);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, 150);
}

void stopMoving()
{
  analogWrite(LEFT_FOR, HIGH);
  analogWrite(LEFT_BACK, HIGH);
  analogWrite(RIGHT_FOR, HIGH);
  analogWrite(RIGHT_BACK, HIGH);
}

float getDistance(int servoAngle, int delayAfterServoMovement)
{
  float distance;
  myservo.write(servoAngle);
  delay(delayAfterServoMovement);
  pinMode(UltrasonicPin, OUTPUT);
  digitalWrite(UltrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicPin, LOW);
  pinMode(UltrasonicPin, INPUT);
  distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
  return distance;
}

// float readFrontUltrasonic()
// {
//   float distance;
//   myservo.write(90);
//   delay(150);
//   pinMode(UltrasonicPin, OUTPUT);
//   digitalWrite(UltrasonicPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(UltrasonicPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(UltrasonicPin, LOW);
//   pinMode(UltrasonicPin, INPUT);
//   distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
//   return distance;
// }

// float readRightUltrasonic()
// {
//   float distance;
//   myservo.write(0);
//   delay(150);
//   pinMode(UltrasonicPin, OUTPUT);
//   digitalWrite(UltrasonicPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(UltrasonicPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(UltrasonicPin, LOW);
//   pinMode(UltrasonicPin, INPUT);
//   distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
//   return distance;
// }

// float readDiagonalUltrasonic()
// {
//   float distance;
//   myservo.write(30);
//   delay(250);
//   pinMode(UltrasonicPin, OUTPUT);
//   digitalWrite(UltrasonicPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(UltrasonicPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(UltrasonicPin, LOW);
//   pinMode(UltrasonicPin, INPUT);
//   distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
//   return distance;
// }
