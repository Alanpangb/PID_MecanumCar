#include <Arduino.h>
#include <Wire.h>
#include <MeMCore.h>
/* [
41 31 51 21 11
42 32 52 22 12
45 35 55 25 15
43 33 53 23 13
44 34 54 24 14

]*/

constexpr byte motor1pin1 = 34; // HIGH-back
constexpr byte motor1pin2 = 35;
constexpr byte motor1pwm = 12;
constexpr byte motor2pin1 = 37;
constexpr byte motor2pin2 = 36;
constexpr byte motor2pwm = 8;
constexpr byte motor3pin1 = 43;
constexpr byte motor3pin2 = 42;
constexpr byte motor3pwm = 9;
constexpr byte motor4pin1 = 29; // HIGH-front
constexpr byte motor4pin2 = 39;
constexpr byte motor4pwm = 5;
uint16_t yMapped = 0;
uint8_t joystickZone = 0;
uint16_t ySpeed;
uint16_t xSpeed, MotorSpeed, PID_Motor;
uint16_t LeftMotor, RightMotor;
MeGyro gyro;
int targetangle = 0;
int angle_error;
float kp = 3.5, ki = 0.005;
int angleZ;
float correction;
int integral = 0;

constexpr uint16_t JOYSTICK_Y_UPPER_THRESHOLD = 950;
constexpr uint16_t JOYSTICK_Y_MID_UPPER_THRESHOLD = 520;
constexpr uint16_t JOYSTICK_Y_MID_LOWER_THRESHOLD = 460;
constexpr uint16_t JOYSTICK_Y_LOWER_THRESHOLD = 30;

constexpr uint16_t JOYSTICK_X_UPPER_THRESHOLD = 950;
constexpr uint16_t JOYSTICK_X_MID_UPPER_THRESHOLD = 520;
constexpr uint16_t JOYSTICK_X_MID_LOWER_THRESHOLD = 460;
constexpr uint16_t JOYSTICK_X_LOWER_THRESHOLD = 30;

void setup()
{
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pwm, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor2pwm, OUTPUT);
  pinMode(motor3pin1, OUTPUT);
  pinMode(motor3pin2, OUTPUT);
  pinMode(motor3pwm, OUTPUT);
  pinMode(motor4pin1, OUTPUT);
  pinMode(motor4pin2, OUTPUT);
  pinMode(motor4pwm, OUTPUT);
  Serial.begin(9600);
  gyro.begin();
}

void Foward(int M1_Speed, int M2_Speed, int M3_Speed, int M4_Speed)
{
  analogWrite(motor1pwm, M1_Speed);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(motor2pwm, M2_Speed);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(motor3pwm, M3_Speed);
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(motor4pwm, M4_Speed);
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void Backward(int motorspeed)
{
  analogWrite(motor1pwm, motorspeed);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(motor2pwm, motorspeed);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(motor3pwm, motorspeed);
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(motor4pwm, motorspeed);
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}

void Left(int motorspeed)
{
  analogWrite(motor1pwm, motorspeed);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(motor2pwm, motorspeed);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(motor3pwm, motorspeed);
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(motor4pwm, motorspeed);
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}

void Right(int motorspeed)
{
  analogWrite(motor1pwm, motorspeed);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(motor2pwm, motorspeed);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(motor3pwm, motorspeed);
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(motor4pwm, motorspeed);
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void LEFT_UP(int M1_Speed, int M3_Speed)
{
  analogWrite(motor1pwm, M1_Speed);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(motor2pwm, 0);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(motor3pwm, M3_Speed);
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(motor4pwm, 0);
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void RIGHT_UP(int M2_Speed, int M4_Speed)
{
  analogWrite(motor1pwm, 0);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(motor2pwm, M2_Speed);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(motor3pwm, 0);
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(motor4pwm, M4_Speed);
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void LEFT_DOWN(int M2_Speed, int M4_Speed)
{
  analogWrite(motor1pwm, 0);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(motor2pwm, M2_Speed);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(motor3pwm, 0);
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(motor4pwm, M4_Speed);
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}

void RIGHT_DOWN(int M1_Speed, int M3_Speed)
{
  analogWrite(motor1pwm, M1_Speed);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(motor2pwm, 0);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(motor3pwm, M3_Speed);
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(motor4pwm, 0);
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}

void RotateLeft(int M1_Speed, int M2_Speed, int M3_Speed, int M4_Speed)
{
  analogWrite(motor1pwm, M1_Speed);
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  analogWrite(motor2pwm, M2_Speed);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

  analogWrite(motor3pwm, M3_Speed);
  digitalWrite(motor3pin1, HIGH);
  digitalWrite(motor3pin2, LOW);

  analogWrite(motor4pwm, M4_Speed);
  digitalWrite(motor4pin1, HIGH);
  digitalWrite(motor4pin2, LOW);
}

void RotateRight(int M1_Speed, int M2_Speed, int M3_Speed, int M4_Speed)
{
  analogWrite(motor1pwm, M1_Speed);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  analogWrite(motor2pwm, M2_Speed);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);

  analogWrite(motor3pwm, M3_Speed);
  digitalWrite(motor3pin1, LOW);
  digitalWrite(motor3pin2, HIGH);

  analogWrite(motor4pwm, M4_Speed);
  digitalWrite(motor4pin1, LOW);
  digitalWrite(motor4pin2, HIGH);
}

void zoneswitchcase(uint16_t joystickX, uint16_t joystickY)
{
  switch (joystickZone)
  {
  case 51:
    Foward(225, 255, 255, 255);
    break;
  case 52:
    ySpeed = map(joystickY, JOYSTICK_Y_MID_LOWER_THRESHOLD, JOYSTICK_Y_LOWER_THRESHOLD, 0, 255);
    if(angle_error < targetangle - 3){
      LeftMotor = ySpeed + correction;
      RightMotor = ySpeed - correction;
    } else if(angle_error > targetangle + 3){
      LeftMotor = ySpeed - correction;
      RightMotor = ySpeed + correction;
    } else{
      LeftMotor = ySpeed;
      RightMotor = ySpeed;
    }
    if(LeftMotor < 50){
      LeftMotor = 75;

    }
    if(RightMotor < 50){
      RightMotor = 75;
      
    }
    Foward(LeftMotor, LeftMotor, RightMotor, RightMotor);
    break;
  case 53:
    ySpeed = map(joystickY, JOYSTICK_Y_MID_UPPER_THRESHOLD, JOYSTICK_Y_UPPER_THRESHOLD, 0, 255);
    Backward(ySpeed);
    break;
  case 54:
    Backward(225);
    break;
  case 35:
    xSpeed = map(joystickX, JOYSTICK_X_MID_UPPER_THRESHOLD, JOYSTICK_X_UPPER_THRESHOLD, 0, 255);
    Left(ySpeed);
    break;
  case 45:
    Left(255);
    break;
  case 25:
    xSpeed = map(joystickX, JOYSTICK_X_MID_LOWER_THRESHOLD, JOYSTICK_X_LOWER_THRESHOLD, 0, 255);
    Right(ySpeed);
    break;
  case 15:
    Right(225);
    break;
  case 41:
    LEFT_UP(225, 225);
    break;
  case 11:
    RIGHT_UP(225, 225);
    break;
  case 44:
    LEFT_DOWN(225, 225);
    break;
  case 14:
    RIGHT_DOWN(225, 225);
    break;
  case 22:
    xSpeed = map(joystickX, JOYSTICK_X_MID_LOWER_THRESHOLD, JOYSTICK_X_LOWER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_LOWER_THRESHOLD, JOYSTICK_Y_LOWER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      RIGHT_UP(xSpeed, MotorSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      RIGHT_UP(ySpeed, MotorSpeed);
    }
    break;
  case 32:
    xSpeed = map(joystickX, JOYSTICK_X_MID_UPPER_THRESHOLD, JOYSTICK_X_UPPER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_LOWER_THRESHOLD, JOYSTICK_Y_LOWER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      LEFT_UP(MotorSpeed, xSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      LEFT_UP(MotorSpeed, ySpeed);
    }
    break;
  case 33:
    xSpeed = map(joystickX, JOYSTICK_X_MID_UPPER_THRESHOLD, JOYSTICK_X_UPPER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_UPPER_THRESHOLD, JOYSTICK_Y_UPPER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      LEFT_DOWN(MotorSpeed, xSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      LEFT_DOWN(MotorSpeed, ySpeed);
    }
    break;
  case 23:
    xSpeed = map(joystickX, JOYSTICK_X_MID_LOWER_THRESHOLD, JOYSTICK_X_LOWER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_UPPER_THRESHOLD, JOYSTICK_Y_UPPER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      RIGHT_DOWN(xSpeed, MotorSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      RIGHT_DOWN(ySpeed, MotorSpeed);
    }
    break;
  case 31:
    xSpeed = map(joystickX, JOYSTICK_X_MID_UPPER_THRESHOLD, JOYSTICK_X_UPPER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_LOWER_THRESHOLD, JOYSTICK_Y_LOWER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      MotorSpeed += 50;
      LEFT_UP(MotorSpeed, xSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      MotorSpeed += 50;
      LEFT_UP(MotorSpeed, ySpeed);
    }
    break;
  case 42:
    xSpeed = map(joystickX, JOYSTICK_X_MID_UPPER_THRESHOLD, JOYSTICK_X_UPPER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_LOWER_THRESHOLD, JOYSTICK_Y_LOWER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      xSpeed += 50;
      LEFT_UP(MotorSpeed, xSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      ySpeed += 50;
      LEFT_UP(MotorSpeed, ySpeed);
    }
    break;
  case 21:
    xSpeed = map(joystickX, JOYSTICK_X_MID_LOWER_THRESHOLD, JOYSTICK_X_LOWER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_LOWER_THRESHOLD, JOYSTICK_Y_LOWER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      MotorSpeed += 50;
      RIGHT_UP(xSpeed, MotorSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      MotorSpeed += 50;
      RIGHT_UP(ySpeed, MotorSpeed);
    }
    break;
  case 12:
    xSpeed = map(joystickX, JOYSTICK_X_MID_LOWER_THRESHOLD, JOYSTICK_X_LOWER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_LOWER_THRESHOLD, JOYSTICK_Y_LOWER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      xSpeed += 50;
      RIGHT_UP(xSpeed, MotorSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      ySpeed += 50;
      RIGHT_UP(ySpeed, MotorSpeed);
    }
    break;
  case 34:
    xSpeed = map(joystickX, JOYSTICK_X_MID_UPPER_THRESHOLD, JOYSTICK_X_UPPER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_UPPER_THRESHOLD, JOYSTICK_Y_UPPER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      MotorSpeed += 50;
      LEFT_DOWN(MotorSpeed, xSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      MotorSpeed += 50;
      LEFT_DOWN(MotorSpeed, ySpeed);
    }
    break;
  case 43:
    xSpeed = map(joystickX, JOYSTICK_X_MID_UPPER_THRESHOLD, JOYSTICK_X_UPPER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_UPPER_THRESHOLD, JOYSTICK_Y_UPPER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      xSpeed += 50;
      LEFT_DOWN(MotorSpeed, xSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      ySpeed += 50;
      LEFT_DOWN(MotorSpeed, ySpeed);
    }
    break;
  case 24:
    xSpeed = map(joystickX, JOYSTICK_X_MID_LOWER_THRESHOLD, JOYSTICK_X_LOWER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_UPPER_THRESHOLD, JOYSTICK_Y_UPPER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      MotorSpeed += 50;
      RIGHT_DOWN(xSpeed, MotorSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      MotorSpeed += 50;
      RIGHT_DOWN(ySpeed, MotorSpeed);
    }
    break;
  case 13:
    xSpeed = map(joystickX, JOYSTICK_X_MID_LOWER_THRESHOLD, JOYSTICK_X_LOWER_THRESHOLD, 0, 255);
    ySpeed = map(joystickY, JOYSTICK_Y_MID_UPPER_THRESHOLD, JOYSTICK_Y_UPPER_THRESHOLD, 0, 255);
    if (xSpeed > ySpeed)
    {
      MotorSpeed = xSpeed - ySpeed;
      xSpeed += 50;
      RIGHT_DOWN(xSpeed, MotorSpeed);
    }
    else
    {
      MotorSpeed = ySpeed - xSpeed;
      ySpeed += 50;
      RIGHT_DOWN(ySpeed, MotorSpeed);
    }
    break;
  default:
    Foward(0, 0, 0, 0);
    break;
  }
}

void PID()
{
  angle_error = targetangle - angleZ;
  if (angle_error == 0){ // prevent the integral term from 'overshooting'
       integral = 0;
  } else{
       integral = integral + abs(angle_error);
  } 
  if (angle_error < targetangle - 3)
  {
     
    int temp = abs(angle_error);
    correction =  temp * kp + ki * integral;
    
    LeftMotor = 75 + correction;
    RightMotor = 25 + correction;
    if (LeftMotor > 255 )
    {
      LeftMotor = 255;
    } if (RightMotor > 255){
      RightMotor = 255;
    }
    RotateRight(LeftMotor, LeftMotor, RightMotor, RightMotor);
    // delay(10);
  }
  else if (angle_error > targetangle + 3)
  {

    int temp = angle_error;
    correction =  temp * kp + ki * integral;
 
    RightMotor = 75 + correction;
    LeftMotor = 25 + correction;
    if (LeftMotor > 255 )
    {
      LeftMotor = 255;
    } if (RightMotor > 255){
      RightMotor = 255;
    }
    RotateLeft(LeftMotor, LeftMotor, RightMotor, RightMotor);
  }
    // delay(10);
  // }else {
  //   LeftMotor = 0;
  //   RightMotor = 0;
  // }
}
void loop()
{
  uint16_t joystickX = analogRead(A6);
  uint16_t joystickY = analogRead(A5);

  angleZ = gyro.getAngleZ();
  gyro.update();

  // angle_error = targetangle - angleZ;
  // if(angle_error < targetangle - 1 ){

  //   int temp = abs(angle_error);
  //   MotorSpeed = 100 + temp * kp;

  //   RotateRight(MotorSpeed, MotorSpeed , MotorSpeed - 50 , MotorSpeed - 50);
  //   //delay(10);
  // } else if(angle_error > targetangle + 1 ){

  //   int temp = angle_error;
  //   MotorSpeed = 100 + temp * kp;

  //   RotateLeft(MotorSpeed, MotorSpeed , MotorSpeed - 50 , MotorSpeed - 50);
  //   Serial.print("Hi");
  //   //delay(10);
  // }

  if (joystickY < JOYSTICK_Y_LOWER_THRESHOLD /*30*/)
  {
    joystickZone += 1;
  }
  else if (joystickY < JOYSTICK_Y_MID_LOWER_THRESHOLD /*460*/)
  {
    joystickZone += 2;
  }
  else if (joystickY > JOYSTICK_Y_MID_UPPER_THRESHOLD /*520*/ && joystickY < JOYSTICK_Y_UPPER_THRESHOLD /*950*/)
  {
    joystickZone += 3;
  }
  else if (joystickY > JOYSTICK_Y_UPPER_THRESHOLD /*950*/)
  {
    joystickZone += 4;
  }
  else
  {
    joystickZone += 5;
  }

  if (joystickX < JOYSTICK_X_LOWER_THRESHOLD /*30*/)
  {
    joystickZone += 10;
  }
  else if (joystickX < JOYSTICK_X_MID_LOWER_THRESHOLD /*460*/)
  {
    joystickZone += 20;
  }
  else if (joystickX > JOYSTICK_X_MID_UPPER_THRESHOLD /*520*/ && joystickX < JOYSTICK_X_UPPER_THRESHOLD /*950*/)
  {
    joystickZone += 30;
  }
  else if (joystickX > JOYSTICK_X_UPPER_THRESHOLD /*950*/)
  {
    joystickZone += 40;
  }
  else
  {
    joystickZone += 50;
  }

  
  zoneswitchcase(joystickX, joystickY);
  PID();
  // Serial.print(joystickX);
  // Serial.print(" ");
  // Serial.print(joystickY);
  // Serial.print(" ");
  // Serial.print(angleZ);
  // Serial.print(" ");
  // Serial.println(joystickZone);
  Serial.print(angleZ);
  Serial.print(" ");
  Serial.print(angle_error);
  Serial.print(" ");
  Serial.println(correction);

  // Foward(100);
  // delay(1000);
  // Backward(100);
  // delay(1000);
  // Left(100);
  // Right(100);
  // delay(1000);
  joystickZone = 0;
}
