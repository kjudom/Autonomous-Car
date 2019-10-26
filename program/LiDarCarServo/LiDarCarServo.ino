/* This robot automatically avoid the obstracles and find the direction by using
  a LiDAR sensor sweeping in front of the robot. The speed of the wheels is
  controled by the PWM signal using the analogWrite() function.
  
  Author: Udom
  Date: 9 Oct 2019

  *** you may need to disconnect the motor control pins before uploading the sketch ***
*/

#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>

#define stepAng  18       // step angle
#define numStep 10        // = 180/stepAng 

#define servoPin 2        // GIO2 = D4

// motors control pins
#define LDir 14  //D5 = GPIO14
#define LSpd 12  //D6 = GPIO12
#define RSpd 13  //D7 = GPIO13
#define RDir 15  //D8 = GPIO15

// sampling interval for the motor control @80MHz
#define CtrlIntv  4000000    // this gives 0.05 sec or 50ms

#define TurnDelay 300       // turn for 300ms
#define MinDistance 100     // 100mm

Servo myservo;  		// create servo object to control a servo

VL53L1X sensor;

int pos = 0;          // servo position
int dir = 1;          // servo moving direction: +1/-1
int val;              // LiDAR measured value

float distances[numStep+1]; // array for the distance at each angle, including at 0 and 180

float leftSum, rightSum, frontSum;

volatile unsigned long next;

//=======================================================================
// software timer interrupt: counts clock cycles at 80MHz
void inline motorCtrl_ISR(void){
  if (frontSum < MinDistance)
  {
    if (leftSum < rightSum)
      turnRight();     
    else
      turnLeft();
  }          
  
  int lsp = (int)(1000*rightSum/(leftSum+rightSum));
  int rsp = (int)(1000*leftSum/(leftSum+rightSum));
  
  LeftMove(lsp, true);
  RightMove(rsp, true); 

  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
}

void setup() {
  Serial.begin(115200);
  myservo.attach(servoPin);  
    
  pinMode(LSpd, OUTPUT);
  pinMode(LDir, OUTPUT);
  pinMode(RSpd, OUTPUT);
  pinMode(RDir, OUTPUT);  
  motorsStop();

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  //Initialize the timer
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(motorCtrl_ISR);
  next=ESP.getCycleCount()+CtrlIntv;
  timer0_write(next);
  interrupts();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(35000); //35ms
  sensor.startContinuous(35);   
}

void LeftMove(int speed, bool dir)
{
  digitalWrite(LDir, dir); 
  if (dir)
    analogWrite(LSpd, 1023-speed);  
  else
    analogWrite(LSpd, speed);
}

void RightMove(int speed, bool dir)
{
  digitalWrite(RDir, dir); 
  if (dir)
    analogWrite(RSpd, 1023-speed);
  else
    analogWrite(RSpd, speed);
}

void turnRight()
{
  noInterrupts();

  digitalWrite(LDir, true);    
  analogWrite(LSpd, 256);     // duty cycle 25%
  digitalWrite(RDir, false);    
  analogWrite(RSpd, 768);     // duty cycle 75%  
  
  delay(TurnDelay);
  interrupts();
}

void turnLeft()
{
  noInterrupts();

  digitalWrite(LDir, false);    
  analogWrite(LSpd, 768);     // duty cycle 75%
  digitalWrite(RDir, true);    
  analogWrite(RSpd, 256);     // duty cycle 25%  
  
  delay(TurnDelay);
  interrupts();
}

void loop() { 
  pos += dir;
  myservo.write(pos*stepAng);
    
  val = sensor.read();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  distances[pos] = 0.3*distances[pos] + 0.7*val;
  
  if (pos == numStep)
  {
    dir = -1;
  }
  else if (pos == 0)
  {
    dir = 1;
  }

  // find the left and right average sum
  if (pos > (numStep/2))
    leftSum = 0.3*leftSum + 1.4*distances[pos]/numStep; 
  else if (pos < (numStep/2))
    rightSum = 0.3*rightSum + 1.4*distances[pos]/numStep;

  // find the front average sum
  if ((pos > (numStep/4)) && (pos < (numStep*3/4)))
    frontSum = 0.3*frontSum + 1.4*distances[pos]/numStep;  
}
