/*
  PlutoBUG_OS   |  Version: 1.0.0
  by plutoLABS  |           02/09/2023
  --------------------------------------
  FIRMWARE NOTES:
  - Code to be used on robot body assembly: [23003-000-000]
  - Co-Ordinate System: RHS --> BUG Front = Positive X-axis
  - ESP32-WROOM DEVKIT: 3.3V System
  - MG90S Servo System Configuration:
    Joint_A1 = D4  | Joint_A2 = D2
    Joint_B1 = D18 | Joint_B2 = D23
    Joint_C1 = D13 | Joint_C2 = D12
    Joint_D1 = D14 | Joint_D2 = D27
*/

#include <ESP32Servo.h>
#include "plutoBUG_OS-v1.0.0.h"
#include <Ps3Controller.h>

void setup()
{
  // Run PlutoBUG initialisation
  plutoBUGInit();

  // Run PS3 controller initialisation
  roboticControllerInit();

  // Run joint servo controller initialisation
  jointControllerInit();

  // Joint servo angles zeroise
  jointInit();

  delay(3000); // Safety Delay...
}

void loop()
{
  if(!Ps3.isConnected())
  {
    Serial.println("  *** ERROR: Robotic Controller System DISCONNECTED! ***");
    delay(1000);
    return;
  }

  roboticCommands();
}

void roboticControllerInit()
{
  // Initialise Robotic Controller System (RCS)
  Ps3.attach(roboticEvents);
  Ps3.attachOnConnect(controllerConnect);
  Ps3.begin("50:4C:55:54:4F:22"); // Define controller MAC address [hex --> P:L:U:T:O:22]
}

void controllerConnect()
{
  // On controller connect set flag
  Serial.println("   >>> Controller Connected Successfully! <<<");
  controller_isInit = true;
}

void plutoBUGInit()
{
  // PlutoBUG_OS setup serial
  Serial.begin(115200);
  
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
}

void jointControllerInit()
{
  // Attach all servos to control pins
  // Default: [500µs - 2500µs] --> Tune last number per servo
  joint_A1.setPeriodHertz(50);
  joint_A1.attach(servo_A1, 500, 2500);

  joint_A2.setPeriodHertz(50);
  joint_A2.attach(servo_A2, 500, 2500);

  joint_B1.setPeriodHertz(50);
  joint_B1.attach(servo_B1, 500, 2420);

  joint_B2.setPeriodHertz(50);
  joint_B2.attach(servo_B2, 500, 2400);

  joint_C1.setPeriodHertz(50);
  joint_C1.attach(servo_C1, 500, 2500);

  joint_C2.setPeriodHertz(50);
  joint_C2.attach(servo_C2, 500, 2400);

  joint_D1.setPeriodHertz(50);
  joint_D1.attach(servo_D1, 500, 2500);

  joint_D2.setPeriodHertz(50);
  joint_D2.attach(servo_D2, 500, 2500);
}

void jointInit()
{
  if (!joints_isInit)
  {
    // Initialise to zero position:
    joint_A1.write(zero_A1);
    joint_B1.write(zero_B1);
    joint_C1.write(zero_C1);
    joint_D1.write(zero_D1);

    joint_A2.write(zero_A2);
    joint_B2.write(zero_B2);
    joint_C2.write(zero_C2);
    joint_D2.write(zero_D2);
  }

  joints_isInit = true; // Joint servos initialised!
}

// Looped robotic commands from controller events
void roboticCommands()
{
  // Enable walking commands only when walking
  if(isWalking)
  {
    if(walkForward) forwardWalk();
    if(walkBack)    backWalk();
    if(pivotLeft)   leftPivot();
    if(pivotRight)  rightPivot();
    if(walkHalt)    moveJoints(crossStand);
  }
}

// Controller interrupt actions / events
void roboticEvents()
{
  //-----------------------------------------------------------------------
  // Define robotic events on controller input button interrupts
  //-----------------------------------------------------------------------

  // Button START Command
  //------------------------
  if(Ps3.event.button_down.start && !move_isInit)
  {
    move_isInit = true;
    isStanding = true;
    startupSequence();
  }

  // Button SELECT Command
  //------------------------
  if(Ps3.event.button_down.select && move_isInit)
  {
    isStanding = false;
    MAX_SPEED = 5.0;
    moveJoints(layAngle);
    MAX_SPEED = 2.0;
  }
  
  // Button TRIANGE Command
  //------------------------
  if(Ps3.event.button_down.triangle && move_isInit)
  {
    isStanding = true;
    moveJoints(crossStand);
  }

  // Button SQUARE Command
  //------------------------
  if(Ps3.event.button_down.square && move_isInit)
  {
    isStanding = true;
    waveSequence();
  }

  // Button CIRCLE Command
  //------------------------
  if(Ps3.event.button_down.circle && move_isInit)
  {
    isStanding = true;
    bowSequence();
  }

  // Button CROSS Command
  //------------------------
  if(Ps3.event.button_down.cross && move_isInit)
  {
    isStanding = false;
    moveJoints(crossSquat);
  }
  
  // Button UP Command
  //------------------------
  if(Ps3.event.button_down.up && move_isInit)
  {
    isStanding = true;
    moveJoints(frontStand);
  }

  // Button DOWN Command
  //------------------------
  if(Ps3.event.button_down.down && move_isInit)
  {
    isStanding = false;
    moveJoints(frontSquat);
  }

  // Button LEFT Command
  //------------------------
  if(Ps3.event.button_down.left && move_isInit)
  {
    isStanding = true;
    MAX_SPEED = 6.0;
    moveJoints(pushUp_UP);
    MAX_SPEED = 2.0;
  }

  // Button RIGHT Command
  //------------------------
  if(Ps3.event.button_down.right && move_isInit)
  {
    isStanding = false;
    MAX_SPEED = 4.0;
    moveJoints(pushUp_DOWN);
    MAX_SPEED = 2.0;
  }

  // Left SHOULDER Command
  //------------------------
  if(Ps3.event.button_down.l1 && move_isInit && isStanding)
  {    
    // Define walking commands
    isWalking = true;
    walkForward = false;
    walkBack = false;
    pivotRight = false;
    pivotLeft = true;
    walkHalt = false;     
  }

  // Right SHOULDER Command
  //------------------------
  if(Ps3.event.button_down.r1 && move_isInit && isStanding)
  {    
    // Define walking commands
    isWalking = true;
    walkForward = false;
    walkBack = false;
    pivotRight = true;
    pivotLeft = false;
    walkHalt = false;     
  }

  // Released L/R SHOULDER Command
  //------------------------
  if(Ps3.event.button_up.l1 || Ps3.event.button_up.r1 && move_isInit && isStanding)
  {    
    // Define walking commands
    isWalking = false;
    walkForward = false;
    walkBack = false;
    pivotRight = false;
    pivotLeft = false;
    walkHalt = true;     
  }

  // Analog Right-Stick Commands
  //-----------------------------
  if(move_isInit && isStanding)
  {
    if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 )
    {
      // Forward Stick: Walk Forward
      if(Ps3.data.analog.stick.ry <= -stickSenseMIN && move_isInit && isStanding)
      {
        // Define walking commands
        isWalking = true;
        walkForward = true;
        walkBack = false;
        pivotRight = false;
        pivotLeft = false;
        walkHalt = false;
      }

      // Back Stick: Walk Back
      if(Ps3.data.analog.stick.ry >= stickSenseMIN && move_isInit && isStanding)
      {
        // Define walking commands
        isWalking = true;
        walkForward = false;
        walkBack = true;
        pivotRight = false;
        pivotLeft = false;
        walkHalt = false;
      }
      
      // Centre Stick: Auto recentre back to standing once stick released
      if(Ps3.data.analog.stick.rx <= stickSenseMIN  && 
         Ps3.data.analog.stick.rx >= -stickSenseMIN && // Deadzone [-stickSenseMIN<= lx/ly <= stickSenseMIN]
         Ps3.data.analog.stick.ry <= stickSenseMIN  && 
         Ps3.data.analog.stick.ry >= -stickSenseMIN && move_isInit && isWalking) // Centre Stick
      {
        // Define walking commands
        isWalking = false;
        walkForward = false;
        walkBack = false;
        pivotRight = false;
        pivotLeft = false;
        walkHalt = true;
      }
    }
  }
}

void moveJoints(float targetAngle[4][2])
{
  bool targetNotReached;
  static unsigned long servo_time;
  do // Move until reach target joint angle
  {
    if ((millis() - servo_time) >= MAX_SPEED) // Non-blocking delay emulate servo speed
    {
      servo_time = millis(); // Save time reference for next update

      // Update ALL servo positions
      for (int leg = 0 ; leg < 4 ; leg++)         // Iterate through each leg
      {
        for (int joint = 0 ; joint < 2 ; joint++) // Iterate through each joint in leg
        {
          if (currentAngle[leg][joint] < targetAngle[leg][joint])
          {
            moveServo(leg, joint, currentAngle[leg][joint] + 1);
            currentAngle[leg][joint] = currentAngle[leg][joint] + 1;
          }
          else if (currentAngle[leg][joint] > targetAngle[leg][joint])
          {
            moveServo(leg, joint, currentAngle[leg][joint] - 1);
            currentAngle[leg][joint] = currentAngle[leg][joint] - 1;
          }
        }
      }
    }
    targetNotReached = memcmp((const void *)currentAngle, (const void *)targetAngle, sizeof(currentAngle)); // Compares the num memory bytes of two arrays [1 = unequal | 0 = equal]
  } while (targetNotReached);
}

void moveServo(int leg, int joint, int pos)
{
  switch(leg)
  {
    case 0: // Leg A
      switch(joint)
      {
        case 0: // Joint_A1
          joint_A1.write(pos);
          break;
        case 1: // Joint_A2
          joint_A2.write(pos);
          break;
      }
      break;
    case 1: // Leg B
      switch(joint)
      {
        case 0: // Joint_B1
          joint_B1.write(180-pos);
          break;
        case 1: // Joint_B2
          joint_B2.write(180-pos);
          break;
      }
      break;
    case 2: // Leg C
      switch(joint)
      {
        case 0: // Joint_C1
          joint_C1.write(180-pos);
          break;
        case 1: // Joint_C2
          joint_C2.write(180-pos);
          break;
      }
      break;
    case 3: // Leg D
      switch(joint)
      {
        case 0: // Joint_D1
          joint_D1.write(pos);
          break;
        case 1: // Joint_D2
          joint_D2.write(pos);
          break;
      }
      break;
  }
}

void startupSequence()
{
  int sequenceDelay = 100;
  moveJoints(frontStand);
  delay(500);
  moveJoints(startStand_A);
  delay(sequenceDelay);
  moveJoints(startStand_B);
  delay(sequenceDelay);
  moveJoints(startStand_C);
  delay(sequenceDelay);
  moveJoints(startStand_D);
  delay(sequenceDelay);
}

void forwardWalk()
{
  int walkDelay = 70;

  moveJoints(forwardWalk_A);
  delay(walkDelay);
  moveJoints(forwardWalk_B);
  delay(walkDelay);
  moveJoints(forwardWalk_C);
  delay(walkDelay);
  moveJoints(forwardWalk_D);
  delay(walkDelay);
}

void backWalk()
{
  int walkDelay = 70;

  moveJoints(backWalk_A);
  delay(walkDelay);
  moveJoints(backWalk_B);
  delay(walkDelay);
  moveJoints(backWalk_C);
  delay(walkDelay);
  moveJoints(backWalk_D);
  delay(walkDelay);
}

void leftPivot()
{
  int walkDelay = 50;

  if(!altLeft)
  {
    moveJoints(leftPivot_A1);
    delay(walkDelay);
    moveJoints(leftPivot_B1);
    delay(walkDelay);
    moveJoints(leftPivot_C1);
    delay(walkDelay);
    moveJoints(leftPivot_D1);
    delay(walkDelay);
  }
  
  if(altLeft)
  {
    moveJoints(leftPivot_A2);
    delay(walkDelay);
    moveJoints(leftPivot_B2);
    delay(walkDelay);
    moveJoints(leftPivot_C2);
    delay(walkDelay);
    moveJoints(leftPivot_D2);
    delay(walkDelay);
  }

  altLeft = !altLeft;
}

void rightPivot()
{
  int walkDelay = 50;

  if(!altRight)
  {
    moveJoints(rightPivot_A1);
    delay(walkDelay);
    moveJoints(rightPivot_B1);
    delay(walkDelay);
    moveJoints(rightPivot_C1);
    delay(walkDelay);
    moveJoints(rightPivot_D1);
    delay(walkDelay);
  }

  if(altRight)
  {
    moveJoints(rightPivot_A2);
    delay(walkDelay);
    moveJoints(rightPivot_B2);
    delay(walkDelay);
    moveJoints(rightPivot_C2);
    delay(walkDelay);
    moveJoints(rightPivot_D2);
    delay(walkDelay);
  }

  altRight = !altRight;
}

void waveSequence()
{
  int waveDelay = 10;
  int waveTimes = 4;
  
  moveJoints(crossStand);
  delay(waveDelay);
  moveJoints(waveUp);
  
  for(int i = 0; i <= waveTimes; i++)
  {
    moveJoints(waveLeft);
    delay(waveDelay);
    moveJoints(waveRight);
    delay(waveDelay);
  }
  
  moveJoints(crossStand);
}

void bowSequence()
{
  moveJoints(bowUp);
  delay(500);
  moveJoints(bowDown);
  delay(1000);
  moveJoints(bowUp);
  delay(600);
  moveJoints(crossStand);  
}
