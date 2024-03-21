#include <SoftwareSerial.h>

//placeholder functions to remove intellisense errors
void stop();
void reverse();
void forward();
void cw();
void ccw();
void strafe_left();
float millis();
float abs(float input);
void fast_flash_double_LED_builtin();
void straight_align(float input1, float input2, float input3);

float sonarThreshold = 2;
float yawThreshold = 5;
float speed_val;

// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Corner Finder
void FindCorner(float sonarDistance, float shortIR, float yawAngle){
  static int runState = 0;
  static int prevRunState = 0;

  float timeExitInitial = 0;
  
  float time45deg = 1500.0 * (100.0/(float)speed_val);

  switch (runState){
    //Reverse to wall
    case 0:
      static int numStationary = 0;
      static int  prevSonar = 0;
      reverse();

      //keep count of how often sonar reading doesn't change
      if (abs(sonarDistance - prevSonar) < sonarThreshold)
        numStationary++;

      prevSonar = sonarDistance;
      
      //if sonar detects stationary enough times (~2s), move to next state
      if(numStationary > 200){
        stop();

        if(prevRunState == 4)
          runState = 5;
        else
          runState++;

        prevRunState = 0;
      }

      break;

    //Detect which wall
    case 1:
      if (sonarDistance > 135)
        runState++;
      else
        runState+=2;
      
      break;

    //Short walls
    case 2:
      strafe_left();

      if (shortIR < 15){
        stop();
        prevRunState = runState;
        runState = 5;
      }
      break;

    //Long walls pt. 1 (move forwards)
    case 3:
      static int sonarInit = sonarDistance;
      
      forward();

      if ((sonarInit - sonarDistance) > 15){
        stop();
        prevRunState = runState;
        runState++;
      }
      break;

    //Long walls pt. 2 (rotate CCW)
    case 4:
      static int timeInitial = millis();
      int timeElapsed = millis() - timeInitial;

      static bool withinYawThresh = 0;

      if (timeElapsed >= time45deg){
        straight_align(shortIR, yawAngle, 15);
      } else
        ccw();

      //Determine exit condition for straight_alight controller
      if (yawAngle < yawThreshold){
        //if yaw angle has been within threshold for 1.5s, finish case
        if(withinYawThresh && ((millis() - timeExitInitial) > 150)){
          stop();
          runState = 0;
          prevRunState = 4;          
        }
        
        //if prev loop was not within threshold & current loop is, start a timer
        if (!withinYawThresh){
          timeExitInitial = millis();
          withinYawThresh = 1;
        }
        //store whether this loop was within the yaw threshold in a bool
      } else
        withinYawThresh = 0;
      break;

    //controller finished
    case 5:
      fast_flash_double_LED_builtin();
      break;
  };

  //debugging
  BluetoothSerial.print("State: ");
  BluetoothSerial.println(runState, DEC);

  BluetoothSerial.print("Ultrasonic: ");
  BluetoothSerial.print(sonarDistance, DEC);
  BluetoothSerial.println(" cm");

  BluetoothSerial.print("IR: ");
  BluetoothSerial.print(shortIR, DEC);
  BluetoothSerial.println(" cm");

  BluetoothSerial.print("Yaw: ");
  BluetoothSerial.print(yawAngle, DEC);
  BluetoothSerial.println(" deg");

  BluetoothSerial.println(" ");

}

//redundant code to align the corner directly ahead
/*
    //Scan for corner distance
    case 0:
      static int timeElapsed = 0;

      if (timeElapsed >= time90deg){
        stop();
        runState++;
        break;
      }

      cw();

      if (maxSonar < sonarDistance)
        maxSonar = sonarDistance;

      timeElapsed += T;
      break;
    
    //Align with corner
    case 1:
      if (sonarDistance >= (maxSonar - sonarTolerance)) {
        stop();
        runState++;
        break;
      }
      break;
*/