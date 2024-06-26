/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include <Servo.h>  //Need for Servo pulse output

#define NO_READ_GYRO  //Uncomment of GYRO is not attached.   
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Inverse motor control pins
const byte left_front = 51;
const byte left_rear = 50;
const byte right_rear = 47;
const byte right_front = 46;

//Default motor control pins
/*
const byte left_front = 47;
const byte left_rear = 46;
const byte right_rear = 51;
const byte right_front = 50;
*/

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

int speed_val = 100;
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;

////////////////////// IR SENSOR PINS and Variables//////////////
const int irsensorSF = A10; //Short Front sensor is attached on pinA0
const int irsensorSB = A11; //Short Side sensor is attached on 
const int irsensorLF = A9; //Long Front sensor is attached on 
const int irsensorLB = A8; //Long Side sensor is attached on 
byte serialRead = 0; //for control serial communication
#define LF 0 //Long front sensor
#define LB 1 //Long back sensor
#define SF 2 //Short front sensor
#define SB 3 //Short rear sensor
float DistanceLong = 21.55;//WILL NEED TO BE TUNED add float to setup for Distance between front and back long sensors in CM
float DistanceShort = 21.55;//WILL NEED TO BE TUNED add float to setup for Distance between front and back short sensors in CM
////////////////////////////////////////////////////////////////////

//threshold for sonar error
float sonarThreshold = 2;
float yawThreshold = 5;

//int irsensor = A3; //sensor is attached on pinA0
//byte serialRead = 0; //for control serial communication
//int signalADC = 0; // the read out signal in 0-1023 corresponding to 0-5v

//Serial Controls
#define IR_SERIAL 0
#define US_SERIAL 1
#define GYRO_SERIAL

int pos = 0;

//Delay time between loops
int T = 50;

void setup()
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000); //settling time but no really needed

  Serial.begin(115200); // start serial communication

}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };

  if (Serial.available()) // Check for input from terminal
  {
  serialRead = Serial.read(); // Read input
    if (serialRead==49) // Check for flag to execute, 49 is ascii for 1, stop serial printing
    {
      Serial.end(); // end the serial communication to get the sensor data
    }
  }
  float SF_IR = DistanceIR(SF);
  float SR_IR = DistanceIR(SB);
  float S_IR = (SF_IR + SR_IR)/2;

  float yaw_IR = IrYawError(0);

  float Sonar = HC_SR04_range();

  FindCorner(Sonar, S_IR, yaw_IR);

  //CORNER-FINDER
  //STRAIGHT-DRIVE
  //AREA COVERAGE
  //STRAIGHT-DRIVE

  if(IR_SERIAL){ //Serial controll for the IR sensor
    Serial.print("YAW: "); Serial.print(IrYawError(1)); Serial.print("         ");
    Serial.print("LF: "); Serial.print(DistanceIR(LF)); Serial.print("cm  "); Serial.print(analogRead(irsensorLF)); Serial.print("      ");
    Serial.print("LB: "); Serial.print(DistanceIR(LB)); Serial.print("cm  "); Serial.print(analogRead(irsensorLB)); Serial.print("          ");
    Serial.print("SF: "); Serial.print(DistanceIR(SF)); Serial.print("cm  "); Serial.print(analogRead(irsensorSF)); Serial.print("         ");
    Serial.print("SB: "); Serial.print(DistanceIR(SB)); Serial.print("cm  "); Serial.print(analogRead(irsensorSB)); Serial.println("");
    
    
    delay(500);
  }

  delay(T);
}

STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {

  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

    #ifndef NO_READ_GYRO
      GYRO_reading();
    #endif

    #ifndef NO_HC-SR04
      HC_SR04_range();
    #endif

    #ifndef NO_BATTERY_V_OK
      if (!is_battery_voltage_OK()) return STOPPED;
    #endif


        turret_motor.write(pos);

        if (pos == 0)
        {
          pos = 45;
        }
        else
        {
          pos = 0;
        }
  }

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

#ifndef NO_HC-SR04
float HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return -2;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return -1;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
    return -1;
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
    return cm;
  }
}
#endif

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif

//Serial command pasing
void read_serial_command()
{
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        SerialCom->println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        SerialCom->println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      default:
        stop();
        SerialCom->println("stop");
        break;
      case 'h'://Turn Right
      case 'H':
        diagonal_upright();
        SerialCom->println("diagonal_upright");
        break;
      case 'g'://Turn Right
      case 'G':
        diagonal_upleft();
        SerialCom->println("diagonal_upleft");
        break;
      case 'n'://Turn Right
      case 'N':
        diagonal_downright();
        SerialCom->println("diagonal_downright");
        break;
      case 'b'://Turn Right
      case 'B':
        diagonal_downleft();
        SerialCom->println("diagonal_downleft");
        break;
    }

  }

}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void diagonal_upright ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);

  left_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void diagonal_upleft ()
{
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);

  left_font_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
}

void diagonal_downright ()
{
  right_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);

  left_font_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
}

void diagonal_downleft ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);

  left_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
} 

float DistanceIR(int range){//Could add the sonar sensor to this and just have one complete distance fuction

  static float distance = 0;

  if (range == LF){
    distance = analogRead(irsensorLF); // the read out is a signal from 0-1023 corresponding to 0-5v Short Front 
    distance = 13.16092 + (153.9881 - 13.16092)/(1 + pow((distance/78.4712), 2.3085)); // calculate the distance using the datasheet graph
  }
  else if (range == LB){
    distance = analogRead(irsensorLB); // the read out is a signal from 0-1023 corresponding to 0-5v Short Front 
    distance = 13.16092 + (153.9881 - 13.16092)/(1 + pow((distance/78.4712), 2.3085)); // calculate the distance using the datasheet graph
  }
  else if (range == SF){
    distance = analogRead(irsensorSF); // the read out is a signal from 0-1023 corresponding to 0-5v Short Front 
    distance = 2428*pow(distance,-1); // calculate the distance using the datasheet graph
  }
  else {
    distance = analogRead(irsensorSB); // the read out is a signal from 0-1023 corresponding to 0-5v Short Front 
    distance = 2428*pow(distance,-1); // calculate the distance using the datasheet graph
  }

  //Add a const to distance to make reading from centre of mass
  distance += 6.35;

  return distance;
}

float IrYawError(bool range)//Function calculates the yaw error in degrees of the robot based on the IR sensors
{
  float error = 0;

  if (range == 0){//If range is 0 get value for/from short range sensors
  error = (DistanceIR(SF)-DistanceIR(SB));
  error = tan(error/DistanceShort);
  }
  else {//Range is 1 get value for/from long range sensors
  error = (DistanceIR(LF)- DistanceIR(LB));
  error = tan(error/DistanceLong);
  }

  error = error * (180.0 / PI);//converts from rads to degrees
  return error;
}

void straight_align (float distance, float yaw_angle, float edgeDist)
{
  float angleError = 20* yaw_angle;
  float edgeError = 20 * (edgeDist - distance);

  left_font_motor.writeMicroseconds(1500 + angleError + edgeError);
  left_rear_motor.writeMicroseconds(1500 + angleError - edgeError);
  right_rear_motor.writeMicroseconds(1500 + angleError - edgeError);
  right_font_motor.writeMicroseconds(1500 + angleError + edgeError);

}

// Corner Finder
void FindCorner(float sonarDistance, float shortIR, float yawAngle){
  static int runState = 0;
  static int prevRunState = 0;
  static float maxSonar = 0;

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
}
