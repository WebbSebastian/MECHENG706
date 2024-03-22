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

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.   
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 51;
const byte left_rear = 50;
const byte right_rear = 47;
const byte right_front = 46;

//const byte left_front = 47;
//const byte left_rear = 46;
//const byte right_rear = 51;
//const byte right_front = 50;


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

int speed_val = 200;
int speed_change;

// Variables for straight drive error
const float edgeDist = 30;
float angleError = 0;
float edgeError = 0;
float distanceUS = 0;

//Serial Pointer
HardwareSerial *SerialCom;

////////////////////// IR SENSOR PINS and Variables//////////////
const int irsensorSF = A10; //Short Front sensor is attached on pinA0
const int irsensorSS = A11; //Short Side sensor is attached on 
const int irsensorLF = A8; //Long Front sensor is attached on 
const int irsensorLS = A9; //Long Side sensor is attached on 
byte serialRead = 0; //for control serial communication
float ADCsignalSF = 0; // the read out signal in 0-1023 corresponding to 0-5v
float ADCsignalSS = 0; // the read out signal in 0-1023 corresponding to 0-5v
float ADCsignalLF = 0; // the read out signal in 0-1023 corresponding to 0-5v
float ADCsignalLS = 0; // the read out signal in 0-1023 corresponding to 0-5v
////////////////////////////////////////////////////////////////////

//test variables
float maxDist = 0;
float maxAngle = 0;
int has180 = 0;
int rotating = 1;
int align = 0;
int corner = 0;
//float currentAngle = 0;

int sensorPin = A2;         //define the pin that gyro is connected 
int T = 100;                // T is the time of one loop, 0.1 sec
int sensorValue = 0;        // read out value of sensor 
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 512;  // the value of voltage when gyro is zero 
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet 
float rotationThreshold = 1.5;  // because of gyro drifting, defining rotation angular velocity less 
                                // than this value will be ignored
float gyroRate = 0;         // read out value of sensor in voltage 
float current_Angle = 0;     // current angle calculated by angular velocity integral on 

//int irsensor = A3; //sensor is attached on pinA0
//byte serialRead = 0; //for control serial communication
//int signalADC = 0; // the read out signal in 0-1023 corresponding to 0-5v

//Serial Controls
#define IR_SERIAL 0
#define US_SERIAL 0
#define GYRO_SERIAL

int pos = 0;
void setup(void)
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

  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(sensorPin)*gyroSupplyVoltage)/1023; 
  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage/1023 * gyroSupplyVoltage); 
  // read out voltage divided the gyro sensitivity to calculate the angular velocity 
  float angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps
    
  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
      // we are running a loop in T (of T/1000 second).
      float angleChange = angularVelocity/(1000/T);
      current_Angle += angleChange; 
  }
    
  // keep the angle between 0-360
  if (current_Angle < 0)
  {current_Angle += 360;}   
  else if (current_Angle > 359)
  {current_Angle -= 360;}

  ADCsignalSF = analogRead(irsensorSF); // the read out is a signal from 0-1023 corresponding to 0-5v Short Front 
  ADCsignalSS = analogRead(irsensorSS); // the read out is a signal from 0-1023 corresponding to 0-5v Short Side
  ADCsignalLF = analogRead(irsensorLF); // the read out is a signal from 0-1023 corresponding to 0-5v Long Front 
  ADCsignalLS = analogRead(irsensorLS); // the read out is a signal from 0-1023 corresponding to 0-5v Long Side
  float distanceSF = 2428*pow(ADCsignalSF,-1); // calculate the distance using the datasheet graph
  float distanceSS = 2428*pow(ADCsignalSS,-1); // calculate the distance using the datasheet graph
  float distanceLF = 13.16092 + (153.9881 - 13.16092)/(1 + pow((ADCsignalLF/78.4712), 2.3085)); // calculate the distance using the datasheet graph
  float distanceLS = 13.16092 + (153.9881 - 13.16092)/(1 + pow((ADCsignalLS/78.4712), 2.3085)); // calculate the distance using the datasheet graph

  float distanceUS = HC_SR04_range();
  //SerialCom->println(distanceUS);

  float edgeDist = 30;
  int dir = 1;

  ////////////////////////// If driving forwards, LS then LF. If driving backwards, LF then LS //////////////////////////////////
  //straight_drive(distanceLF, distanceLS, distanceUS, dir, edgeDist); 
  //straight_drive(distanceLS, distanceLF, distanceUS, dir, edgeDist);
  //straight_align(distanceLS, distanceLF, edgeDist);


/*
  if (rotating)
  {
    rotate();
    if (distanceUS >= maxDist)
    {
      maxDist = distanceUS;
      maxAngle = current_Angle;
      //maxIR = 0;
    }
  }

  if (current_Angle >= 180)
  {
    if(current_Angle <= 270)
    {
      has180 = 1;
    }
  }

  if (current_Angle <= 30)
  {
    if (has180)
    {
      rotating = 0;
      align = 1;
    }
  }

  if(align)
  {
    rotate();
    if (current_Angle >= (maxAngle - 5))
    {
      if (current_Angle <= (maxAngle + 5))
      {
        align = 0;
        corner = 1;
      }
    }
  }

  if(corner)
  {
    corner_align(distanceLS, distanceLF, distanceUS, 15);
  }
  */

  corner_align(distanceLS, distanceLF, distanceUS, 15);
  
  

  //Serial.print(angularVelocity);
  //Serial.print(" ");
  //Serial.println(current_Angle);
  //Serial.print(" ");
  //Serial.print(analogRead(sensorPin));

  delay(100);


  if(IR_SERIAL){ //Serial controll for the IR sensor

    Serial.print("SF: "); Serial.print(distanceSF); Serial.print("cm  "); Serial.print(ADCsignalSF); Serial.print("      ");
    Serial.print("SS: "); Serial.print(distanceSS); Serial.print("cm  "); Serial.print(ADCsignalSS); Serial.print("          ");
    Serial.print("LF: "); Serial.print(distanceLF); Serial.print("cm  "); Serial.print(ADCsignalLF); Serial.print("         ");
    Serial.print("LS: "); Serial.print(distanceLS); Serial.print("cm "); Serial.print(ADCsignalLS); Serial.println("");
    Serial.print("US: "); Serial.print(distanceUS); Serial.print("cm "); Serial.println("");
    delay(50);

  }
  
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
      //HC_SR04_range();
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
      return;
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
      //SerialCom->println("HC-SR04: Out of range");
      return;
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
    //SerialCom->println("HC-SR04: Out of range");
  } else {
    //SerialCom->print("HC-SR04:");
    //SerialCom->print(cm);
    //SerialCom->println("cm");
  }
  return cm;
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

void rotate ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void lateral_move (float distanceLS, float distanceLF, float edgeDist, float targetDist, float direction)
{
  float edgeError = targetDist - edgeDist;
  left_font_motor.writeMicroseconds(1500 + direction * edgeError);
  left_rear_motor.writeMicroseconds(1500 - direction * edgeError);
  right_rear_motor.writeMicroseconds(1500 - direction * edgeError);
  right_font_motor.writeMicroseconds(1500 + direction * edgeError);
}

void straight_align (float distanceLS, float distanceLF, float edgeDist)
{
  float angleError = 20 * (distanceLS - distanceLF);
  float edgeError = 20 * (edgeDist - distanceLF);

  left_font_motor.writeMicroseconds(1500 + angleError + edgeError);
  left_rear_motor.writeMicroseconds(1500 + angleError - edgeError);
  right_rear_motor.writeMicroseconds(1500 + angleError - edgeError);
  right_font_motor.writeMicroseconds(1500 + angleError + edgeError);
}

void corner_align (float distanceLS, float distanceLF, float distanceUS, float edgeDist)
{
  float angleError = 20 * (distanceLS - distanceLF);
  float edgeError = 20 * (edgeDist - distanceLF);
  float distError = 20 * (198 - (24 + edgeDist));
  
  left_font_motor.writeMicroseconds(1500 - distError + angleError + edgeError);
  left_rear_motor.writeMicroseconds(1500 - distError + angleError - edgeError);
  right_rear_motor.writeMicroseconds(1500 + distError + angleError - edgeError);
  right_font_motor.writeMicroseconds(1500 + distError + angleError + edgeError);
}

void straight_drive (float distanceLS, float distanceLF, float distanceUS, float direction, float edgeDist)
{
  

  float angleError = direction * 30 * (distanceLS - distanceLF);
  //float edgeError = 30 * (edgeDist - (distanceLF + distanceLS)/2);
  float edgeError = 30 * (edgeDist - distanceLF);
  //int edgeError = 0;

  left_font_motor.writeMicroseconds(1500 + direction * (speed_val) + angleError + edgeError);
  left_rear_motor.writeMicroseconds(1500 + direction * (speed_val) + angleError - edgeError);
  right_rear_motor.writeMicroseconds(1500 - direction * (speed_val) + angleError - edgeError);
  right_font_motor.writeMicroseconds(1500 - direction * (speed_val) + angleError + edgeError);


  if (direction > 0)
  {
    if (distanceUS <= 15)
    {
      stop();
    } 
  }

  if (direction < 0)
  {
    if (distanceUS >= (198 - (24 + 15)))
    {
      stop();
    } 
  }

   
  
}

