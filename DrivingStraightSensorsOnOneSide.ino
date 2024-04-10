#define arraySize = 8;
class Sensor {
   private:
   int pin;
   bool isLeft;
   bool isFront;
   bool isShort;
   
   float prevDistance;
   public:
   bool prevDistanceStored;

   Sensor(int Pin,bool IsLeft,bool IsFront,bool IsShort){
     this->pin = Pin;
     this->isLeft = IsLeft;
     this->isFront = IsFront;
     this->isShort = IsShort;
     this->prevDistanceStored = false;
     this->prevDistance = 0; 
   }
   float getDistance(){
     float cm = 0;
     if (isShort){
       cm = 2428*pow(analogRead(pin),-1);;
     } else {
      cm = 13.16092 + (153.9881 - 13.16092)/(1 + pow((analogRead(pin)/78.4712), 2.3085));
     }
     return cm;
   }
   float getDistanceAveraged(){
     
     float current = getDistance();
     float averaged = 0;
     if (prevDistanceStored){
       float variabilityAllowance = 3;
       float change = current - prevDistance;
       if(abs(change)>variabilityAllowance){
         change = variabilityAllowance;
       }
       change = abs(change)/variabilityAllowance;
       averaged = prevDistance*(0.1 + change*0.8) + current*(0.9 - change*0.8);
     } else {
       averaged = getDistanceMean();
       prevDistanceStored = true;
     }  
     prevDistance = averaged;
     return averaged;
   }
   float getDistanceMean(){
     int numOfReadings = 15;
     float mean = 0;
     int i = 0;
     while (i < numOfReadings ){
       mean += getDistance();
       delay(1);
       i++;
     }
     return (mean)/numOfReadings;
   }  
   int getPin(){
     return pin;
   }
   bool isSensorLeft(){
     return isLeft;
   }
   bool isSensorFront(){
     return isFront;
   }
   bool isSensorShort(){
     return isShort;
   }
};



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

//Default motor control pins
const byte left_front = 47;
const byte left_rear = 46;
const byte right_rear = 51;
const byte right_front = 50;


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
const int irsensorSR = A10; //Short Front sensor is attached on pinA0
const int irsensorSF = A11; //Short Side sensor is attached on 
const int irsensorLR = A9; //Long Side sensor is attached on 
const int irsensorLF = A8; //Long Front sensor is attached on 
byte serialRead = 0; //for control serial communication
float ADCsignalSF = 0; // the read out signal in 0-1023 corresponding to 0-5v
float ADCsignalSS = 0; // the read out signal in 0-1023 corresponding to 0-5v
float ADCsignalLF = 0; // the read out signal in 0-1023 corresponding to 0-5v
float ADCsignalLS = 0; // the read out signal in 0-1023 corresponding to 0-5v
////////////////////////////////////////////////////////////////////

Sensor shortRear(irsensorSR,true,false,true);
Sensor shortFront(irsensorSF,true,true,true);
Sensor longRear(irsensorLR,true,false,false);
Sensor longFront(irsensorLF,true,true,false);

//int irsensor = A3; //sensor is attached on pinA0
//byte serialRead = 0; //for control serial communication
//int signalADC = 0; // the read out signal in 0-1023 corresponding to 0-5v

//Serial Controls
#define IR_SERIAL 1
#define US_SERIAL 0
#define GYRO_SERIAL

int x = 1;
int pos = 0;
float readingFrontRight,readingRearRight,readingFrontLeft,readingRearLeft;
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
  // ADCsignalSF = analogRead(irsensorSR); // the read out is a signal from 0-1023 corresponding to 0-5v Short Front 
  // ADCsignalSS = analogRead(irsensorSF); // the read out is a signal from 0-1023 corresponding to 0-5v Short Side
  // ADCsignalLF = analogRead(irsensorLR); // the read out is a signal from 0-1023 corresponding to 0-5v Long Front 
  // ADCsignalLS = analogRead(irsensorLF); // the read out is a signal from 0-1023 corresponding to 0-5v Long Side
  // float distanceSF = 2428*pow(ADCsignalSF,-1); // calculate the distance using the datasheet graph
  // float distanceSS = 2428*pow(ADCsignalSS,-1); // calculate the distance using the datasheet graph
  // float distanceLF = 13.16092 + (153.9881 - 13.16092)/(1 + pow((ADCsignalLF/78.4712), 2.3085)); // calculate the distance using the datasheet graph
  // float distanceLS = 13.16092 + (153.9881 - 13.16092)/(1 + pow((ADCsignalLS/78.4712), 2.3085)); // calculate the distance using the datasheet graph

  // float distanceUS = HC_SR04_range();
  //SerialCom->println(distanceUS);


  //straight_drive(distanceLS, distanceLF, distanceUS);
  if(x == 1){
  driveAtDistFromWall(shortFront, shortRear, 8, 5);
  _delay_ms(100);
  strafe_right();
  _delay_ms(500);
  stop();
  _delay_ms(100);
  driveAtDistFromWall(shortFront, shortRear, 18, 170);
  _delay_ms(100);
  strafe_right();
  _delay_ms(500);
  stop();
  _delay_ms(100);
  driveAtDistFromWall(longFront, longRear, 28, 5);
  _delay_ms(100);
  strafe_right();
  _delay_ms(500);
  stop();
  _delay_ms(100);
  driveAtDistFromWall(longFront, longRear, 38, 170);
    _delay_ms(100);
  strafe_right();
  _delay_ms(500);
  stop();
  _delay_ms(100);
  driveAtDistFromWall(longFront, longRear, 48, 5);

  }
  x = 0;
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
void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}
float getSensorDist(Sensor sensor){
  float cm = 0;
  if (sensor.isSensorShort()){
    cm = 2428*pow(analogRead(sensor.getPin()),-1);;
  } else {
    cm = 13.16092 + (153.9881 - 13.16092)/(1 + pow((analogRead(sensor.getPin())/78.4712), 2.3085));
  }
  return cm;
}
void driveAtDistFromWall(Sensor frontSensor, Sensor rearSensor, float distFromWall, float sonarDesired){
  if(!frontSensor.isSensorFront()){
    return;
  }
  if(rearSensor.isSensorFront()){
    return;
  }
  if(frontSensor.isSensorLeft() != rearSensor.isSensorLeft()){// sensors must be from same side
    return;
  } 

  frontSensor.prevDistanceStored = false;
  rearSensor.prevDistanceStored = false;

  float sonarReading = 0;
  float readingFrontIR = 0;
  float readingRearIR = 0;

  //Proportional gains
  int Ka = 5;
  int Kd = 10;
  int Ks = 8;

  //integral gains
  float KaI = 0.2;
  float KdI = 0.5;
  float KsI = 0.2;
  
  // variable for exit condition
  int withinRange = 0;
  
  //current errors
  float currentAngleError = 0;
  float currentDistError = 0;
  float currentSonarError = 0;

  //error integrals
  float angleErrorIntegral = 0;
  float distErrorIntegral = 0;
  float sonarErrorIntegral = 0;

  //final Error coefficients
  int angleError = 0;
  int distError = 0; 
  int sonarError = 0;

  //Variable for kinematic calculation
  int leftOrRight = frontSensor.isSensorLeft() ? -1 : 1;

  int numOfVals = 10;
  int i = 0;


  while(withinRange < 10){


    // i=0;
    // readingFrontIR = 0;
    // readingRearIR = 0;
    // while(i < numOfVals){
      readingFrontIR += frontSensor.getDistanceAveraged();
      readingRearIR += rearSensor.getDistanceAveraged();
    //   delay(1);
    //   i++;
    // }
    
    // readingFrontIR = readingFrontIR/(float)numOfVals;
    // readingRearIR = readingRearIR/(float)numOfVals;

    currentAngleError = readingFrontIR - readingRearIR;
    currentDistError = ((readingFrontIR + readingRearIR)/2) - distFromWall;

    sonarReading = HC_SR04_range();
    currentSonarError = sonarReading - sonarDesired;

    if ((sonarReading > 120)){ // ends of the table
      rearSensor.prevDistanceStored = false;
      currentDistError = readingFrontIR - distFromWall;
      // distErrorIntegral = 0;
      currentAngleError = 0;
      angleErrorIntegral = 0;
    }
    if (sonarReading < 20){ // ends of the table
      frontSensor.prevDistanceStored = false;
      currentDistError = readingRearIR - distFromWall;
      // distErrorIntegral = 0;
      currentAngleError = 0;
      angleErrorIntegral = 0;
    }
    angleErrorIntegral += currentAngleError;
    distErrorIntegral += currentDistError;
    if (abs(currentSonarError)< 10){
      sonarErrorIntegral += currentSonarError;
    } else {
      sonarErrorIntegral = 0;
    }


    angleError = SpeedCap(Ka*(currentAngleError) + KaI*angleErrorIntegral,500);
    distError = SpeedCap(Kd*currentDistError + KdI*distErrorIntegral,500 - abs(angleError));
    sonarError = SpeedCap(Ks*currentSonarError + KsI*sonarErrorIntegral,500 - abs(angleError) - abs(distError));
    
    right_rear_motor.writeMicroseconds(1500 + sonarError + leftOrRight*(angleError + distError));
    right_font_motor.writeMicroseconds(1500 + sonarError + leftOrRight*(angleError - distError));
    left_font_motor.writeMicroseconds(1500 - sonarError + leftOrRight*(angleError - distError));
    left_rear_motor.writeMicroseconds(1500 - sonarError + leftOrRight*(angleError + distError));

    if(abs(currentSonarError) < 5){
      withinRange++;
    } else {
      withinRange = 0;
    }
    _delay_ms(2);
  }
  Serial.println("Exit");
  stop();
  return;
} 



int SpeedCap(float speed,int maxSpeed){
  int adjustedSpeed = speed;
  if (speed > maxSpeed){
    adjustedSpeed = maxSpeed;
  }
  else if (speed < -maxSpeed){
    adjustedSpeed = -maxSpeed;
  }

  return adjustedSpeed;
}
