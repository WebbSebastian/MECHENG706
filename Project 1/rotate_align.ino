# include <Servo.h>   // include the library of servo motor control 

// define the control pin of each motor 
const byte left_front = 51; 
const byte left_rear = 50; 
const byte right_rear = 47; 
const byte right_front = 46;  

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;


int sensorPin = A2;         //define the pin that gyro is connected 
int T = 100;                // T is the time of one loop, 0.1 sec
int sensorValue = 0;        // read out value of sensor 
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 515.5;  // the value of voltage when gyro is zero 
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet 
float rotationThreshold = 1.5;  // because of gyro drifting, defining rotation angular velocity less 
                                // than this value will be ignored
float gyroRate = 0;         // read out value of sensor in voltage 
float currentAngle = 0;     // current angle calculated by angular velocity integral on 
byte serialRead = 0;        // for serial print control 

int speed_val = 200;

//PID Parameters
float desiredAngle = 50;

float u = 0;

float error = 0;
float prevError = 0;
float errorDiff = 0;
float errorInt = 0;

float P = 1;
float I = 0;
float D = 0;

int stage1 = 1;
int stage2 = 0;
int stage3 = 0;

int maxAngle = 0; 
int currentDist = 0;
int maxDist = 0;

////////////////////// IR SENSOR PINS and Variables//////////////
const int irsensorSF = A10; //Short Front sensor is attached on pinA0
const int irsensorSS = A11; //Short Side sensor is attached on 
const int irsensorLF = A8; //Long Front sensor is attached on 
const int irsensorLS = A9; //Long Side sensor is attached on 
float ADCsignalSF = 0; // the read out signal in 0-1023 corresponding to 0-5v
float ADCsignalSS = 0; // the read out signal in 0-1023 corresponding to 0-5v
float ADCsignalLF = 0; // the read out signal in 0-1023 corresponding to 0-5v
float ADCsignalLS = 0; // the read out signal in 0-1023 corresponding to 0-5v
////////////////////////////////////////////////////////////////////

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

//Serial Pointer
HardwareSerial *SerialCom;

//test variables
float maxDist = 0;
float maxAngle = 0;
int has180 = 0;
int rotating = 1;
int align = 0;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    // this section is initialize the sensor, find the value of voltage when gyro is zero
    int i;
    float sum = 0; 
    pinMode(sensorPin,INPUT); 
    
    Serial.println("please keep the sensor still for calibration");
    Serial.println("get the gyro zero voltage");

    for (i=0;i<100;i++) // read 100 values of voltage when gyro is at still, to calculate the zero-drift. 
    {
        sensorValue = analogRead(sensorPin);
        sum += sensorValue;
        delay(5);
    }

    gyroZeroVoltage = sum/100; // average the sum as the zero drifting 

    enable_motors();
}


void loop() {
    // put your main code here, to run repeatedly:
    /*
    error = desiredAngle - currentAngle;
    
    errorDiff = (error - prevError) / T;
    errorInt = errorInt + error * T;


    u = P * error + I * errorInt + D * errorDiff;
    
    if (u > 500)
        u = 500;
    else if (u < -500)
        u = -500;

    speed_val = (int)u;

    prevError = error;
    */

    //rotate();    

    if (Serial.available()) // Check for input from terminal
    {
        serialRead = Serial.read(); // Read input

        if (serialRead==49) // Check for flag to execute, 49 is ascii for 1
        {
            stop();
            disable_motors();
            Serial.end(); // end the serial communication to display the sensor data on monitor
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
        currentAngle += angleChange; 
    }
    
    // keep the angle between 0-360
    if (currentAngle < 0)
    {currentAngle += 360;}   
    else if (currentAngle > 359)
    {currentAngle -= 360;}

    currentDist = HC_SR04_range();
    

    
    Serial.print(angularVelocity);
    Serial.print(" ");
    Serial.println(currentAngle);
    Serial.print(" ");
    Serial.print(analogRead(sensorPin));
    
    // control the time per loop 
    delay (T);

    if (rotating)
    {
      rotate();
      if (currentDist >= maxDist)
      {
        maxDist = currentDist;
        maxAngle = currentAngle;
        //maxIR = 0;
      }
    }

    if (currentAngle >= 180)
    {
      if(currentAngle <= 270)
      {
        has180 = 1;
      }
    }

    if (currentAngle <= 30)
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
      if (currentAngle >= (maxAngle - 5))
      {
        if (currentAngle <= (maxAngle + 5))
        {
          align = 0;
          stop();
        }
      }
    }

}

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
      SerialCom->println("HC-SR04: Out of range");
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
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
  return cm;
}
#endif

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

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

void disable_motors(){                             
    left_font_motor.detach(); 
    left_rear_motor.detach(); 
    right_rear_motor.detach(); 
    right_font_motor.detach(); 
} 

/*
void enable_motors() {                                         
    left_font_motor.attach(left_front); 
    left_rear_motor.attach(left_rear); 
    right_rear_motor.attach(right_rear); 
    right_font_motor.attach(right_front); 
}
*/

void stop(){                                                                
    left_font_motor.writeMicroseconds(1500); 
    left_rear_motor.writeMicroseconds(1500); 
    right_rear_motor.writeMicroseconds(1500); 
    right_font_motor.writeMicroseconds(1500); 
}


void rotate() {
    left_font_motor.writeMicroseconds(1500 + speed_val); 
    left_rear_motor.writeMicroseconds(1500 + speed_val); 
    right_rear_motor.writeMicroseconds(1500 + speed_val); 
    right_font_motor.writeMicroseconds(1500 + speed_val);
}

void straight_drive (float distanceLS, float distanceLF, float distanceUS, float direction, float edgeDist, float speed_val)
{
  

  float angleError = direction * 30 * (distanceLS - distanceLF);
  float edgeError = 30 * (edgeDist - (distanceLF + distanceLS)/2);
  //int edgeError = 0;

  left_font_motor.writeMicroseconds(1500 + direction * (speed_val) + angleError + edgeError);
  left_rear_motor.writeMicroseconds(1500 + direction * (speed_val) + angleError - edgeError);
  right_rear_motor.writeMicroseconds(1500 - direction * (speed_val) + angleError - edgeError);
  right_font_motor.writeMicroseconds(1500 - direction * (speed_val) + angleError + edgeError);


  if (direction > 0)
  {
    if (distanceUS <= 30)
    {
      stop();
    } 
  }

  if (direction < 0)
  {
    if (distanceUS >= (198 - (24 + 30)))
    {
      stop();
    } 
  }

   
  
}
