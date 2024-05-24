int test = 0;

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
const byte left_front = 46;
const byte left_rear = 50;
const byte right_rear = 47;
const byte right_front = 51;

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

//----------------------------------------------- IR SENSOR PINS----------------------------------------------------------//
const int irsensorFL = A10; //Front Left sensor SHORT RANGE
const int irsensorFR = A11; //Front Right sensor SHORT RANGE
const int irsensorRL = A8; //Rear Left sensor LONG RANGE
const int irsensorRR = A9; //Rear Right sensor LONG RANGE
int ir_pin_array[4] = {irsensorFL,irsensorFR,irsensorRL,irsensorRR};
//---------------------------------------------- PT PINS and Variables-----------------------------------------------------//
const int pt1 = A4;
const int pt2 = A5;
const int pt3 = A6;
const int pt4 = A7;
int pt_pin_array[4] = {pt1,pt2,pt3,pt4};
//----------------------------------------------- Avoidance global variables ----------------------------------------------//
int idle = 1;
int forwards = 0;
int leftArc = 0;
int rightArc = 0;
int backwards = 0;
int activeAvoid = 0;
int timeOut = 0;
int timer = 100;
int left = 0;
int right = 0;
int front = 0;
int motorUpper = 1600;
int motorLower = 1400;
float leftSide = 0;
float rightSide = 0;


//---------------------------------------------------- ULTRASONIC AND SERVO VARIABLES------------------------------------------//
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
int servoPin = 42;

#define USL 0 //Ultrasonic left
#define USF 1 //Ultrasonic Front
#define USR 2 //Ultrasonic right
long UStimerPrev = 0;
int USTime = 175; //ms min before US reading
int USstate = USF;//defualt US State 
int USstatePrev = USL; //set start sweep direction 
float USvalues[3] = {20,20,20};// US Sensors
int USdegrees[3] = {110,0,-110};

//-------------------------------------- PT VALUE ARRAY------------------------------------------//
int pt_adc_vals[4];

//-----------------------------------IR Boolean ARRAY--------------------------------------------//
bool ir_obj_detect[4]; //  {FL,FR,RL,RR} 1 if obstacle detected else 0

//----------------------------- seek and avoid motor commands -----------------------------------//
int motorCommands[4];
int seekMotorCommands[4];
int avoidMotorCommands[4];
float error = 0;

//------------------------------------ seek state variable ----------------------------------------//
#define ALIGN 0
#define DRIVE 1
#define EXTINGUISH 2
int seek_state = 0;

int pos = 0;
//------------------------------------ align variables ----------------------------------------//
int lastFireDetected = 0;
void setup(void)
{
  turret_motor.attach(servoPin);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000); //settling time but no really needed
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

  sensorGather();
  USReading();
  //seek();
  //Serial.print(seek_state);
  //driveTo();
  avoid();
  suppressor();
  timeOut += 1;
  
  left_font_motor.writeMicroseconds(motorCommands[0]);
  left_rear_motor.writeMicroseconds(motorCommands[1]);
  right_rear_motor.writeMicroseconds(motorCommands[2]);
  right_font_motor.writeMicroseconds(motorCommands[3]);

  //Serial.println(USvalues[0]);
  //Serial.println(USvalues[1]);
  //Serial.println(USvalues[2]);
  //int test = USvalues[0];
  //Serial.print("test = ");
  //Serial.println(test);
  //Serial.print("idle = ");
  //Serial.println(idle);
  //Serial.print("right = ");
  //Serial.println(right);
  //Serial.print("left = ");
  //Serial.println(left);
  //Serial.print("front = ");
  //Serial.println(front);
  //Serial.print("forwards = ");
  //Serial.println(forwards);
  //Serial.print("leftArc = ");
  //Serial.println(leftArc);
  //Serial.print("rightArc = ");
  //Serial.println(rightArc);
  //Serial.print("backwards = ");
  //Serial.println(backwards);
  //Serial.print("timeOut = ");
  //Serial.println(timeOut);

  //int forwards = 0;
//int leftArc = 0;
//int rightArc = 0;
//int backwards = 0;

  //Serial.print("pt4: ");
  // Serial.print(pt_adc_vals[1]);
  // Serial.print(',');
  // Serial.print(1030);
  // Serial.print(',');
  // Serial.println(0);
  // delay(40);

  // left_font_motor.writeMicroseconds(1500);
  // left_rear_motor.writeMicroseconds(1500);
  // right_rear_motor.writeMicroseconds(1500);
  // right_font_motor.writeMicroseconds(1700);
  delay(10);
}


void sensorGather(){
  int i;

  //calculate angle?
  for (i = 0; i < 4; i++){
    pt_adc_vals[i] = analogRead(pt_pin_array[i]);
  }
  for (i = 0; i < 4; i++){
    ir_obj_detect[i] = isObjectDetected(i);
  }
}

bool isObjectDetected(int pinIndex){
  int irADCVal = analogRead(ir_pin_array[pinIndex]);
  float dist = 0;
  const int objDetectThreshold = 10; //cm

  //old IR eqn.s (may need re-tuning)
  switch(pinIndex){

    case 0:  //FL
      dist = 1.552784 + (37047010 - 1.552784)/(1 + pow((irADCVal/0.003269664),1.361069));
          break;
    case 1:  //FR
      dist = -0.7084069 + (36.47473 - -0.7084069)/(1 + pow((irADCVal/111.6309),1.182963));
          break;
    case 2:  //RL
      dist = -13.60015 + (100.785 + 13.60015)/(1 + pow((irADCVal/58.12811),0.6362555));
          break;
    case 3:  //RR
      dist = 4.093802 + (77059520 - 4.093802)/(1 + pow((irADCVal/0.02542993),1.658215));
          break;
    default:
      dist = 0;
          break;
    return (dist < objDetectThreshold);
  }
}


void USReading() {
  if (millis() - UStimerPrev >= USTime) {//has the required time interval elapsed
    USvalues[USstate] = HC_SR04_range();//get reading for current sensor position 
    if (USstate == USF) {//If the state is front get the next state
      if (USstatePrev == USL) {
        USstate = USR;
      } else /*if (USstatePrev == USR)*/{
        USstate = USL;
      }
    }
    else {
      if (USstate == USL) {
        USstatePrev = USL; // Keep prev value to know which way to rotate when the servo is facing foward 
      }
      else /*if (USstate == USR)*/ {
        USstatePrev = USR;
      }
      USstate = USF; // Reset back to USF to complete the cycle
    }
    rotateServo(USdegrees[USstate]); // Adjust servo position based on current state
    UStimerPrev = millis(); // Reset the timer for the next interval
  }
}

void rotateServo(int degrees){
  //900 = -120 degrees, 2100 = +120 degrees, 1500 = 0
  int pwPD = 5; //plusewidth per degree
  int pwS = (degrees*pwPD)+1500;//plusewidth out to servo
  
  if(pwS>2100){//Cap outpt variable
    pwS = 2100;
  }
  else if(pwS<900){
    pwS = 900;
  }
  turret_motor.writeMicroseconds(pwS);
}

void seek(){
  if (seek_state == ALIGN){
    alignTo();
  } else if(seek_state == DRIVE){
    driveTo();
  } else if(seek_state == EXTINGUISH){
    extinguish();
  }
  //using pt array and IR array figure out motor commands
}
void alignTo(){
  int alignError = 3*pt_adc_vals[0] + pt_adc_vals[1] - pt_adc_vals[2] - 3*pt_adc_vals[3];
  int i;
  bool fireDetected = false;
  for (i = 0; i < 4;i++){
    if (pt_adc_vals[i] < 0.98*1023){
      fireDetected = true;
      lastFireDetected = millis();
    }
  }
  alignError = alignError*5;
  if(fireDetected || ((millis() - lastFireDetected) <= 200)){
    
    seekMotorCommands[0] = 1500 + SpeedCap( alignError, 300);
    seekMotorCommands[1] = 1500 + SpeedCap( alignError, 300);
    seekMotorCommands[2] = 1500 + SpeedCap( alignError, 300);
    seekMotorCommands[3] = 1500 + SpeedCap( alignError, 300);
    if (alignError < 10){
      seek_state = DRIVE;
    }
  } else if(!alignError) {
    seekMotorCommands[0] = 1600;
    seekMotorCommands[1] = 1600;
    seekMotorCommands[2] = 1600;
    seekMotorCommands[3] = 1600;
  }

}
void driveTo(){  // TODO currently this just moves forward immediately which could cause issues down the line.
  float u = 300;
  bool detected = 0; //checks if something detected
  float Kp = 0.5;  
  float Ki = 0;

  float integralerror = 0;
  bool direction = 1;
  int maxSpeed = 300;
  
  float currenterror = pt_adc_vals[1]- pt_adc_vals[0];
  //Serial.print(currenterror);
  //error = right -left sensor ;
  error = currenterror;
  
  Serial.print("pt0   ");
  Serial.print(pt_adc_vals[0]);
  Serial.print("                         ");
  Serial.print("pt1   ");
  Serial.print(pt_adc_vals[1]);
  Serial.print("                         ");
  // Serial.print("pt2   ");
  // Serial.print(pt_adc_vals[2);
  // Serial.print("                         ");
  // Serial.print("pt3   ");
  // Serial.println(pt_adc_vals[3]);



  if (error > 0){
    direction = 1;
  }
  else{
    direction = 1;
  }
  int i;
  for (i = 0; i < 4; i++){
  if(ir_obj_detect[4]==1)
    detected = 1;
  }

  if(detected == 1) {
    avoid();
  }
  Serial.println(error);
  error = SpeedCap(Kp * error + Ki * integralerror, maxSpeed);
  
  // add state change to extinguishing

  seekMotorCommands[0] = 1500 - error + 200; // should a direction be added?
  seekMotorCommands[1] = 1500 + error + 200;
  seekMotorCommands[2] = 1500 + error - 200;
  seekMotorCommands[3] = 1500 - error - 200;
  
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

void extinguish(){
 unsigned long start = millis();

}
void avoid()
{
  Serial.println("TEST");
  Serial.println(USvalues[0]);
  if (USvalues[0] <= 15){
    left = 1;
  }
  else {
    left = 0;
  }
  if (USvalues[2] <= 15){
    right = 1;
  }
  else{
    right = 0;
  }
  if (USvalues[1] <= 10){
    front = 1;
  }
  else{
    front = 0;
  }
  
  leftSide = ir_obj_detect[0];
  rightSide = ir_obj_detect[1];

  if (idle){
    if(front){
      idle = 0;
      timeOut = 0;
      backwards = 1;
    }
    else if(right){
      idle = 0;
      timeOut = 0;
      if (left){
        backwards = 1;
      }
      else{
        leftArc = 1;
      }
    }
    else if (left){
      idle = 0;
      timeOut = 0;
      rightArc = 1;
    }
    
  }

  if (forwards){
    if (right){
      forwards = 0;
      timeOut = 0;
      if (left){
        backwards = 1;
      }
      else{
        leftArc = 1;
      }
    }
    else if (left){
      forwards = 0;
      timeOut = 0;
      rightArc = 1;
    }
    else if (front){
      backwards = 1;
      forwards = 0;
      timeOut = 0;
    }
    else if (ir_obj_detect[0] || ir_obj_detect[1]){
      timeOut = 0;
    }

    if (timeOut >= timer){
      forwards = 0;
      idle = 1;
    }
  }

  if (backwards){
    if (timeOut >= timer){
      backwards = 0;
      timeOut = 0;
      rightArc = 1;
    }
  }

  if (leftArc){
    if (leftSide <= 5){
      leftArc = 0;
      timeOut = 0;
      backwards = 1;
    }
    else if (timeOut >= timer){
      leftArc = 0;
      timeOut = 0;
      forwards = 1;
    }
  }

  if (rightArc){
    if (rightSide <= 5){
      rightArc = 0;
      timeOut = 0;
      backwards = 1;
    }
    else if (timeOut >= timer){
      rightArc = 0;
      timeOut = 0;
      forwards = 1;
    }
  }

  if (forwards){
    avoidMotorCommands[0] = motorUpper;
    avoidMotorCommands[1] = motorUpper;
    avoidMotorCommands[2] = motorLower;
    avoidMotorCommands[3] = motorLower;
  }
  else if (backwards){
    avoidMotorCommands[0] = motorLower;
    avoidMotorCommands[1] = motorLower;
    avoidMotorCommands[2] = motorUpper;
    avoidMotorCommands[3] = motorUpper;
  }
  else if (idle){
    avoidMotorCommands[0] = motorUpper;
    avoidMotorCommands[1] = motorUpper;
    avoidMotorCommands[2] = motorLower;
    avoidMotorCommands[3] = motorLower;
  }
  else if (leftArc){
    avoidMotorCommands[0] = motorLower;
    avoidMotorCommands[1] = motorLower;
    avoidMotorCommands[2] = motorLower;
    avoidMotorCommands[3] = motorLower;
  }
  else if (rightArc){
    avoidMotorCommands[0] = motorUpper;
    avoidMotorCommands[1] = motorUpper;
    avoidMotorCommands[2] = motorUpper;
    avoidMotorCommands[3] = motorUpper;
  }


}
/*
void avoid(){
  avoidMotorCommands[0] = 1700;
  avoidMotorCommands[1] = 1700;
  avoidMotorCommands[2] = 1300;
  avoidMotorCommands[3] = 1300;
}
*/
void suppressor(){
  int i;
  if(0){
    for (i = 0; i < 4; i++){
    //Serial.println(error);
    motorCommands[i] = seekMotorCommands[i];
    }
  } else {
    for (i = 0; i < 4; i++){
    motorCommands[i] = avoidMotorCommands[i];
    }
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
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif

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
  // Calculate distance in centimeters and inches. The constants are found in the datasheet, and calculated from the assumed speed of sound in air at sea level (~340 m/s).
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



// void avoid(float left, float right, float leftSide, float rightSide)
// {
//   if (idle)
//   {
//     if(right)
//     {
//       idle = 0;
//       timeOut = 0;
//       if (left)
//       {
//         backwards = 1;
//       }
//       else
//       {
//         leftArc = 1;
//       }
//     }
//     else if (left)
//     {
//       idle = 0;
//       timeOut = 0;
//       rightArc = 1;
//     }
//   }

//   if (forward)
//   {
//     if (right)
//     {
//       forward = 0;
//       timeOut = 0;
//       if (left)
//       {
//         backwards = 1;
//       }
//       else
//       {
//         leftArc = 1;
//       }
//     }
//     else if (left)
//     {
//       forward = 0;
//       timeOut = 0;
//       rightArc = 1;
//     }

//     if (timeOut >= timer)
//     {
//       forward = 0;
//       idle = 1;
//     }
//   }

//   if (backward)
//   {
//     if (timeOut >= timer)
//     {
//       backwards = 0;
//       timeOut = 0;
//       rightArc = 1;
//     }
//   }

//   if (leftArc)
//   {
//     if (timeOut >= timer)
//     {
//       leftArc = 0;
//       timeOut = 0;
//       forward = 1;
//     }
//   }

//   if (rightArc)
//   {
//     if (timeOut >= timer)
//     {
//       rightArc = 0;
//       timeOut = 0;
//       forward = 1;
//     }
//   }

// }

