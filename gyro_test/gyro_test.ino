# include <Servo.h>   // include the library of servo motor control 

// define the control pin of each motor 
const byte left_front = 46; 
const byte left_rear = 47; 
const byte right_rear = 50; 
const byte right_front = 51;  


int sensorPin = A1;         //define the pin that gyro is connected 
int T = 100;                // T is the time of one loop, 0.1 sec
int sensorValue = 0;        // read out value of sensor 
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 0;  // the value of voltage when gyro is zero 
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet 
float rotationThreshold = 1.5;  // because of gyro drifting, defining rotation angular velocity less 
                                // than this value will be ignored
float gyroRate = 0;         // read out value of sensor in voltage 
float currentAngle = 0;     // current angle calculated by angular velocity integral on 
byte serialRead = 0;        // for serial print control 

// create servo objects for each motor  
Servo left_front_motor; 
Servo left_rear_motor; 
Servo right_rear_motor; 
Servo right_front_motor;

int speed_val = 0;

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

    rotate(u);    

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
    
    Serial.print(angularVelocity);
    Serial.print(" ");
    Serial.println(currentAngle);
    
    // control the time per loop 
    delay (T);
}

void disable_motors(){                             
    left_front_motor.detach(); 
    left_rear_motor.detach(); 
    right_rear_motor.detach(); 
    right_front_motor.detach(); 
} 


void enable_motors() {                                         
    left_front_motor.attach(left_front); 
    left_rear_motor.attach(left_rear); 
    right_rear_motor.attach(right_rear); 
    right_front_motor.attach(right_front); 
}

void stop(){                                                                
    left_front_motor.writeMicroseconds(1500); 
    left_rear_motor.writeMicroseconds(1500); 
    right_rear_motor.writeMicroseconds(1500); 
    right_front_motor.writeMicroseconds(1500); 
}


void rotate(int pwr) {
    left_front_motor.writeMicroseconds(1500 + speed_val); 
    left_rear_motor.writeMicroseconds(1500 + speed_val); 
    right_rear_motor.writeMicroseconds(1500 + speed_val); 
    right_front_motor.writeMicroseconds(1500 + speed_val);
}