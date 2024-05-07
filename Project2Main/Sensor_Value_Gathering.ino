class SensorDebug {
    private:
    float readingArray[50][4];


    public:
    //constructor
    SensorDebug(float sensorArray[50][4]){
        for(int i = 0; i < 50; i++){
            for(int j = 0; j < 4; j++){
                readingArray[i][j] = sensorArray[i][j];
            }
        }
    }

    //return average of sensor readings
    float getAvg(int sensor){       
        if(sensor > 4 || sensor <= 0)
            return -2;

        float sensorAvg = 0;

        for (int i = 0; i < 50; i++)
            sensorAvg += readingArray[i][sensor - 1];

        return sensorAvg/50.0;
    }

    //return highest/lowest sensor readings
    float* getBounds(int sensor){
        float bounds[2] = {-1, -1};

        if(sensor > 4 || sensor <= 0){
            return  bounds;
        }

        bounds[1] = readingArray[0][sensor - 1];
        bounds[2] = readingArray[0][sensor - 1];
        float localReading;

        for (int i = 1; i < 50; i++){
            localReading = readingArray[i][sensor - 1];

            if(localReading < bounds[1])
                bounds[1] = localReading;

            if(localReading > bounds[2])
                bounds[2] = localReading;        
        }

        return bounds;
    }
};


float sensorValuesInst[4];
float sensorValues[50][4];
SensorDebug* sensorResults[5][4];
int testStage = 0;

int testStageDistance[4] = {75, 100, 150, 200};
int testStageAngle[5] = {-30, -15, 0, 15, 30};
bool testMessage = true;

bool startTest = false;

int i = 0;

main loop() {
      if (Serial.available()) // Check for input from terminal
  {
    serialRead = Serial.read(); // Read input
    
    if (serialRead==50) // if input is 2, start testing pt
    {
      startTest = true;
    }
  }

    if (startTest && testStage < 20)
  {  
    sensorValues[i][0] = adc1;
    sensorValues[i][1] = adc2;
    sensorValues[i][2] = adc3;
    sensorValues[i][3] = adc4;

    i++;

    if(i == 49){
        i = 0;

        *sensorResults[testStage/4][testStage%4] = SensorDebug(sensorValues);
        testStage++;
        
        startTest = false;
        testMessage = true;
    }
  } else
  {
    if(testMessage){
      SerialCom->print("Position robot at "); SerialCom->print(testStageDistance); SerialCom->print("mm & "); SerialCom->print(testStageAngle); SerialCom->println(" degrees from fire. ");
      SerialCom->println("Input character 2 when done.");
      testMessage = false;      
    }
  }
}