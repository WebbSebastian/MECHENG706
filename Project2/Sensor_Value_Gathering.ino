class SensorDebug {
    private:
    float readingArray[50][4];

    bool isCorrectArray(float **array){
        int numRow;
        int numColumn;

        numColumn = sizeof(array[0])/sizeof(array[0][0]);
        numRow = sizeof(array)/sizeof(array[0]);

        return (numRow == 50) && (numColumn == 4);
    }
    public:

    SensorDebug(float** sensorArray){
        if(isCorrectArray(sensorArray)){
            for(int i = 0; i < 50; i++){
                for(int j = 0; j < 4; j++){
                    readingArray[i][j] = sensorArray[i][j];
                }
            }
        }
    }

    float getAvg(int sensor){       
        if(sensor > 4 || sensor <= 0)
            return -2;

        float sensorAvg = 0;

        for (int i = 0; i < 50; i++)
            sensorAvg += readingArray[i][sensor - 1];

        return sensorAvg/50.0;
    }

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

int i = 0;

main loop() {
    sensorValues[i][0] = pt1;
    sensorValues[i][1] = pt2;
    sensorValues[i][2] = pt3;
    sensorValues[i][3] = pt4;

    i++;

    if(i == 49){
        i = 0;
        SensorDebug iter1 = SensorDebug(sensorValues);
    }
}