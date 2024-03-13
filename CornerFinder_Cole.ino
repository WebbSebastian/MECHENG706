void stop();


// Corner Finder
void findCorner(float sonarDistance, float shortIR){
  static int runState = 0;
  static int prevRunState = 0;
  static float maxSonar = 0;
  
  int time45deg = 2000;

  switch (runState){
    //Reverse to wall
    case 0:
      static int numStationary = 0;
      static int  prevSonar = 0;
      reverse();

      //keep count of how often sonar reading doesn't change
      if ((sonarDistance - prevSonar) < threshold)
        numStationary++;

      prevSonar = sonarDistance;
      
      //if sonar detects stationary enough times, move to next state
      if(numStationary > 5){
        stop();
        prevRunState = runState;
      }

      if(prevRunState == 4)
        runState = 5;
      else
        runState++;

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
        runState = 6;
      }
      break;

    //Long walls pt. 1 (move forwards)
    case 3:
      static int sonarInit = sonarDistance;
      
      forward();

      if ((sonarInit - sonarDistance) > 5){
        stop();
        prevRunState = runState;
        runState++;
      }
      break;

    //Long walls pt. 2 (rotate CCW)
    case 4:
      static int timeElapsed = 0;

      if (timeElapsed >= time45deg){
        //P controller to get 0 angle => exit to case 0
      } else
        ccw();

      timeElapsed += T;
      break;

    //controller finished
    case 6:
      break;
  };
  
}



//redundant code to find the corner directly ahead
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