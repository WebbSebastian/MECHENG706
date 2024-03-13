void rotate90CW_OL();


// Corner Finder
void cornerFindOpenLoop(float sonarDistance){
  float longest = 0;
  float time_elsapsed = 0;
  
  //spin robot 90 degrees => may need gyro for this?
  //(don't need to do full 360 as at least one corner will be passed in 90 degrees)

  //Could also rotate for a set amount of time that we know will pass 90 degrees reliably

  //poll this while rotating
  if(sonarDistance > longest){
    longest = sonarDistance;
  }

  while(sonarDistance < longest){
    //turn robot a little bit
    ccw();
  }

  while(sonarDistance > 20){
    forward();
  }

  averageShort = (distanceSF + distanceSB)/2;
  averageLong = (distanceLF + distanceLB)/2;



  if(averageShort <= 30){ 

    while(averageShort > 10){
      // move robot right till wall
    }
  }
  else if(averageLong <= 80){
    if(averageLong < 60){
      while(averageLong > 10){
        //move left till wall
      }
    }
    else {
      while(averageLong > 10){
        // move robot right till wall
      }
    }
  }
  
  Serial.println("Am I in a corner?");

///////////////////////////////////

  /*float shortLF = 0;
  float shortLB = 0;
  bool turnRight = 0;
  
  // spin robot 360 degrees
  
  if(distanceLF > shortLF){shortLF = distanceLF;}
  if(distanceLB > shortLB){shortLB = distanceLFB;}
   float aveShort = (distanceLF + distanceLB)/2;

  // spin robot again till you refind the aveShort distance

  if(sonarDistance > 100){
    turnRight = 1;
    shortSonar = 200 - sonarDistance;
    while(sonarDistance > shortSonar){ 
      // turn robot a little bit more than 90 degrees 
      // turn robot small amounts after that
    }
    
  }

  // move forward till 10cm or something.... using sonar sensor
  if(turnRight){ // have to turn right as the short side has the long distance sensors vs short, could move right first using long distance then turn to sonar later aswell

    //turn right 90 degrees 
    // move forward till 10 cm...
  }
  else{
    // move left till short sensors pick up 10 cm 
  }
 */


 
}



