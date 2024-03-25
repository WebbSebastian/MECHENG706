void setup() {
  // put your setup code here, to run once:
  int drive = 0; // to tell the car the current action is to drive forwards/backwards
  int edge = 0; // to tell the car the current action is to drive laterally
  int driveDir = 0; // to tell the car to drive forward/backward
  int edgeCalc = 0; // to tell the car if it needs to calculate the distance from the wall to drive to
  int hasRotated = 1; // 1 until car has rotated, -1 after

  float edgeDist = 0; // the distance from the wall the car needs to drive laterally to
  float lateralStep = 0; // how many cm we want the car to drive laterally each time
}

void loop() {
  // put your main code here, to run repeatedly:
  ////////// Drive straight using short sensor then set drive to 1 ////////
  
  while (drive)
  {
    if(driveDir)
    {
      driveAtDistFromWall();
      if (atWall) //////// need to change atWall to whatever we are using to sense the end wall
      {
        drive = 0;
        driveDir = 0;
        edge = 1;
      }
      
    }
    else
    {
      driveAtDistFromWall();
      if (atWall) //////// need to change atWall to whatever we are using to sense the end wall
      {
        drive = 0;
        driveDir = 1;
        edge = 1;
      }
      
    }
  }

  while(edge)
  {
    if (edgeCalc)
    {
      edgeDist = /*current distance*/ + (hasRotated * lateralStep)
      if ((edgeDist <= /*final distance*/) && (hasRotated <= 0))
      {
        edgeDist = /*final distance*/
        finale = 1;
      }
      ////////// finale will tell the robot this is the last run //////////
      edgeCalc = 0;
    }
    if (hasRotated > 0)
    {
      if (edgeDist >= (/*tableLength*/ / 2))
      {
        rotate();
        hasRotated = -1;
        edgeDist = /*current Distance*/ - lateralStep;
        if (driveDir)
        {
          driveDir = 0;
        }
        else
        {
          driveDir = 1;
        }
      }
    }
    driveToDistFromWall()
    if (atEdgeWall) //////// need to change atEdgeWall to whatever we are using to sense the edge wall
    {
      drive = 1;
      edge = 0;
      edgeCalc = 1;
    }
  }

  while(finale)
  {
    //////// drive to end wall using short sensor then end ////////
  }
}
