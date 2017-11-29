#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <waypointClass.h>
#include <SparkFun_MAG3110.h>
#include <NewPing.h>
#include <moving_average.h>                       // simple moving average class; for Sonar functionality


/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
MAG3110 mag = MAG3110(); //Instantiate MAG3110
// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

boolean usingInterrupt = false;
float currentLat,
      currentLong,
      targetLat,
      targetLong,targetLat2,
      targetLong2;
int distanceToTarget,            // current distance to target (current waypoint)
    originalDistanceToTarget;    // distance to original waypoing when we started navigating to it

// Waypoints
#define WAYPOINT_DIST_TOLERANE  5   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 4         // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()

waypointClass waypointList[NUMBER_WAYPOINTS] = {waypointClass(-83.906478, 9.853903),waypointClass(-83.906562, 9.853563),waypointClass(-83.906272, 9.853385),waypointClass(-83.906196, 9.853349)};

/*9.930070   -84.035911
9.930083   -84.035659*/
/*9.930235   -84.035911
9.930262   -84.035812
9.930335   -84.035812
9.930454   -84.035827
9.930370   -84.035858
9.930310   -84.035888
9.930268   -84.035888*/
// Compass navigation
float targetHeading;              // where we want to go to reach current waypoint
float currentHeading;             // where we are actually facing now
float headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99

// Steering/turning 
enum directions {left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT} ;
directions turnDirection = straight;

// Object avoidance distances (in inches)
#define SAFE_DISTANCE 70
#define TURN_DISTANCE 40
#define STOP_DISTANCE 12

// MOTORES

int ENA = 10; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 9;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 8;  // MCU Digital Pin 8 to IN2 on L298n Board
 
int ENB = 11;  // MCU PWM Pin 5 to ENB on L298n Board
int IN3 = 12;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 13;  // MCU Digital pin 6 to IN4 on L298n Board


// Ultrasonic ping sensor
#define TRIGGER_PIN  6      
#define ECHO_PIN  7         
#define MAX_DISTANCE_CM 250                        // Maximum distance we want to ping for (in CENTIMETERS). Maximum sensor distance is rated at 400-500cm.  
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM / 2.5)    // same distance, in inches
int sonarDistance;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);     // NewPing setup of pins and maximum distance.
MovingAverage<int, 3> sonarAverage(MAX_DISTANCE_IN);       // moving average of last n pings, initialize at MAX_DISTANCE_IN

float dist_AD, brng_AD, dist_AB, brng_AB, dist_XT, dist_AT;





void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
  
  
  mag.initialize(); //Initializes the mag sensor
  mag.start();      //Puts the sensor in active mode
  

  pinMode(ENA, OUTPUT); //Set all the L298n Pin to output
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPS++ features"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("---------------------------------------------------------------------------------------------------------------------------------------"));
   // get initial waypoint; also sets the distanceToTarget and courseToTarget varilables
   smartDelay(1000);
   nextWaypoint();
   
}

void loop()
{ 
   int x, y, z;

  if(!mag.isCalibrated()) //If we're not calibrated
  {
    if(!mag.isCalibrating()) //And we're not currently calibrating
    {
      Serial.println("Entering calibration mode");
      mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
    }
    else
    {
      //Must call every loop while calibrating to collect calibration data
      //This will automatically exit calibration
      //You can terminate calibration early by calling mag.exitCalMode();
      mag.calibrate(); 
    }
  }
  else
  {
    Serial.println("Calibrated!");
    
  }
  

  Serial.print("Heading: ");
  Serial.println(mag.readHeading());

  Serial.println("--------");
  
  delay(100);
  processGPS();

  // navigate 
  currentHeading = readCompass();    // get our current heading
  calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles

   // distance in front of us, move, and avoid obstacles as necessary
  checkSonar();
  moveAndAvoid(); 
  
  static const double LONDON_LAT = 9.854561, LONDON_LON = -83.907534;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);
  printInt( distanceToTarget, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
    
  

  
}

void nextWaypoint(void)
{
  
  waypointNumber++;
  
  targetLat = waypointList[waypointNumber].getLat();
  targetLong = waypointList[waypointNumber].getLong();
 
  if ((targetLat == 0 && targetLong == 0) || waypointNumber >= NUMBER_WAYPOINTS)    // last waypoint reached? 
  {
    Detener();    // make sure we stop
    //turnMotor->run(RELEASE);  
    loopForever();
  }
  
   processGPS();
   distanceToTarget = originalDistanceToTarget = distanceToWaypoint();
   courseToWaypoint();
   
}  // nextWaypoint()



// end of program routine, loops forever
void loopForever(void)
{
  while (1)
    ;
}

// Called after new GPS data is received; updates our position and course/distance to waypoint
void processGPS(void)
{
  
  Serial.println(F("inicio el process"));
  currentLat = convertDegMinToDecDeg(gps.location.lat());
  currentLong = convertDegMinToDecDeg(gps.location.lng());
  Serial.println(currentLat);
  Serial.println(currentLong);
  Serial.println(convertDegMinToDecDeg(gps.location.lng()));
  
             
  // update the course and distance to waypoint based on our new position
  distanceToWaypoint();
  
  courseToWaypoint();
   
}   // processGPS(void)



// returns distance in meters between two positions, both specified 
// as signed decimal-degrees latitude and longitude. Uses great-circle 
// distance computation for hypothetical sphere of radius 6372795 meters.
// Because Earth is no exact sphere, rounding errors may be up to 0.5%.
// copied from TinyGPS library
int distanceToWaypoint() 
{
  
  Serial.println(F("inicio de distance to W metros q faltan"));
  distanceToTarget =  (long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);
   
   Serial.println(distanceToTarget);
  // check to see if we have reached the current waypoint
  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE){
      nextWaypoint();
      Serial.println(F("cambio de waypoint"));}  
      
  Serial.println(F("Final de distance to W"));  
  return distanceToTarget;
}  // distanceToWaypoint()

// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// copied from TinyGPS library
float courseToWaypoint() 
{
      Serial.println(F("inicio de course"));
      float a2=TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);
        targetHeading = a2;
      Serial.println(a2);
      Serial.println(F("Final"));
      return targetHeading;
}   // courseToWaypoint()


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}


float readCompass(void)
{
  Serial.println(F("inicio de magnetometro"));
  int x, y, z;
   //Only read data when it's ready
  if(mag.dataReady()) {
    //Read the data
    mag.readMag(&x, &y, &z);
  Serial.println(x);
  Serial.println(y);}   
  float heading = atan2(y, x);
  float heading2 = mag.readHeading(); 
  Serial.println("heading calculado");
  Serial.println(heading); 
  Serial.println("heading funcionmag");
  Serial.println(mag.readHeading()); 
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/ 
  // Cedar Park, TX: Magnetic declination: 4° 11' EAST (POSITIVE);  1 degreee = 0.0174532925 radians
  
  #define DEC_ANGLE -0.017
  heading += DEC_ANGLE;
  heading2 += DEC_ANGLE;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Correct for when signs are reversed.
  if(heading2 < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading2 > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;
  float headingDegrees2 = heading2 * 180/M_PI;
  Serial.println("curso actual en grados con formula"); 
  Serial.println(headingDegrees);
  Serial.println("curso actual en grados con funcion mag"); 
  Serial.println(headingDegrees2);
  return ((int)headingDegrees); 
}  // readCompass()

void calcDesiredTurn(void)
{
     
    Serial.println(F("inicio de vuelta"));
    Serial.println(currentHeading);
    Serial.println(targetHeading);
    // calculate where we need to turn to head to destination
    headingError = targetHeading-currentHeading;
    Serial.println(F("heading error"));
    Serial.println(headingError);
    // adjust for compass wrap
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;

    Serial.println(F("heading error corregido"));
    Serial.println(headingError);
    // calculate which way to turn to intercept the targetHeading
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
      turnDirection = straight; 
    else if (headingError < 0)
      turnDirection = left;
    else if (headingError > 0)
      turnDirection = right;
    else
      turnDirection = straight;

    Serial.println(F("la direccion es"));
    Serial.println(turnDirection);  
 
}  // calcDesiredTurn()



void checkSonar(void)
{   
 
  int dist;

  dist = sonar.ping_in();                   // get distqnce in inches from the sensor
  if (dist == 0)                                // if too far to measure, return max distance;
    dist = MAX_DISTANCE_IN;  
  sonarDistance = sonarAverage.add(dist);      // add the new value into moving average, use resulting average
} // checkSonar()

void moveAndAvoid(void)
{
     Serial.println(F("Move and avoid"));
    if (sonarDistance >= SAFE_DISTANCE)       // no close objects in front of car
        {
           Serial.println(F("adelante"));
           Serial.println(turnDirection);
           if (turnDirection == straight)
             AdelanteHS();
           else if(turnDirection == left)
             IZQUIERDA();
           else if(turnDirection == right)
             DERECHA();
             
              
           return;
        }
      
     if (sonarDistance > TURN_DISTANCE && sonarDistance < SAFE_DISTANCE)    // not yet time to turn, but slow down
       {
         if (turnDirection == straight)
           Normal();
         else
           {
              if (turnDirection == straight)
             Normal();
           else if(turnDirection == left)
             IZQUIERDA();
           else if(turnDirection == right)
             DERECHA();     // alraedy turning to navigate
            }       
         return;
       }
     
     if (sonarDistance <  TURN_DISTANCE && sonarDistance > STOP_DISTANCE)  // getting close, time to turn to avoid object        
        {
          Slowdown(); 
          switch (turnDirection)
          {
            case straight:                  // going straight currently, so start new turn
              {
                if (headingError <= 0)
                  IZQUIERDA();
                else
                  DERECHA();
                
                break;
              }
            case left:                         // if already turning left, try right
              {
                DERECHA();    
                break;  
              }
            case right:                       // if already turning right, try left
              {
                IZQUIERDA();
                break;
              }
          } // end SWITCH
          
         return;
        }  


     if (sonarDistance <  STOP_DISTANCE)          // too close, stop and back up
       {
         Detener();
         turnDirection = straight;
         Atras();          
         while (sonarDistance < TURN_DISTANCE)       // backup until we get safe clearance
           {
              if(gps.location.isValid())
                 processGPS();  
              currentHeading = readCompass();    // get our current heading
              calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles      
              checkSonar();
              delay(100);
           } // while (sonarDistance < TURN_DISTANCE)
         Detener();        // stop backing up
         return;
        } // end of IF TOO CLOSE
     
}   // moveAndAvoid()

void AdelanteHS()//Camina a alta velocidad 
{
  // Run the motors on both direction at fixed speed
  digitalWrite(IN1, HIGH); // Turn HIGH motor A
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200); // TO set the turning speed to 200 out of possible range 0 to 255
 
  digitalWrite(IN3, HIGH); // turn HIGH motor B
  digitalWrite(IN4, LOW);  // To set the turning speed to 200 out of possible range 0 to 255 
  analogWrite(ENB, 200);
}

void Normal()//Camina a alta velocidad 
{
  // Run the motors on both direction at fixed speed
  digitalWrite(IN1, HIGH); // Turn HIGH motor A
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200); // TO set the turning speed to 200 out of possible range 0 to 255
 
  digitalWrite(IN3, HIGH); // turn HIGH motor B
  digitalWrite(IN4, LOW);  // To set the turning speed to 200 out of possible range 0 to 255 
  analogWrite(ENB, 200);
}

void Despacito(){
   // Run the motors on both direction at fixed speed
  digitalWrite(IN1, HIGH); // Turn HIGH motor A
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 125); // TO set the turning speed to 200 out of possible range 0 to 255
 
  digitalWrite(IN3, HIGH); // turn HIGH motor B
  digitalWrite(IN4, LOW);  // To set the turning speed to 200 out of possible range 0 to 255 
  analogWrite(ENB, 125);
  
}

void Slowdown(){
  
  // Run the motors on both direction at fixed speed
  digitalWrite(IN1, HIGH); // Turn HIGH motor A
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 75); // TO set the turning speed to 200 out of possible range 0 to 255
 
  digitalWrite(IN3, HIGH); // turn HIGH motor B
  digitalWrite(IN4, LOW);  // To set the turning speed to 200 out of possible range 0 to 255 
  analogWrite(ENB, 75);

}

void DERECHA()
{
  // Run the motors on both direction at fixed speed
  digitalWrite(IN1, HIGH); // Turn HIGH motor A
  digitalWrite(IN2, LOW);
  analogWrite(ENA,255);
  
  
  digitalWrite(IN3, LOW); // turn HIGH motor B
  digitalWrite(IN4, LOW);  // To set the turning speed to 200 out of possible range 0 to 255 
  //analogWrite(ENB, 255);
}

void IZQUIERDA()
{
  Serial.print(F("Izquierda"));
  // Run the motors on both direction at fixed speed
  digitalWrite(IN1, LOW); // Turn HIGH motor A
  digitalWrite(IN2, LOW);
 // analogWrite(ENA, 255); // TO set the turning speed to 200 out of possible range 0 to 255
  
  digitalWrite(IN3, HIGH); // turn HIGH motor B
  digitalWrite(IN4, LOW);  // To set the turning speed to 200 out of possible range 0 to 255 
  analogWrite(ENB, 255);
  
}

void Atras()
{
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  
}

void Detener()
{
  digitalWrite(IN1, LOW); // Turn the motor off
  digitalWrite(IN2, LOW); // Turn the motor off  
  digitalWrite(IN3, LOW); // Turn the motor off
  digitalWrite(IN4, LOW); // Turn the motor off
}

// converts lat/long from Adafruit degree-minute format to decimal-degrees; requires <math.h> library
double convertDegMinToDecDeg (float degMin) 
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}






