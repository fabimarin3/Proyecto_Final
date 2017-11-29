#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <waypointClass.h>
#include <SparkFun_MAG3110.h>
#include <NewPing.h>
#include <moving_average.h>                       

//variables de inicializacion de GPS y magnetometro
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
MAG3110 mag = MAG3110(); 
TinyGPSPlus gps;
//conexion serial con el GPS
SoftwareSerial ss(RXPin, TXPin);

boolean usingInterrupt = false;
float currentLat,
      currentLong,
      targetLat,
      targetLong,targetLat2,
      targetLong2;
int distanceToTarget,            
    originalDistanceToTarget;    

// Puntos de que tiene que recorrer el carro
#define WAYPOINT_DIST_TOLERANE  5   // tolerance in meters to waypoint; once within this tolerance, will advance to the next waypoint
#define NUMBER_WAYPOINTS 4         // enter the numebr of way points here (will run from 0 to (n-1))
int waypointNumber = -1;            // current waypoint number; will run from 0 to (NUMBER_WAYPOINTS -1); start at -1 and gets initialized during setup()

waypointClass waypointList[NUMBER_WAYPOINTS] = {waypointClass(-83.906532, 9.853824),waypointClass(-83.906532, 9.853707),waypointClass(-83.906562, 9.853549),waypointClass(-83.906410, 9.853492)};

// Variables con las que trabaja el Magnetometro y la navegacion
float targetHeading;              // where we want to go to reach current waypoint
float currentHeading;             // where we are actually facing now
float headingError;               // signed (+/-) difference between targetHeading and currentHeading
#define HEADING_TOLERANCE 5     // tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading

#define TURN_LEFT 1
#define TURN_RIGHT 2
#define TURN_STRAIGHT 99

// Direcciones 
enum directions {left = TURN_LEFT, right = TURN_RIGHT, straight = TURN_STRAIGHT} ;
directions turnDirection = straight;

/* Object avoidance distances (in inches)
#define SAFE_DISTANCE 70
#define TURN_DISTANCE 40
#define STOP_DISTANCE 12 */

// MOTORES

int ENA = 10; // MCU PWM Pin 10 to ENA on L298n Board
int IN1 = 9;  // MCU Digital Pin 9 to IN1 on L298n Board 
int IN2 = 8;  // MCU Digital Pin 8 to IN2 on L298n Board
 
int ENB = 11;  // MCU PWM Pin 5 to ENB on L298n Board
int IN3 = 12;  // MCU Digital pin 7 to IN3 on L298n Board
int IN4 = 13;  // MCU Digital pin 6 to IN4 on L298n Board


/* Ultrasonic ping sensor
#define TRIGGER_PIN  6      
#define ECHO_PIN  7         
#define MAX_DISTANCE_CM 250                        // Maximum distance we want to ping for (in CENTIMETERS). Maximum sensor distance is rated at 400-500cm.  
#define MAX_DISTANCE_IN (MAX_DISTANCE_CM / 2.5)    // same distance, in inches
int sonarDistance;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);     // NewPing setup of pins and maximum distance.
MovingAverage<int, 3> sonarAverage(MAX_DISTANCE_IN);       // moving average of last n pings, initialize at MAX_DISTANCE_IN */

float dist_AD, brng_AD, dist_AB, brng_AB, dist_XT, dist_AT;





void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  mag.initialize(); //Inicializa el magnetometro
  mag.start();      //el sensor se pone en modo activo

  pinMode(ENA, OUTPUT); 
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  smartDelay(1000);
  nextWaypoint();
   
}

void loop()
{ 
   int x, y, z;
  // Calibra el magnetometro
  if(!mag.isCalibrated()) 
  {
    if(!mag.isCalibrating()) 
    {
      Serial.println("Entering calibration mode");
      mag.enterCalMode(); 
    }
    else
    {
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
  
  //obtencion de distancia y curso 
  processGPS();
  // navegacion
  currentHeading = readCompass();    // get our current heading
  calcDesiredTurn();                // calculate how we would optimatally turn, without regard to obstacles
  
  //checkSonar();
  //Asigna el sentido de giro a los motores para desplazarse
  moveAndAvoid(); 
   
  smartDelay(1000);
  
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



// fin del programa 
void loopForever(void)
{
  while (1)
    ;
}

// obtiene las coordenadas del GPS, actualiza la distancia y el curso
void processGPS(void)
{
  currentLat = convertDegMinToDecDeg(gps.location.lat());
  currentLong = convertDegMinToDecDeg(gps.location.lng());
         
  // actualiza el curso y la distancia basado en la nueva posicion
  distanceToWaypoint();
  
  courseToWaypoint();
   
}   

//retorna la distancia en metros
int distanceToWaypoint() 
{
  
  distanceToTarget =  (long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);
  // valida si no se ha alcanzado el punto de destino
  if (distanceToTarget <= WAYPOINT_DIST_TOLERANE){
      nextWaypoint();
    
  return distanceToTarget;
}  // distanceToWaypoint()

//retorna el curso en grados 
float courseToWaypoint() 
{
      float a2=TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);
        targetHeading = a2;
      return targetHeading;
}   // courseToWaypoint()


// actualiza el GPS
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

//actuliza los datos del magnetometro
float readCompass(void)
{
  
  int x, y, z;
  if(mag.dataReady()) {
    mag.readMag(&x, &y, &z);}   
  float heading = atan2(y, x);
  float heading2 = mag.readHeading(); 

  //suma el angulo de declinacion de la tierra
  #define DEC_ANGLE -0.017
  heading += DEC_ANGLE;
  heading2 += DEC_ANGLE;
 
  if(heading < 0)
    heading += 2*PI;
    
  if(heading > 2*PI)
    heading -= 2*PI;

  if(heading2 < 0)
    heading += 2*PI;
    
  if(heading2 > 2*PI)
    heading -= 2*PI;
   
  // convierte de gradianes a grados
  float headingDegrees = heading * 180/M_PI;
  float headingDegrees2 = heading2 * 180/M_PI;
  return ((int)headingDegrees); 
}  

void calcDesiredTurn(void)
{
    // calcula hacia donde hay que girar para llegar al destino
    headingError = targetHeading-currentHeading;
    
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
      
    if (abs(headingError) <= HEADING_TOLERANCE)      // if within tolerance, don't turn
      turnDirection = straight; 
    else if (headingError < 0)
      turnDirection = left;
    else if (headingError > 0)
      turnDirection = right;
    else
      turnDirection = straight;
 
} 


/*void checkSonar(void)
{   
 
  int dist;

  dist = sonar.ping_in();                   // get distqnce in inches from the sensor
  if (dist == 0)                                // if too far to measure, return max distance;
    dist = MAX_DISTANCE_IN;  
  sonarDistance = sonarAverage.add(dist);      // add the new value into moving average, use resulting average
} // checkSonar()*/

// mueve los motores en la direccion indicada
void moveAndAvoid(void)
{
     
    if (sonarDistance >= SAFE_DISTANCE)       // no close objects in front of car
        {
           if (turnDirection == straight)
             AdelanteHS();
           else if(turnDirection == left)
             IZQUIERDA();
           else if(turnDirection == right)
             DERECHA();
             
              
           return;
        }
      
     if (sonarDistance > TURN_DISTANCE && sonarDistance < SAFE_DISTANCE)    
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
     
     if (sonarDistance <  TURN_DISTANCE && sonarDistance > STOP_DISTANCE)         
        {
          Slowdown(); 
          switch (turnDirection)
          {
            case straight:                  
              {
                if (headingError <= 0)
                  IZQUIERDA();
                else
                  DERECHA();
                
                break;
              }
            case left:                         
              {
                DERECHA();    
                break;  
              }
            case right:                       
              {
                IZQUIERDA();
                break;
              }
          } // end SWITCH
          
         return;
        }  


     if (sonarDistance <  STOP_DISTANCE)          
       {
         Detener();
         turnDirection = straight;
         Atras();          
         while (sonarDistance < TURN_DISTANCE)       
           {
              if(gps.location.isValid())
                 processGPS();  
              currentHeading = readCompass();    
              calcDesiredTurn();                      
              checkSonar();
              delay(100);
           } 
         Detener();        
         return;
        } 
     
}   

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




