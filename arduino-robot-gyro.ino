// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include <math.h>
#include <NewPing.h> // ultrasonic library
#include <AFMotor.h> // motor shield library

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// ================================================================
// ===                     CLASS DEFINITION                     ===
// ================================================================
// a class used to receive data from ultrasonic sensors (currently not used because of NewPing library)
class UltraSonicSensor
{
  private:
    long duration;
    long distance;
  public:
    uint8_t trigPin;
    uint8_t echoPin;
    bool enableSerialPrint;
  public:
    UltraSonicSensor(uint8_t trigPin, uint8_t echoPin);
    long scan();
    void printDistanceToSerial();
    long getDistance() {return distance;};
    long getDuration() {return duration;};
};

UltraSonicSensor::UltraSonicSensor(uint8_t tPin, uint8_t ePin) : duration(0), distance(0), enableSerialPrint(true)
{
  trigPin = tPin;
  echoPin = ePin;
  pinMode(trigPin, OUTPUT);// set the trig pin to output (Send sound waves)
  pinMode(echoPin, INPUT);// set the echo pin to input (recieve sound waves)
}

long UltraSonicSensor::scan()
{
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2); // delays are required for a succesful sensor operation.
  digitalWrite(trigPin, HIGH);
  
  delayMicroseconds(10); //this delay is required as well!
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  delay(10);
  distance = (duration / 2) / 29.1;// convert the distance to centimeters.
  if (distance == 0) {
    //distance = this->scan();
  }
  return distance;
}

void UltraSonicSensor::printDistanceToSerial()
{
  if (!enableSerialPrint) return;
  Serial.print("distance: ");
  Serial.print(distance);
  Serial.println( " CM!"); // print out the distance in centimeters.
  Serial.println("-------------------------");  
}

// ================================================================
// ===                         GLOBALS                          ===
// ================================================================

// sensors' pins
#define TRIG_PIN_FRONT_MID 46
#define ECHO_PIN_FRONT_MID 47 
#define TRIG_PIN_FRONT_LEFT 8
#define ECHO_PIN_FRONT_LEFT 7
#define TRIG_PIN_FRONT_RIGHT 4
#define ECHO_PIN_FRONT_RIGHT 2



#define DMP_DATA_DELAY 0.2                         // the delay between data packages from gyroscope (seconds)

#define ROBOT_VELOCITY_EQ_TO_BASE_MOTOR_SPEED 0.31 // experimentally calculated robot's velocity (m/s)

#define RADIANS_TO_DEGREES 180/M_PI

//#define COURSE_DEVIATION_ERROR 2.5 // degrees
//#define COURSE_DEVIATION_ERROR_RAD COURSE_DEVIATION_ERROR*(M_PI/180)

//#define X_DEVIATION_ERROR 0.01 // meters
#define X_DEVIATION_MAX 0.02                      // maximum x axis deviation allowed for the robot (meters)
#define X_TURN_DEVIATION 0.02                     // maximum x axis deviation allowed for the robot (used for the TURN_STATE) (meters)

#define TRAVEL_DISTANCE 10                        // the distance the robot have to travel (meters)

// the base speed used on motors
#define BASE_MOTOR_SPEED 100 
#define BASE_MOTOR_SPEED_MIN 50
#define BASE_MOTOR_SPEED_MAX 130

// robot's states
#define STATE_ON_ROUTE 0                          // just moving
#define STATE_CORRECT_COURSE 1                    // course correction base on gyroscope values
#define STATE_NEW_ROUTE 2                         // new route calculating, triggered when coords.x > X_DEVIATION_MAX
#define STATE_AVOID_OBSTACLE 3                    // obstacles avoidance
#define STATE_DISTANCE_REACHED 4                  // the robot traveled TRAVEL_DISTANCE meters
#define STATE_ACCELERATION 5                      // used to simulate acceleration
#define STATE_MOVE_FORWARD 6                      // forward moving without course correction
#define STATE_WAIT 7                              // just wait for a given time
#define STATE_TURN 8                              // not implemented yet

// timers (seconds)
#define ACCELERATION_TIME 0.5 
#define FORWARD_MOVEMENT_TIME 3
#define WAIT_TIME 1

// the number of distances to travel
#define DISTANCE_NUMBER 1

// used for more smooth route ending
#define DISTANCE_FOR_NEW_ROUTE_MIN_1 0.2
#define DISTANCE_FOR_NEW_ROUTE_MIN_2 0.4

// the ways to build new route when STATE_NEW_ROUTE triggered
#define NEW_ROUTE_MODE__HYPOT_TO_THE_END_POINT 0   // in this mode the robot will build hypotenuse from the current destination to the end destination
#define NEW_ROUTE_MODE__BACK_TO_THE_START_COURSE 1 // in this mode the robot will be trying to get back to the start course

// gyroscope errors
#define ANGLE_ERROR_RATIO_1 -0.0847*(M_PI/180)
#define ANGLE_ERROR_RATIO_2 -0.0032*(M_PI/180)
#define ANGLE_ERROR_CHANGE_ERROR_TIME 13.76
#define ANGLE_ERROR_BASE_ERROR 0.1*(M_PI/180)

#define SPEED_COEFFICIENT 4                       // experimentally found. Used to calculate speedRatio 

//#define USE_US_SENSOR                           // uncomment to turn on ultrasonic sensors

// set up ultra sonic sensors
#ifdef USE_US_SENSOR
  #define US_SCAN_DELAY 0.2 // the delay between ultrasonic scans (seconds)
  // minimum distances to obstacles from which the robot changes its behaviour
  #define DISTANCE_MIN1 5
  #define DISTANCE_MIN2 15
  #define DISTANCE_MIN3 25
  //UltraSonicSensor usSensor_FrontMid(TRIG_PIN_FRONT_MID, ECHO_PIN_FRONT_MID);
  NewPing usSensor_FrontMid(TRIG_PIN_FRONT_MID, ECHO_PIN_FRONT_MID, 400);
#endif

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;                                      // gyroscope library variable
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

// set up motors
AF_DCMotor motor1(2,MOTOR12_64KHZ);
AF_DCMotor motor2(1,MOTOR12_8KHZ);
//AF_DCMotor motor3(3,MOTOR34_64KHZ);
//AF_DCMotor motor4(4,MOTOR34_8KHZ);


bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// timers
uint32_t DMPTimer;                 // used only for DMP delays
uint32_t timer                     // used for any other program needs;
double lastDMPReceptionTimeElapsed // time elapsed since last DMP reception;
double accelerationTimeElapsed;    // time elapsed while in the STATE_ACCELERATION;
double forwardMovementTimeElapsed; // time elapsed while in the STATE_MOVE_FORWARD;
double waitTimeElapsed;            // time elapsed while in the STATE_WAIT;
double robotStartTimeElapsed;      // not implemented
#ifdef USE_US_SENSOR
  double lastUSScanTimeElapsed;    // time elapsed since last ultrasonic scan;
#endif

double dt;                         // delta time between micros() and DMPTimer
double dt2;                        // delta time between micros() and timer

bool blockTransmission;            // = true if lastDMPReceptionTimeElapsed == DMP_DATA_DELAY

double distanceTraveled;           // the distance the robot has traveled at the current moment
double travelTime;                 // the time spent in travel

bool isRobotStopped;               // = true if robot has reached the destination

double courseAngle;                // current course
double yaw;
double deltaAngle;                 // courseAngle = yaw + deltaAngle. deltaAngle != 0 when the route has been changed so we need to add it to gyroscope values
                                   // in order to get the right courseAngle.

// robot's coordinates
struct Coordinates {
  double x;
  double y;
} coords;

double v;                          // robot's velocity. Currently it gets its value from ROBOT_VELOCITY_EQ_TO_BASE_MOTOR_SPEED global while there are no
                                   // tools to get velocity in real time
double travelDistance;             // the remaining distance
double travelDistanceOld;          // the old remaining distance

int baseSpeed;                     // motors' base speed used in STATE_ACCELERATION


char motorsState;                  // used to define an argument for AF_DCMotor::run function.
                                   // 'r' == RIGHT, 'l' == LEFT, 'f' == FORWARD, 'b' == BACKWARD, 's' == RELEASE
char motorsStateOld;

// distances to obstacles received from ultrasonic sensors
long distance_FrontMid, distance_FrontLeft, distance_FrontRight;

//long oldDistance;

bool ignoreRoute;                  // not used currently

int speedRatio;                    // the value of this variable depends on courseAngle and used to turn the robot left or right

int robotState;                    // the of the robot. See STATE_ globals

int accelerationStage;             // not used currently

int distanceCount;                 // the number of distances traveled

bool changeRoute;                  // = false if we need to ignore STATE_NEW_ROUTE

int newRouteMode;                  // stores the value of one of the NEW_ROUTE_MODE__ globals

// needed for NEW_ROUTE_MODE__BACK_TO_THE_START_COURSE
double correctionAngle;           
double correctionDistance;




// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART DMPTimer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    /*mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip*/
    
    mpu.setXGyroOffset(75);
    mpu.setYGyroOffset(-31);
    mpu.setZGyroOffset(18);
    mpu.setXAccelOffset(1278);
    mpu.setYAccelOffset(-429);
    mpu.setZAccelOffset(1598);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for the first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    motor1.setSpeed(BASE_MOTOR_SPEED); 
    motor2.setSpeed(BASE_MOTOR_SPEED);
    lastDMPReceptionTimeElapsed = 0;
    DMPTimer = micros();
    timer = DMPTimer;
    dt = (double)(micros() - DMPTimer) / 1000000; // calculate delta time
    dt2 = (double)(micros() - timer) / 1000000;   // calculate delta time
    lastDMPReceptionTimeElapsed += dt;
    robotStartTimeElapsed += dt;
    blockTransmission = false;
    distanceTraveled = 0;
    travelTime = 0;
    isRobotStopped = false;
    baseSpeed = 0;
    accelerationTimeElapsed = 0;
    forwardMovementTimeElapsed = 0;
    waitTimeElapsed = 0;

    courseAngle = 0;
    deltaAngle = -1000;                           // set to some random big value so we know when it's the first run of the sketch
    coords.x = 0;
    coords.y = 0;
    v = ROBOT_VELOCITY_EQ_TO_BASE_MOTOR_SPEED;
    travelDistance = TRAVEL_DISTANCE;

    motorsState = 'f';
    motorsStateOld = '\0';

    ignoreRoute = false;

    robotState = STATE_ON_ROUTE;
    //robotState = STATE_MOVE_FORWARD;  

    accelerationStage = 1;

    distanceCount = 0;

    changeRoute = true;

    newRouteMode = NEW_ROUTE_MODE__HYPOT_TO_THE_END_POINT;
    //newRouteMode = NEW_ROUTE_MODE__BACK_TO_THE_START_COURSE;

    correctionAngle = 0;

    lastUSScanTimeElapsed = 0;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if robot has reached the destination then don't do anything
    if (isRobotStopped) return;
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    /*dt = (double)(micros() - DMPTimer) / 1000000; // Calculate delta time
    DMPTimer = micros();
    dt2 = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    lastDMPReceptionTimeElapsed += dt;*/

    
    if (lastDMPReceptionTimeElapsed > DMP_DATA_DELAY) {
      // wait for MPU is ready to transmit data
      while (!mpuInterrupt);
      lastDMPReceptionTimeElapsed = 0;
      blockTransmission = false;
    } else {
      // reset FIFO so it won't overflow
      mpu.resetFIFO();
      mpuInterrupt = false;
    }
    
    // we need to receive data from gyroscope at first so wait while mpuInterrupt will become true
    if (deltaAngle == -1000) while (!mpuInterrupt);

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && (fifoCount < packetSize) || blockTransmission) {
        // calculate delta time
        dt2 = (double)(micros() - timer) / 1000000;
        timer = micros();
        lastDMPReceptionTimeElapsed += dt2;
        // refersh timers
        if (robotState == STATE_ACCELERATION) accelerationTimeElapsed += dt2;
        if (robotState == STATE_MOVE_FORWARD) forwardMovementTimeElapsed += dt2;
        if (robotState == STATE_WAIT) waitTimeElapsed += dt2;

        // scan for obstacles
        #ifdef USE_US_SENSOR
          lastUSScanTimeElapsed += dt2;
          if (lastUSScanTimeElapsed >= US_SCAN_DELAY) {
            lastUSScanTimeElapsed = 0;
            //mpu.setSleepEnabled(true);
            // send a ping and call echoCheck to test if ping is complete
            usSensor_FrontMid.ping_timer(echoCheck);
            /*delay(50);
            distance_FrontMid = usSensor_FrontMid.ping_cm(); //usSensor_FrontMid.scan();
            if (distance_FrontMid <= DISTANCE_MIN3) {
              robotState = STATE_AVOID_OBSTACLE;
              Serial.print("usSensor_FrontMid: ");Serial.print(distance_FrontMid);Serial.println(" CM");
            }*/
            /*distance_FrontMid = usSensor_FrontMid.ping_cm(); //usSensor_FrontMid.scan();
            if (distance_FrontMid <= DISTANCE_MIN3) {
              robotState = STATE_AVOID_OBSTACLE;
            }*/
            //mpu.setSleepEnabled(false);
          }
        #endif

        /*if (robotState != STATE_MOVE_FORWARD && robotState != STATE_WAIT) {
          distance_FrontMid = usSensor_FrontMid.scan();
          if (distance_FrontMid <= DISTANCE_MIN3) {
            robotState = STATE_AVOID_OBSTACLE;
          } else if (robotState == STATE_AVOID_OBSTACLE) {
            robotState = STATE_NEW_ROUTE;
          }
        }*/

        // the handle of the STATE_ globals
        switch (robotState) {
          case STATE_ON_ROUTE:
            motorsState = 'f';
            break;
          case STATE_CORRECT_COURSE:
            courseAngle += correctionAngle;
            //speedRatio = lround(1.5 * map(lround(abs(courseAngle * RADIANS_TO_DEGREES)), 0, 180, 0, 255));
            //Serial.print("mappedOld: ");Serial.println(speedRatio);
            // calculate speedRatio by bringing the courseAngle to the scale of 0 to BASE_MOTOR_SPEED_MAX - BASE_MOTOR_SPEED_MIN. We consider courseAngle is in the scale of 0 to 180. 
            // And then multiply it by SPEED_COEFFICIENT. Reference to the map() function: https://www.arduino.cc/en/Reference/Map
            speedRatio = lround(SPEED_COEFFICIENT * map(lround(abs(courseAngle * RADIANS_TO_DEGREES)), 0, 180, 0, BASE_MOTOR_SPEED_MAX - BASE_MOTOR_SPEED_MIN));
            Serial.print("mappedNew: ");Serial.println(speedRatio);
            // scopes needed to delete speed1 and speed2
            {
              // constrain the speed because if it's too slow wheels can stop moving and we don't want the robot moves too fast because it's difficult to handle.
              // We add or subtract speedRatio to the BASE_MOTOR_SPEED which depends on the sign of the courseAngle. Thus it will turn left or right base on the courseAngle
              int speed1 = constrain(BASE_MOTOR_SPEED - sign(courseAngle) * speedRatio, BASE_MOTOR_SPEED_MIN, BASE_MOTOR_SPEED_MAX);
              int speed2 = constrain(BASE_MOTOR_SPEED + sign(courseAngle) * speedRatio, BASE_MOTOR_SPEED_MIN, BASE_MOTOR_SPEED_MAX);
              motor1.setSpeed(speed1);
              motor2.setSpeed(speed2);
              Serial.print("motor1: ");Serial.print(speed1);
              Serial.print(" motor2: ");Serial.println(speed2);
            }
            motorsState = 'f';
            // back to the STATE_ON_ROUTE
            robotState = STATE_ON_ROUTE;
            courseAngle -= correctionAngle;
            break;
          case STATE_NEW_ROUTE:
            //if (travelDistance <= 0.5)
            
            if (newRouteMode == NEW_ROUTE_MODE__HYPOT_TO_THE_END_POINT) {
              // save old distance
              travelDistanceOld = travelDistance;
              // calculate new distance
              travelDistance = hypot(coords.x, travelDistance - coords.y);
              if (travelDistance > DISTANCE_FOR_NEW_ROUTE_MIN_1) {
                //deltaAngle = asin(abs(coords.x) / travelDistance * coords.x) + deltaAngle * sign(courseAngle);
                
                // calculate deltaAngle. It is the angle between the hypotenuse and the cathetus along the y axis
                deltaAngle = asin(coords.x / travelDistance) + deltaAngle;
                
                //if (deltaAngle <= -180 || deltaAngle >= 180) deltaAngle = -deltaAngle;
                coords.x = 0;
                coords.y = 0;
                Serial.print("STATE_NEW_ROUTE:   new distance: ");Serial.print(travelDistance);Serial.println(" m");Serial.print(" new angle: ");Serial.print(deltaAngle);Serial.println(" rad");
              } else if (travelDistance > DISTANCE_FOR_NEW_ROUTE_MIN_2) {                
                // make the angle 2 times less so the robot will end its route more smooth 
                deltaAngle = asin(coords.x / travelDistance) / 2 + deltaAngle;
                
                //if (deltaAngle <= -180 || deltaAngle >= 180) deltaAngle = -deltaAngle;
                
                coords.x = 0;
                coords.y = 0;
                Serial.print("STATE_NEW_ROUTE:   new distance (smoothed): ");Serial.print(travelDistance);Serial.println(" m");Serial.print(" new angle: ");Serial.print(deltaAngle);Serial.println(" rad");
              } else {
                Serial.print("STATE_NEW_ROUTE: too short distance: ");Serial.print(travelDistance);Serial.println(" m");
                // if new distance is too short then follow the old route
                travelDistance = travelDistanceOld;
                changeRoute = false;
              }
              robotState = STATE_ON_ROUTE;
            // this mode doesn't work fine yet  
            } else if (newRouteMode == NEW_ROUTE_MODE__BACK_TO_THE_START_COURSE) {
              correctionAngle = -courseAngle + deltaAngle;
              correctionDistance = abs(coords.x / sin(courseAngle));
              Serial.print("NEW_ROUTE_MODE__BACK_TO_THE_START_COURSE: correctionAngle: ");Serial.print(correctionAngle);Serial.println(" rad");Serial.print(" correctionDistance: ");Serial.print(correctionDistance);Serial.println(" m");
            }
            break;
          case STATE_AVOID_OBSTACLE:
            #ifdef USE_US_SENSOR
              // an obstacle ahead
              if (distance_FrontMid <= DISTANCE_MIN3) {
                // it's just the turn right yet
                motorsState = 'r';
                motor1.setSpeed(BASE_MOTOR_SPEED);
                motor2.setSpeed(BASE_MOTOR_SPEED - 50);
                //Serial.print("usSensor_FrontMid: ");Serial.print(distance_FrontMid);Serial.println("CM");
                //usSensor_FrontMid.printDistanceToSerial();
              } else {
                motor1.setSpeed(BASE_MOTOR_SPEED);
                motor2.setSpeed(BASE_MOTOR_SPEED);
                robotState = STATE_ON_ROUTE;
              }
            #endif
            break;
          case STATE_DISTANCE_REACHED:
            distanceCount++;
            // all distances are reached
            if (distanceCount == DISTANCE_NUMBER) {
              Serial.print("Covered ");Serial.print(TRAVEL_DISTANCE);Serial.println("m. Stopping motors.");
              Serial.print("Travel time: ");Serial.println(travelTime);
              Serial.print("X, Y coords: ");Serial.print(coords.x);Serial.print(" ");Serial.println(coords.y);
              motorsState = 's';
              isRobotStopped = true;
            } else {
              // start new distance
              travelDistance = TRAVEL_DISTANCE;
              Serial.print("deltaAngleOld: ");Serial.println(deltaAngle);
              // turn right on 90 degrees (a turn angle will be set via global in future)
              deltaAngle = deltaAngle - M_PI / 2; //90 / RADIANS_TO_DEGREES;
              //if (deltaAngle <= -180 || deltaAngle >= 180) deltaAngle = -deltaAngle;
              Serial.print("deltaAngle: ");Serial.println(deltaAngle);
              coords.x = 0;
              coords.y = 0;
              robotState = STATE_ON_ROUTE; 
            }
            break;
          // not completed yet
          case STATE_ACCELERATION:
            baseSpeed = map(lround(accelerationTimeElapsed * 100), 0, ACCELERATION_TIME * 100, 0, 150);
            motor1.setSpeed(baseSpeed);
            motor2.setSpeed(baseSpeed);
            Serial.print("timer: ");Serial.println(accelerationTimeElapsed);
            Serial.print("baseSpeed: ");Serial.println(baseSpeed);
            if (baseSpeed >= BASE_MOTOR_SPEED) {
              motor1.setSpeed(baseSpeed);
              motor2.setSpeed(baseSpeed);
              robotState = STATE_ON_ROUTE;
              lastDMPReceptionTimeElapsed = 0;
            }
            break;
          case STATE_MOVE_FORWARD:
            Serial.println("STATE_MOVE_FORWARD");
            motorsState = 'f';
            Serial.print("forwardMovementTimeElapsed: ");Serial.print(forwardMovementTimeElapsed);Serial.print("dt2: ");Serial.println(dt2);
            if (forwardMovementTimeElapsed > FORWARD_MOVEMENT_TIME) {
              robotState = STATE_WAIT;
              forwardMovementTimeElapsed = 0;
              motorsState = 's';
            }
            break;
          case STATE_WAIT:
            Serial.println("STATE_WAIT");
            motorsState = 's';
            if (waitTimeElapsed > WAIT_TIME) {
              waitTimeElapsed = 0;
              /*if (abs(courseAngle) >= 1.57) {
                robotState = STATE_TURN;
              } else {
                robotState = STATE_NEW_ROUTE;
              }*/
              
              // build new route
              robotState = STATE_NEW_ROUTE;
              lastDMPReceptionTimeElapsed = 0;
            }
            break;
          // not completed yet
          case STATE_TURN:
            Serial.println("STATE_TURN");
            if (abs(courseAngle) < 0.17) {
              motorsState = 's';
              robotState = STATE_NEW_ROUTE;
            } else {
              motor1.setSpeed(courseAngle < 0 ? BASE_MOTOR_SPEED - 10 : 80);
              motor2.setSpeed(courseAngle > 0 ? BASE_MOTOR_SPEED - 10 : 80);
              motorsState = courseAngle < 0 ? 'r' : 'l';
            }
            break;
          default:
            break;
        }
        
        moveRobot(motorsState);
        //if (robotState == STATE_DISTANCE_REACHED) return;
        if (isRobotStopped) return;

        
        if ((lastDMPReceptionTimeElapsed > DMP_DATA_DELAY) && (robotState != STATE_ACCELERATION)) {
          while (!mpuInterrupt) {/*Serial.println("!mpuInterrupt2");*/}
          lastDMPReceptionTimeElapsed = 0;
          blockTransmission = false;
        } else {
          mpu.resetFIFO();
          mpuInterrupt = false;
        }
    }

    blockTransmission = true;
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {Serial.print("while (fifoCount < packetSize) ");Serial.println(fifoCount); fifoCount = mpu.getFIFOCount();}

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*Serial.print("dt\t");
            Serial.print(dt);
            Serial.print("\t");
            Serial.print("elapsed\t");
            Serial.print(lastDMPReceptionTimeElapsed);
            Serial.print("\t");
            Serial.print("DMPTimer\t");
            Serial.print(DMPTimer);
            Serial.print("\t");
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif

        dt = (double)(micros() - DMPTimer) / 1000000;
        DMPTimer = micros();
        travelTime += dt;

        // get yaw (gyroscope z axis value)
        yaw = ypr[0];
        if (deltaAngle == -1000) deltaAngle = 0; //-yaw;

        // getting rid of gyroscope errors
        if (travelTime <= ANGLE_ERROR_CHANGE_ERROR_TIME)
          yaw -= (ANGLE_ERROR_RATIO_1 * travelTime + ANGLE_ERROR_BASE_ERROR);
        else
          yaw += (ANGLE_ERROR_RATIO_2 * travelTime - ANGLE_ERROR_BASE_ERROR - ANGLE_ERROR_RATIO_1 * ANGLE_ERROR_CHANGE_ERROR_TIME);
          
        courseAngle = yaw + deltaAngle;

        // the gyroscope starts to show opposite values when it turns more then 180 degrees left or right around z axis
        // but we need to work with values above 180 degrees
        if (abs(courseAngle) > 180) {
          courseAngle = -sign(courseAngle) * (2 * M_PI - abs(yaw)); 
        }
        
        Serial.print("course: ");Serial.println(courseAngle * RADIANS_TO_DEGREES);

        coords.x += v * sin(courseAngle) * dt;
        coords.y += v * cos(courseAngle) * dt;
        distanceTraveled += v * dt;
        Serial.print("distance: ");Serial.print(distanceTraveled);Serial.print(" ");Serial.print("y: ");Serial.println(coords.y);
        // the distance is reached when the robot has traveled TRAVEL_DISTANCE meters along y axis
        if (coords.y >= travelDistance) {
          robotState = STATE_DISTANCE_REACHED;
        // in the states listed in the condition we don't need to correct course
        } else if (robotState != STATE_AVOID_OBSTACLE && robotState != STATE_MOVE_FORWARD && robotState != STATE_WAIT &&
                   robotState != STATE_TURN) {
          Serial.print("state: ");Serial.println(robotState);
          /*if (abs(courseAngle) >= 1.57) {
            robotState = STATE_TURN;
          } else */
          if (abs(coords.x) > X_DEVIATION_MAX && changeRoute) {
            robotState = STATE_NEW_ROUTE;
          } else if (correctionAngle != 0 && coords.x < X_TURN_DEVIATION) {
            // the correctionAngle no more needed
            correctionAngle = 0;
            correctionDistance = 0;
            robotState = STATE_CORRECT_COURSE;
          } else {
            robotState = STATE_CORRECT_COURSE;
            //robotState = STATE_ON_ROUTE;
          }
        }
          /*if (abs(courseAngle) <= COURSE_DEVIATION_ERROR_RAD) {
            motorsState = 'f';
            motor1.setSpeed(150);
            motor2.setSpeed(150);
          } else if (courseAngle < 0) {
            motorsState = 'r';
            motor1.setSpeed(150);
            motor2.setSpeed(80);
          } else {
            motorsState = 'l';
            motor1.setSpeed(80);
            motor2.setSpeed(150);
          }*/
        
        Serial.print("X, Y coords: ");Serial.print(coords.x);Serial.print(" ");Serial.println(coords.y);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

// functions

// moves the robot
void moveRobot(char motorsState)
{
  if (motorsState == motorsStateOld) return;
  switch (motorsState) {
    case 'f':
      //Serial.println("forward");
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      delay (10);
      break;
    case 'b':
      Serial.println("backward");
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      delay(10);
      break;
    case 'l':
      Serial.println("left");
      //motor1.run(BACKWARD);
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      delay(10);
      break;
    case 'r':
      Serial.println("right");
      motor1.run(FORWARD);
      //motor2.run(BACKWARD);
      motor2.run(FORWARD);
      delay(10);
      break;
    case 's':
      Serial.println("stop");
      motor1.run(RELEASE); 
      motor2.run(RELEASE);
      delay(10);
      break;
    default:
      break;
  }
  motorsStateOld = motorsState;
}

// finds the sign of the value
int sign(double value)
{
  if (value >= 0)
    return 1;
  else
    return -1;
}

// checks the ultrasonic ping status
#ifdef USE_US_SENSOR
  void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
    // Don't do anything here!
    if (usSensor_FrontMid.check_timer()) { // This is how you check to see if the ping was received.
      // Here's where you can add code.
      distance_FrontMid = usSensor_FrontMid.ping_cm(); //usSensor_FrontMid.scan();
      if (distance_FrontMid <= DISTANCE_MIN3) {
        robotState = STATE_AVOID_OBSTACLE;
        Serial.print("usSensor_FrontMid: ");Serial.print(distance_FrontMid);Serial.println(" CM");
      }
    }
    // Don't do anything here!
  }
#endif

