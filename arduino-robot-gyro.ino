// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include <math.h>

#include <AFMotor.h> //motor shield library

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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

#define TRIG_PIN_FRONT_MID 43 // sensors' pins
#define ECHO_PIN_FRONT_MID 42 
#define TRIG_PIN_FRONT_LEFT 8
#define ECHO_PIN_FRONT_LEFT 7
#define TRIG_PIN_FRONT_RIGHT 4
#define ECHO_PIN_FRONT_RIGHT 2



#define DMP_DATA_DELAY 0.2 //seconds

#define ROBOT_VELOCITY_EQ_TO_MOTOR_SPEED_150 0.28 // 0.32 // 0.24 // (m/s)

#define RADIANS_TO_DEGREES 180/M_PI

#define COURSE_DEVIATION_ERROR 2.5 // degrees
#define COURSE_DEVIATION_ERROR_RAD COURSE_DEVIATION_ERROR*(M_PI/180)

//#define X_DEVIATION_ERROR 0.01 // meters
#define X_DEVIATION_MAX 0.02 // meters
#define X_TURN_DEVIATION 0.02 // meters

#define TRAVEL_DISTANCE 10 // meters

#define BASE_MOTOR_SPEED 150
#define BASE_MOTOR_SPEED_MIN 100
#define BASE_MOTOR_SPEED_MAX 180

#define STATE_ON_ROUTE 0
#define STATE_CORRECT_COURSE 1
#define STATE_NEW_ROUTE 2
#define STATE_AVOID_OBSTACLE 3
#define STATE_DISTANCE_REACHED 4
#define STATE_ACCELERATION 5
#define STATE_MOVE_FORWARD 6
#define STATE_WAIT 7
#define STATE_TURN 8

#define ACCELERATION_TIME 0.5
#define FORWARD_MOVEMENT_TIME 3
#define WAIT_TIME 1

#define DISTANCE_NUMBER 1

#define DISTANCE_FOR_NEW_ROUTE_MIN_1 0.2
#define DISTANCE_FOR_NEW_ROUTE_MIN_2 0.4

#define NEW_ROUTE_MODE__HYPOT_TO_THE_END_POINT 0
#define NEW_ROUTE_MODE__BACK_TO_THE_START_COURSE 1

#define ANGLE_ERROR_RATIO_1 0.0847*(M_PI/180)
#define ANGLE_ERROR_RATIO_2 0.0032*(M_PI/180)
#define ANGLE_ERROR_CHANGE_ERROR_TIME 13.76
#define ANGLE_ERROR_BASE_ERROR 0.1*(M_PI/180)

//#define USE_US_SENSOR


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// set up motors
AF_DCMotor motor1(2,MOTOR12_64KHZ);
AF_DCMotor motor2(1,MOTOR12_8KHZ);
//AF_DCMotor motor3(3,MOTOR34_64KHZ);
//AF_DCMotor motor4(4,MOTOR34_8KHZ);

// set up ultra sonic sensors
#ifdef USE_US_SENSOR
  #define US_SCAN_DELAY 0.5 //seconds
  #define DISTANCE_MIN1 5
  #define DISTANCE_MIN2 15
  #define DISTANCE_MIN3 25
  UltraSonicSensor usSensor_FrontMid(TRIG_PIN_FRONT_MID, ECHO_PIN_FRONT_MID);
#endif


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

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

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
// #define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

uint32_t DMPTimer, timer;
double lastDMPReceptionTimeElapsed;
double dt, dt2;
bool blockTransmission;
double distanceTraveled;
double travelTime;
bool isRobotStopped;

double courseAngle;
double startAngle;
double yaw;
struct Coordinates {
  double x;
  double y;
} coords;
double v;
double travelDistance, travelDistanceOld;
int baseSpeed;
double accelerationTimeElapsed;
double forwardMovementTimeElapsed;
double waitTimeElapsed;
double robotStartTimeElapsed;

char motorsState;
char motorsStateOld;

long distance_FrontMid, distance_FrontLeft, distance_FrontRight, oldDistance;
bool ignoreRoute;

int speedRatio;

int robotState;

int accelerationStage;

int distanceCount;

bool changeRoute;

int newRouteMode;

double correctionAngle, correctionDistance;

double lastUSScanTimeElapsed;


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
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
    dt = (double)(micros() - DMPTimer) / 1000000; // Calculate delta time
    dt2 = (double)(micros() - timer) / 1000000; // Calculate delta time
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
    startAngle = -1000;
    coords.x = 0;
    coords.y = 0;
    v = ROBOT_VELOCITY_EQ_TO_MOTOR_SPEED_150;
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
    if (isRobotStopped) return;
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    
    /*dt = (double)(micros() - DMPTimer) / 1000000; // Calculate delta time
    DMPTimer = micros();
    dt2 = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    lastDMPReceptionTimeElapsed += dt;*/
    if (lastDMPReceptionTimeElapsed > DMP_DATA_DELAY) {
      Serial.println(lastDMPReceptionTimeElapsed);
      while (!mpuInterrupt);
      lastDMPReceptionTimeElapsed = 0;
      blockTransmission = false;
    } else {
      mpu.resetFIFO();
      mpuInterrupt = false;
    }
    if (startAngle == -1000) while (!mpuInterrupt);

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && (fifoCount < packetSize) || blockTransmission) {
        
        dt2 = (double)(micros() - timer) / 1000000; // Calculate delta time
        timer = micros();
        lastDMPReceptionTimeElapsed += dt2;
        if (robotState == STATE_ACCELERATION) accelerationTimeElapsed += dt2;
        if (robotState == STATE_MOVE_FORWARD) forwardMovementTimeElapsed += dt2;
        if (robotState == STATE_WAIT) waitTimeElapsed += dt2;
        #ifdef USE_US_SENSOR
          lastUSScanTimeElapsed += dt2;
          if (lastUSScanTimeElapsed >= US_SCAN_DELAY) {
            lastUSScanTimeElapsed = 0;
            distance_FrontMid = usSensor_FrontMid.scan();
            if (distance_FrontMid <= DISTANCE_MIN3) {
              robotState = STATE_AVOID_OBSTACLE;
            }
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
        
        switch (robotState) {
          case STATE_ON_ROUTE:
            motorsState = 'f';
            break;
          case STATE_CORRECT_COURSE:
            courseAngle += correctionAngle;
            //speedRatio = lround(1.5 * map(lround(abs(courseAngle * RADIANS_TO_DEGREES)), 0, 180, 0, 255));
            //Serial.print("mappedOld: ");Serial.println(speedRatio);
            speedRatio = lround(4 * map(lround(abs(courseAngle * RADIANS_TO_DEGREES)), 0, 180, 0, BASE_MOTOR_SPEED_MAX - BASE_MOTOR_SPEED_MIN));
            Serial.print("mappedNew: ");Serial.println(speedRatio);
            {
              int speed1 = constrain(BASE_MOTOR_SPEED - sign(courseAngle) * speedRatio, BASE_MOTOR_SPEED_MIN, BASE_MOTOR_SPEED_MAX);
              int speed2 = constrain(BASE_MOTOR_SPEED + sign(courseAngle) * speedRatio, BASE_MOTOR_SPEED_MIN, BASE_MOTOR_SPEED_MAX);
              motor1.setSpeed(speed1);
              motor2.setSpeed(speed2);
              Serial.print("motor1: ");Serial.print(speed1);
              Serial.print(" motor2: ");Serial.println(speed2);
            }
            motorsState = 'f';
            robotState = STATE_ON_ROUTE;
            courseAngle -= correctionAngle;
            break;
          case STATE_NEW_ROUTE:
            //if (travelDistance <= 0.5)
            if (newRouteMode == NEW_ROUTE_MODE__HYPOT_TO_THE_END_POINT) {
              travelDistanceOld = travelDistance;
              travelDistance = hypot(coords.x, travelDistance - coords.y);
              if (travelDistance > DISTANCE_FOR_NEW_ROUTE_MIN_1) {
                //startAngle = asin(abs(coords.x) / travelDistance * coords.x) + startAngle * sign(courseAngle);
                startAngle = asin(coords.x / travelDistance) + startAngle;
                //if (startAngle <= -180 || startAngle >= 180) startAngle = -startAngle;
                coords.x = 0;
                coords.y = 0;
                Serial.print("STATE_NEW_ROUTE:   new distance: ");Serial.print(travelDistance);Serial.println(" m");Serial.print(" new angle: ");Serial.print(startAngle);Serial.println(" rad");
              } else if (travelDistance > DISTANCE_FOR_NEW_ROUTE_MIN_2) {
                //travelDistance = hypot(coords.x, (travelDistance - coords.y) * 2);
                startAngle = asin(coords.x / travelDistance) / 2 + startAngle;
                //if (startAngle <= -180 || startAngle >= 180) startAngle = -startAngle;
                coords.x = 0;
                coords.y = 0;
                Serial.print("STATE_NEW_ROUTE:   new distance (smoothed): ");Serial.print(travelDistance);Serial.println(" m");Serial.print(" new angle: ");Serial.print(startAngle);Serial.println(" rad");
              } else {
                Serial.print("STATE_NEW_ROUTE: too short distance: ");Serial.print(travelDistance);Serial.println(" m");
                travelDistance = travelDistanceOld;
                changeRoute = false;
              }
              robotState = STATE_ON_ROUTE;
            } else if (newRouteMode == NEW_ROUTE_MODE__BACK_TO_THE_START_COURSE) {
              correctionAngle = -courseAngle + startAngle;
              correctionDistance = abs(coords.x / sin(courseAngle));
              Serial.print("NEW_ROUTE_MODE__BACK_TO_THE_START_COURSE: correctionAngle: ");Serial.print(correctionAngle);Serial.println(" rad");Serial.print(" correctionDistance: ");Serial.print(correctionDistance);Serial.println(" m");
            }
            break;
          case STATE_AVOID_OBSTACLE:
            #ifdef USE_US_SENSOR
              if (distance_FrontMid <= DISTANCE_MIN3) {
                motorsState = 'r';
                motor1.setSpeed(BASE_MOTOR_SPEED);
                motor2.setSpeed(BASE_MOTOR_SPEED - 50);
                usSensor_FrontMid.printDistanceToSerial();
              } else {
                motor1.setSpeed(BASE_MOTOR_SPEED);
                motor2.setSpeed(BASE_MOTOR_SPEED);
                robotState = STATE_ON_ROUTE;
              }
            #endif
            break;
          case STATE_DISTANCE_REACHED:
            distanceCount++;
            if (distanceCount == DISTANCE_NUMBER) {
              Serial.print("Covered ");Serial.print(TRAVEL_DISTANCE);Serial.println("m. Stopping motors.");
              Serial.print("Travel time: ");Serial.println(travelTime);
              Serial.print("X, Y coords: ");Serial.print(coords.x);Serial.print(" ");Serial.println(coords.y);
              motorsState = 's';
              isRobotStopped = true;
            } else {
              travelDistance = TRAVEL_DISTANCE;
              Serial.print("startAngleOld: ");Serial.println(startAngle);
              startAngle = startAngle - M_PI / 2; //90 / RADIANS_TO_DEGREES;
              if (startAngle <= -180 || startAngle >= 180) startAngle = -startAngle;
              Serial.print("startAngle: ");Serial.println(startAngle);
              coords.x = 0;
              coords.y = 0;
              robotState = STATE_ON_ROUTE; 
            }
            break;
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
              robotState = STATE_NEW_ROUTE;
              lastDMPReceptionTimeElapsed = 0;
            }
            break;
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
        if (robotState == STATE_DISTANCE_REACHED) return;

        
        if ((lastDMPReceptionTimeElapsed > DMP_DATA_DELAY) && (robotState != STATE_ACCELERATION)) {
          Serial.println(lastDMPReceptionTimeElapsed);
          while (!mpuInterrupt) {Serial.println("!mpuInterrupt2");}
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

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

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
            Serial.print("dt\t");
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
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        dt = (double)(micros() - DMPTimer) / 1000000; // Calculate delta time
        DMPTimer = micros();
        travelTime += dt;
        
        yaw = ypr[0];
        if (startAngle == -1000) startAngle = 0; //-yaw;

        // getting rid of gyroscope error
        if (travelTime <= ANGLE_ERROR_CHANGE_ERROR_TIME)
          yaw -= (ANGLE_ERROR_RATIO_1 * travelTime + ANGLE_ERROR_BASE_ERROR);
        else
          yaw += (ANGLE_ERROR_RATIO_2 * travelTime - ANGLE_ERROR_BASE_ERROR - ANGLE_ERROR_RATIO_1 * ANGLE_ERROR_CHANGE_ERROR_TIME);
          
        courseAngle = yaw + startAngle;

        // gyroscope starts to show opposite values when it turns more then 180 grad. left or right along any axis
        // but we need to work with values above 180 grad.
        if (abs(courseAngle) > 180) {
          courseAngle = -sign(courseAngle) * (2 * M_PI - abs(yaw)); 
        }
        
        Serial.print("course: ");Serial.println(courseAngle * RADIANS_TO_DEGREES);

        coords.x += v * sin(courseAngle) * dt;
        coords.y += v * cos(courseAngle) * dt;
        distanceTraveled += v * dt;
        Serial.print("distance: ");Serial.print(distanceTraveled);Serial.print(" ");Serial.print("y: ");Serial.println(coords.y);
        if (coords.y >= travelDistance) {
          robotState = STATE_DISTANCE_REACHED;
        } else if (robotState != STATE_AVOID_OBSTACLE && robotState != STATE_MOVE_FORWARD && robotState != STATE_WAIT &&
                   robotState != STATE_TURN) {
          Serial.print("state: ");Serial.println(robotState);
          /*if (abs(courseAngle) >= 1.57) {
            robotState = STATE_TURN;
          } else */if (abs(coords.x) > X_DEVIATION_MAX && changeRoute) {
            robotState = STATE_NEW_ROUTE;
          } else if (correctionAngle != 0 && coords.x < X_TURN_DEVIATION) {
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

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

// functions

// Move the robot
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

int sign(double value)
{
  if (value >= 0)
    return 1;
  else
    return -1;
}


