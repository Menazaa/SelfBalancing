/********** PID **********/
#include <PID_v1.h>

double kP = 100;
double kI = 12;
double kD = 8;

double setpoint, input, output;   // PID variables
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // PID setup


/********** Bluetooth **********/

// include Bluetooth Library
#include "BluetoothSerial.h"

// Create Bluetooth Instance
BluetoothSerial SerialBT;

// Create message handling variables
String message =  "";
char incomingChar;
float angleV = 3, turnV = 0; // values from remote

/********** Line Follower **********/

char lineFollowerMode =0;

/********** Line Follower **********/

const char IR1_Pin = 36;
const char IR2_Pin = 39;

/********** Arm **********/

// #include <Servo.h>
// const int baseLinkServoPin = 0;
// const int grabberServoPin = 0;
// const int grabberLinkServoPin = 0;

// Servo baseLinkServo;
// Servo grabberServo;
// Servo grabberLinkServo;

// int angle1 = 0;
// int angle2 = 0;
// int angle3 = 0;
// int pos1 = 0;
// int pos2 = 0;
// int pos3 = 0;

/********** Ultrasonic **********/

const int trigPin1 = 0;
const int echoPin1 = 0;

/********** L298N **********/
#include <L298N.h>

// Pin definition
const unsigned int EN_A = 4;
const unsigned int IN1_A = 16;
const unsigned int IN2_A = 17;

const unsigned int IN1_B = 18;
const unsigned int IN2_B = 5;
const unsigned int EN_B = 19;

// Create motor instances
L298N rightMotor(EN_A, IN1_A, IN2_A);
L298N leftMotor(EN_B, IN1_B, IN2_B);

// motor speeds
int speedLeft = 0;
int speedRight = 0;

/********** MPU **********/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


// INTERRUP PIN
const unsigned int MPU_INTERRUPT_PIN = 13;

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

float pitch;
long velocity;


int IMUdataReady = 0;
volatile bool mpuInterrupt = false;

/********** SETUP **********/
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(MPU_INTERRUPT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

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
    
    // PID Setuo
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255,255);
    pid.SetSampleTime(10);
    

    // Bluetooth Setup
     SerialBT.begin("Zeko"); //Bluetooth device name


    // Arm Sevos Setup
    // baseLinkServo.attach(baseLinkServoPin);
    // grabberServo.attach(grabberServoPin);
    // grabberLinkServo.attach(grabberLinkServoPin);

    // Ultrasonic Setup
    pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin1, INPUT); // Sets the echoPin as an Input

    // IR Setup
    pinMode(IR1_Pin, INPUT); 
    pinMode(IR2_Pin, INPUT);
}

void loop() {
  

  // only read remote data if remote is available from Bluetooth, otherwise balance normally


  if (SerialBT.available()) {
      incomingChar = SerialBT.read();
      if(incomingChar !='\n' && incomingChar != '/'){
        message +=  String(incomingChar);
      }else if(incomingChar == '/'){
          angleV =  message.toFloat();
          message = "";
      }else{
          turnV =  message.toFloat();
          message = "";
        }
  }
  



  if(lineFollowerMode){
    LineFollower(10);
  }

  // Arm Servo Conrol PWM




  if (IMUdataReady == 1) {
    readAngles();
  }

  pitch = -(ypr[1] * 180/M_PI); // adjust to degrees

  if (abs(turnV) < 15) { // turnV threshold
    turnV = 0;
  }

  if (abs(angleV) < .17) { // angleV threshold
    angleV = 0;
  }

  // PID vars
  setpoint =  angleV; 
  input = pitch;

  pid.Compute();

  // set motor speed with adjusted turn values
  speedLeft = output -  turnV;
  speedRight = output + turnV;
  
  if (pitch > 50 || pitch < -50) { // angle threshold
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0); 
  } else {
    leftMotor.setSpeed(abs(speedLeft));
    rightMotor.setSpeed(abs(speedRight));
  }

  // move motors
  if (speedLeft < 0) { 
    leftMotor.forward();
  } else {
    leftMotor.backward();
  }
  
  if (speedRight < 0) { 
    rightMotor.forward();
  } else {
    rightMotor.backward();
  }

  // print some control info
  Serial.print("pitch: ");
  Serial.print(pitch);
  Serial.print(" , angleV: ");
  Serial.print(angleV);
  Serial.print(" , turnV: ");
  Serial.println(turnV);


}