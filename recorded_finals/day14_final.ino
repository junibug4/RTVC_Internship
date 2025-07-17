#include <Servo.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>


MPU6050 mpu;
Servo pitchServo;
Servo yawServo;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
#define LED_PIN 13
#define REVERSE  1

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

// Pins
#define PITCH_SERVO_PIN 9
#define YAW_SERVO_PIN 10

// PID and Variables
double pitch, pitchSetpoint, pitchOutput;
double yaw, yawSetpoint, yawOutput;

// Pitch PID tuning
double Kp_pitch = 0.2 , Ki_pitch = 0.3 , Kd_pitch = 0.3;

// Yaw PID tuning (adjust as needed)
double Kp_yaw = 0.2 , Ki_yaw = 0.3 , Kd_yaw = 0.3;

PID pitchPID(&pitch, &pitchOutput, &pitchSetpoint, Kp_pitch, Ki_pitch, Kd_pitch, REVERSE);
PID yawPID(&yaw, &yawOutput, &yawSetpoint, Kp_yaw, Ki_yaw, Kd_yaw, REVERSE);

void setup() {
  Serial.begin(115200);

  pitchServo.attach(PITCH_SERVO_PIN);
  yawServo.attach(YAW_SERVO_PIN);


  pitchSetpoint = 0;
  yawSetpoint = 0;


  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-70, 70);
  pitchPID.SetTunings(Kp_pitch, Ki_pitch, Kd_pitch);

  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-70, 70);
  yawPID.SetTunings(Kp_yaw, Ki_yaw, Kd_yaw);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    mpu.resetFIFO();
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    fifoCount = mpu.getFIFOCount();
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 128) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    return;
  }

  if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  pitch = ypr[1] * 180 / M_PI;
  yaw = ypr[0] * 180 / M_PI;

  pitchPID.Compute();
  yawPID.Compute();

  double pitchServoPos = pitchOutput + 86.5;
  double yawServoPos = yawOutput + 90;  // Centered at 90

  pitchServoPos = constrain(pitchServoPos, 15, 160);
  yawServoPos = constrain(yawServoPos, 15, 160);

  pitchServo.write(pitchServoPos);
  yawServo.write(yawServoPos);


  Serial.print("Pitch = "); Serial.print(pitch);
  Serial.print(" | PitchServo = "); Serial.print(pitchServoPos);
  Serial.print(" || Yaw = "); Serial.print(yaw);
  Serial.print(" | YawServo = "); Serial.println(yawServoPos);

  delay(8);
}
