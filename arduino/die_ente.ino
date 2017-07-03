#include "MPU9250.h"
#include "quaternionFilters.h"

#include <SPI.h>
#include <Firmata.h>
#include <Boards.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define STATUS_LED A0

#define DEG_TO_RAD 0.0174533
#define RAD_TO_DEG 57.2957795
#define CAL_COUNTS 50

struct vec3 {
  float x;
  float y;
  float z;
};

struct vec3e {
  float pitch;
  float yaw;
  float roll;
};

struct motors {
  float fr;
  float fl;
  float rl;
  float rr;
};

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

MPU9250 imu;

long last;
long last_intr;

float yaw_offset;
float pitch_offset;
float roll_offset;

float x_offset;
float y_offset;
float z_offset;

float dt = 0;

SoftwareSerial mySerial(2, 3); // RX, TX

//Max safe speed is 50%
void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  Serial.begin(230400);
  mySerial.begin(9600);
  Serial.println("Starting...");
  Wire.begin();

  esc1.attach(6);
  esc2.attach(9);
  esc3.attach(10);
  esc4.attach(11);
  esc1.writeMicroseconds(1060);
  esc2.writeMicroseconds(1060);
  esc3.writeMicroseconds(1060);
  esc4.writeMicroseconds(1060);

  // Setup IMU
  imu.initMPU9250();
  byte id = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (id != 0x71) {
    Serial.println("Failed to connect to MPU9250");
    return;
  }
  Serial.println("Connected to IMU");
  imu.MPU9250SelfTest(imu.SelfTest);
  imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
  imu.initAK8963(imu.magCalibration);
  Serial.println("Calibrating...");
  int counts = 0;
  float yawAvr[CAL_COUNTS];
  float pitchAvr[CAL_COUNTS];
  float rollAvr[CAL_COUNTS];

  bool ledState = false;
  while (counts < CAL_COUNTS) {
    if (!ledState) {
      digitalWrite(STATUS_LED, HIGH);
      ledState = true;
    }
    else {
      digitalWrite(STATUS_LED, LOW);
      ledState = false;
    }
    
    imu.readGyroData(imu.gyroCount);
    imu.getGres();
    pitchAvr[counts] = (float) imu.gyroCount[0] * imu.gRes;
    rollAvr[counts] = (float) imu.gyroCount[1] * imu.gRes;
    yawAvr[counts] = (float) imu.gyroCount[2] * imu.gRes;
    
    counts++;
    delay(100);
  }
  float yaw_sum = 0;
  float roll_sum = 0;
  float pitch_sum = 0;
  
  float x_sum = 0;
  float y_sum = 0;
  float z_sum = 0;
  for (int i = 0; i < CAL_COUNTS; i++) {
    yaw_sum += yawAvr[i];
    pitch_sum += pitchAvr[i];
    roll_sum += rollAvr[i];
  }
  yaw_offset = yaw_sum / CAL_COUNTS;
  pitch_offset = pitch_sum / CAL_COUNTS;
  roll_offset = roll_sum / CAL_COUNTS;

  Serial.println("READY");
  digitalWrite(STATUS_LED, HIGH);
  last = millis();
  last_intr = millis();
  imu.yaw = 0;
  imu.pitch = 0;
  imu.roll = 0;
}
void calcRate() {
  imu.readGyroData(imu.gyroCount);
  imu.getGres();
  float pitch_rate = (float) imu.gyroCount[0] * imu.gRes - pitch_offset;
  float roll_rate = (float) imu.gyroCount[1] * imu.gRes - roll_offset;
  float yaw_rate = (float) imu.gyroCount[2] * imu.gRes - yaw_offset;

  imu.readAccelData(imu.accelCount);
  imu.getAres();
  float x_acc = (float) imu.accelCount[0] * imu.aRes - x_offset;
  float y_acc = (float) imu.accelCount[1] * imu.aRes - y_offset;
  float z_acc = (float) imu.accelCount[2] * imu.aRes - z_offset;

  
  imu.pitch += pitch_rate * dt;
  imu.yaw += yaw_rate * dt;
  imu.roll += roll_rate * dt;
}

#define YAW_P 0.0
#define PITCH_P 0.0132
#define ROLL_P 0.0132

#define YAW_D 0.0
#define PITCH_D 0.0
#define ROLL_D 0.0

float yaw_err_last = 0;
float pitch_err_last = 0;
float roll_err_last = 0;
struct vec3e calc_pid(struct vec3e error) {
  float yaw_output = error.yaw * YAW_P + (error.yaw - yaw_err_last) * YAW_D / dt;
  float pitch_output = error.pitch * PITCH_P + (error.pitch - pitch_err_last) * PITCH_D / dt;
  float roll_output = error.roll * ROLL_P + (error.roll - roll_err_last) * ROLL_D / dt;
  yaw_err_last = error.yaw;
  pitch_err_last = error.pitch;
  roll_err_last = error.roll;
  
  struct vec3e output = {pitch_output, yaw_output, roll_output};
  return output;
}

float clamp_power(float power) {
  return min(max(power, 0.0), 1.0);
}

int power_to_us(float power) {
  if (power > 1) {
    power = 1;
  }
  else if (power < 0) {
    power = 0;
  }
  
  if (power < 0.1) {
    power = 0;
  }
  
  return (int) 480.0 * power + 1060;
}

int un_twos(byte i) {
  if (i > 127) {
    return ((int) (~i + 1)) * -1;
  }
  return (int) i;
}
long last_packet = 0;
long last_debug = 0;

float throttle = 0.0;
struct vec3e desired_angle = {0, 0, 0};
void loop() {
  long now = millis();
  long delta = now - last_intr;
  last_intr = now;
  dt = (float) delta / 1000.0;
  
  calcRate();
  struct vec3e error = {imu.roll - desired_angle.pitch, imu.yaw - desired_angle.yaw, imu.pitch - desired_angle.roll};
  struct vec3e controller_output = calc_pid(error);

  float yaw = controller_output.yaw;
  float pitch = controller_output.pitch;
  float roll = controller_output.roll;

  if (throttle == 0.0) {
    yaw = 0;
    pitch = 0;
    roll = 0;
  }
  else {
    if (now - last_debug > 500) {
      /*mySerial.print("Pitch: ");
      mySerial.print(pitch);
      mySerial.print(" Error: ");
      mySerial.println(error.pitch);
      mySerial.print("Roll: ");
      mySerial.print(roll);
      mySerial.print(" Error: ");
      mySerial.println(error.roll);*/
      last_debug = now;
    }
  }
  
  int fr = power_to_us(clamp_power(throttle - pitch + yaw));
  int fl = power_to_us(clamp_power(throttle - roll - yaw));
  int rl = power_to_us(clamp_power(throttle + pitch + yaw));
  int rr = power_to_us(clamp_power(throttle + roll - yaw));

  
  Serial.print(throttle);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(roll);
  if (millis() - last_packet < 500) {
    esc1.writeMicroseconds(fr);
    esc2.writeMicroseconds(fl);
    esc3.writeMicroseconds(rl);
    esc4.writeMicroseconds(rr);
  }
  else {
    esc1.writeMicroseconds(1060);
    esc2.writeMicroseconds(1060);
    esc3.writeMicroseconds(1060);
    esc4.writeMicroseconds(1060);
  }
  
  if (mySerial.available() >= 5) {
    last_packet = millis();
    
    int8_t in_yaw = mySerial.read();
    int8_t in_pitch = mySerial.read();
    int8_t in_roll = mySerial.read();
    int8_t in_throttle = mySerial.read();
    int8_t special = mySerial.read();
    if (abs(in_throttle) < 10) {
      in_throttle = 0;
    }
    if (abs(in_pitch) < 10) {
      in_pitch = 0;
    }
    if (abs(in_yaw) < 10) {
      in_yaw = 0;
    }
    if (abs(in_roll) < 10) {
      in_roll = 0;
    }

    throttle = ((float) min(max(in_throttle, 0), 127)) / 127.0;
    float yaw = 10 * ((float) in_yaw) / 128.0;
    float pitch = 15 * ((float) in_pitch) / 128.0;
    float roll = 15 * ((float) in_roll) / 128.0;
    desired_angle.yaw += yaw;
    desired_angle.roll = roll;
    desired_angle.pitch = pitch;
  }
  
}
