#include "MPU9250.h"
#include "quaternionFilters.h"

#include <SPI.h>
#include <Firmata.h>
#include <Boards.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define STATUS_LED A0
#define CAL_COUNTS 50

struct vec3 {
  float x;
  float y;
  float z;
};

struct vec3e {
  float yaw;
  float pitch;
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

MPU9250 myIMU;

long last;
long last_intr;

float yaw_offset;
float pitch_offset;
float roll_offset;

float dt = 0;

SoftwareSerial mySerial(2, 3); // RX, TX

//Max safe speed is 50%
void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  Serial.begin(230400);
  mySerial.begin(9600);
  Serial.println(F("Starting..."));
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
  byte id = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (id != 0x71) {
    Serial.println(F("Failed to connect to MPU9250"));
    return;
  }
  
  Serial.println(F("Connected to IMU"));
  // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.SelfTest[0],1); Serial.println(F("% of factory value"));
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.SelfTest[1],1); Serial.println(F("% of factory value"));
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.SelfTest[2],1); Serial.println(F("% of factory value"));
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.SelfTest[3],1); Serial.println(F("% of factory value"));
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.SelfTest[4],1); Serial.println(F("% of factory value"));
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.SelfTest[5],1); Serial.println(F("% of factory value"));

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println(F("MPU9250 initialized for active data mode...."));

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print(F("AK8963 ")); Serial.print(F("I AM ")); Serial.print(d, HEX);
    Serial.print(F(" I should be ")); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println(F("AK8963 initialized for active data mode...."));
  
  Serial.println(F("Calculating bias..."));
  int counts = 0;
  float yawAvr[CAL_COUNTS];
  float pitchAvr[CAL_COUNTS];
  float rollAvr[CAL_COUNTS];

  //Wait a few seconds for IMU values to stabilize
  digitalWrite(STATUS_LED, HIGH);
  long start = millis();
  while (millis() - start < 2000) {
    update_imu();
  }
  
  bool ledState = true;
  while (counts < CAL_COUNTS) {
    if (!ledState) {
      digitalWrite(STATUS_LED, HIGH);
      ledState = true;
    }
    else {
      digitalWrite(STATUS_LED, LOW);
      ledState = false;
    }
    update_imu();
    pitchAvr[counts] = myIMU.pitch;
    rollAvr[counts] = myIMU.roll;
    yawAvr[counts] = myIMU.yaw;
    
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
  Serial.print(F("Yaw bias:"));
  Serial.println(yaw_offset);
  Serial.print(F("Pitch bias:"));
  Serial.println(pitch_offset);
  Serial.print(F("Roll bias:"));
  Serial.println(roll_offset);
  
  Serial.println(F("READY"));
  digitalWrite(STATUS_LED, HIGH);
  last = millis();
  last_intr = millis();
}


#define YAW_P 0.0
#define PITCH_P 0.005
#define ROLL_P 0.005

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
  
  struct vec3e output = {yaw_output, pitch_output, roll_output};
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

void update_imu() {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  
  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;

  // update ypr at 50Hz
  if (myIMU.delt_t > 20)
  {

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
    myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                  *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                  *(getQ()+2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                  *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    myIMU.pitch *= RAD_TO_DEG;
    myIMU.yaw   *= RAD_TO_DEG;
    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    myIMU.yaw   -= 0.2167;
    myIMU.roll  *= RAD_TO_DEG;

    //Subtract offsets
    myIMU.yaw -= yaw_offset;
    myIMU.pitch -= pitch_offset;
    myIMU.roll -= roll_offset;
  }
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
  
  update_imu();
  
  struct vec3e error = {myIMU.yaw - desired_angle.yaw, myIMU.pitch - desired_angle.pitch, myIMU.roll - desired_angle.roll};
  struct vec3e controller_output = calc_pid(error);

  float yaw = controller_output.yaw;
  float pitch = controller_output.pitch;
  float roll = controller_output.roll;

  if (throttle == 0.0) {
    yaw = 0;
    pitch = 0;
    roll = 0;
  }
  if (now - last_debug > 500) {
      Serial.print(F("Pitch: "));
      Serial.print(myIMU.pitch);
      Serial.print(F(" Error: "));
      Serial.println(error.pitch);
      Serial.print(F("Roll: "));
      Serial.print(myIMU.roll);
      Serial.print(F(" Error: "));
      Serial.println(error.roll);
      Serial.print(1000.0 / delta); Serial.println(F(" hz"));
      last_debug = now;
    }
  
  int fr = power_to_us(clamp_power(throttle - pitch + yaw));
  int fl = power_to_us(clamp_power(throttle - roll - yaw));
  int rl = power_to_us(clamp_power(throttle + pitch + yaw));
  int rr = power_to_us(clamp_power(throttle + roll - yaw));

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