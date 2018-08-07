#include <Wire.h>
#ifndef _WIRE2
#define _WIRE2
TwoWire WIRE2 (2,I2C_FAST_MODE);
#endif
#define Wire WIRE2

#include "MPU9250.h"
#include "quaternionFilters.h"

#include <SPI.h>
#include <Servo.h>

#define STATUS_LED PC13
#define VIN_READ PA5
#define CAL_COUNTS 50
#define MIN_THROTTLE 0.2
#define TETHER_CMD 1
// 13.2 / 1023
// Small fudge for variances in resistors
#define VOLTAGE_SCALE 0.0129032258
#define STOP 255

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

struct state {
  float y;
  float p;
  float r;
  float yd;
  float pd;
  float rd;
};

typedef struct motors {
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

//Max safe speed is 50%
void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  pinMode(VIN_READ, INPUT);
  
  Serial.begin(230400);
  Serial2.begin(9600);
  #if TETHER_CMD
  while (!Serial.isConnected()) {
     delay(200);
     digitalWrite(STATUS_LED, HIGH);
     delay(400);
     digitalWrite(STATUS_LED, LOW);
  }
  #endif
  Serial.println(F("Starting..."));

  pinMode(PB1, PWM);
  pinMode(PB0, PWM);
  pinMode(PA7, PWM);
  pinMode(PA6, PWM);
  esc1.attach(PB1);
  esc2.attach(PB0);
  esc3.attach(PA7);
  esc4.attach(PA6);
  esc1.writeMicroseconds(1060);
  esc2.writeMicroseconds(1060);
  esc3.writeMicroseconds(1060);
  esc4.writeMicroseconds(1060);

  Serial.println(F("Wrote initial to ESCs"));
  Wire.begin();
  // Setup IMU
  byte id = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (id != 0x73) {
    while (1) {
      Serial.print(F("Failed to connect to MPU9250: WAI: "));
      Serial.println(id, HEX);
      delay(1000);
    }
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

float clamp(float sig, float cutoff) {
  return min(max(-cutoff, sig), cutoff);
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
  
  /*MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);*/
  
  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;

  // update ypr at 200Hz
  if (myIMU.delt_t > 5)
  {
    MadgwickQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*PI/180.0f, myIMU.gy*PI/180.0f, myIMU.gz*PI/180.0f,  myIMU.my, myIMU.mx, myIMU.mz, myIMU.deltat);

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
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    myIMU.yaw   -= 0.2167;
    myIMU.roll  *= RAD_TO_DEG;

    //Subtract offsets
    myIMU.yaw -= yaw_offset;
    myIMU.pitch -= pitch_offset;
    myIMU.roll -= roll_offset;
  }
}


void desaturate(struct motors *pows) {
  // Rescale all to [min(pows), max(pows)]

  // If all zeroes, then stop because we don't want constant spinning
  if (pows->fr == 0.0 && pows->fl == 0.0 && pows->rl == 0.0 && pows->rr == 0.0) {
    return;
  }
  float minpow = min(pows->fr, min(pows->fl, min(pows->rl, pows->rr)));
  float maxpow = max(pows->fr, max(pows->fl, max(pows->rl, pows->rr)));
  // No need to do anything if it's already in acceptable range
  if (maxpow <= 1.0 && minpow >= MIN_THROTTLE) {
    return;
  }
  
  float d = maxpow - minpow;
  // Scale to between 0 and 1
  pows->fr = (pows->fr - minpow) / d;
  pows->fl = (pows->fl - minpow) / d;
  pows->rl = (pows->rl - minpow) / d;
  pows->rr = (pows->rr - minpow) / d;

  // Rescale to min and max
  float min_ = max(minpow, MIN_THROTTLE);
  float max_ = min(maxpow, 1.0);
  float d2 = max_ - min_;
  pows->fr = pows->fr * d2 + min_;
  pows->fl = pows->fl * d2 + min_;
  pows->rl = pows->rl * d2 + min_;
  pows->rr = pows->rr * d2 + min_;
}

bool is_low_voltage() {
  float vin = ((float) analogRead(VIN_READ)) * VOLTAGE_SCALE;
  if (vin < 8.0) {
    // Either this LiPo is totally screwed or you're on USB power
    return false;
  }
  else {
    return vin < 10.0;
  }
}

int8_t* read_buf = new int8_t[6];
bool serial_read(int8_t* in_yaw, int8_t* in_pitch, int8_t* in_roll, int8_t* in_throttle, int8_t* special) {
  
  #if TETHER_CMD
  #define SERIAL Serial
  #else
  #define SERIAL Serial2
  #endif
  if (SERIAL.available() < 6) {
    return false;
  }
  // Read bytes until the last one read is STOP and I've read at least 6 total
  unsigned long count = 0;
  byte buf_ptr = 0;
  byte last_read = 0;
  while (last_read != STOP) {
    last_read = SERIAL.read();
    
    read_buf[buf_ptr] = last_read;
    count++;
    buf_ptr = (buf_ptr + 1) % 6;
  }
  buf_ptr = (buf_ptr - 1) % 6;
  // Read forwards
  *in_yaw = read_buf[buf_ptr];
  buf_ptr = (buf_ptr + 1) % 6;
  *in_pitch = read_buf[buf_ptr];
  buf_ptr = (buf_ptr + 1) % 6;
  *in_roll = read_buf[buf_ptr];
  buf_ptr = (buf_ptr + 1) % 6;
  *in_throttle = read_buf[buf_ptr];
  buf_ptr = (buf_ptr + 1) % 6;
  *special = read_buf[buf_ptr];
  
  return true;
}

struct vec3e desired_angle = {0, 0, 0};
void update_controller(struct motors *u) {
  struct state x = {myIMU.yaw, myIMU.pitch, myIMU.roll, myIMU.gz, myIMU.gx, myIMU.gy};
  struct state rmx = {desired_angle.yaw - x.y, desired_angle.pitch - x.p, desired_angle.roll - x.r, -x.yd, -x.pd, -x.rd};
  u->fr = -0.0129 * rmx.y +   0.0276 * rmx.p +  0.0219 * rmx.r + -0.0019 * rmx.yd +  0.0017 * rmx.pd +  0.0013 * rmx.rd;
  u->fl =  0.0129 * rmx.y +   0.0276 * rmx.p + -0.0219 * rmx.r +  0.0019 * rmx.yd +  0.0017 * rmx.pd + -0.0013 * rmx.rd;
  u->rl = -0.0129 * rmx.y +  -0.0276 * rmx.p + -0.0219 * rmx.r + -0.0019 * rmx.yd + -0.0017 * rmx.pd + -0.0013 * rmx.rd;
  u->rr =  0.0129 * rmx.y +  -0.0276 * rmx.p +  0.0219 * rmx.r +  0.0019 * rmx.yd + -0.0017 * rmx.pd +  0.0013 * rmx.rd;
  //u->fr *= dt / 0.010;
  //u->fl *= dt / 0.010;
  //u->rl *= dt / 0.010;
  //u->rr *= dt / 0.010;
}
void add_throttle(struct motors *u, float throttle) {
  if (throttle == 0.0) {
    u->fr = 0.0;
    u->fl = 0.0;
    u->rl = 0.0;
    u->rr = 0.0;
    return;
  }
  u->fr = max(u->fr, 0);
  u->fl = max(u->fl, 0);
  u->rl = max(u->rl, 0);
  u->rr = max(u->rr, 0);
  float throttle_ = MIN_THROTTLE + (1 - MIN_THROTTLE) * throttle;
  u->fr += throttle_;
  u->fl += throttle_;
  u->rl += throttle_;
  u->rr += throttle_;
}


long last_packet = 0;
long last_debug = 0;

float throttle = 0.0;

void loop() {
  update_imu();
  long now = millis();
  long delta = now - last_intr;
  if (delta >= 10) {
    last_intr = now;
    dt = (float) 10 / 1000.0;
    
    int8_t in_yaw;
    int8_t in_pitch;
    int8_t in_roll;
    int8_t in_throttle;
    int8_t special;
    bool has_data = serial_read(&in_yaw, &in_pitch, &in_roll, &in_throttle, &special);
  
    if (has_data) {
      last_packet = millis();
      /*Serial.print(in_yaw);
      Serial.print(F(" P: "));
      Serial.print(in_pitch);
      Serial.print(F(" R: "));
      Serial.print(in_roll);
      Serial.print(F(" T: "));
      Serial.print(in_throttle);
      Serial.print(F(" S: "));
      Serial.println(special);*/
      if (abs(in_throttle) < 20) {
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
      float yaw = 10 * ((float) in_yaw) / 127.0;
      float pitch = 5 * ((float) in_pitch) / 127.0;
      float roll = 5 * ((float) in_roll) / 127.0;
      desired_angle.yaw += yaw;
      desired_angle.roll = roll;
      desired_angle.pitch = pitch;
    }
  
    if (now - last_debug > 500) {
      Serial.print(F(" dt "));
      Serial.print(delta);
      Serial.print(F(" gy: "));
      Serial.print(myIMU.gy);
      Serial.print(F(" gx: "));
      Serial.print(myIMU.gx);
      Serial.print(F(" gz: "));
      Serial.print(myIMU.gz);
      Serial.print(F(" CmdAngle: "));
      Serial.println(desired_angle.roll);
      if (is_low_voltage()) {
        Serial.println("!!! LOW VOLTAGE !!!");
      }
      last_debug = now;
    }
  
    struct motors pows;
    update_controller(&pows);
    add_throttle(&pows, throttle);
    //desaturate(&pows);
    
    int fr = power_to_us(pows.fr);
    int fl = power_to_us(pows.fl);
    int rl = power_to_us(pows.rl);
    int rr = power_to_us(pows.rr);
  
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
  }
  
}
