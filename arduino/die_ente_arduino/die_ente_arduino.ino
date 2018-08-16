#include <Eigen.h>

#include <Wire.h>
#include "ente.h"
TwoWire WIRE2 (2,I2C_FAST_MODE);

#include "MPU9250.h"
#include "quaternionFilters.h"

#include <SPI.h>
#include <Servo.h>

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

MPU9250 myIMU(WIRE2, 0x68);
Madgwick AHRSFilter;

long last;
long last_intr;

float yaw_offset;
float pitch_offset;
float roll_offset;

float dt = 0;
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float yaw, pitch, roll;
float dyaw, dpitch, droll;
float _ayaw, _apitch, _aroll;
float _gyaw, _gpitch, _groll;

Eigen::MatrixXf K(4, 6);
Eigen::MatrixXf A(6, 6);
Eigen::MatrixXf B(6, 4);
Eigen::MatrixXf L(6, 6);
Eigen::VectorXf x_hat(6);
Eigen::VectorXf y_hat(6);
Eigen::VectorXf u_last(4);

float _min(float a, float b);
float _max(float a, float b);

//Max safe speed is 50%
void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  pinMode(VIN_READ, INPUT);

  // Setup matrices
  K << 
   0.054419,  0.069447, -0.112244,  0.007390,  0.007095, -0.005849, 
  -0.054419,  0.069447,  0.112244, -0.007390,  0.007095,  0.005849, 
   0.054419, -0.069447,  0.112244,  0.007390, -0.007095,  0.005849, 
  -0.054419, -0.069447, -0.112244, -0.007390, -0.007095, -0.005849;
  A << 
   1.000000, 0.000000, 0.000000,  0.002000, 0.000000, 0.000000, 
  0.000000,  1.000000, 0.000000, 0.000000,  0.002000, 0.000000, 
  0.000000, 0.000000,  1.000000, 0.000000, 0.000000,  0.002000, 
  0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 
  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 
  0.000000, 0.000000, 0.000000, 0.000000, 0.000000,  1.000000;
  B << 
   0.027620, -0.027620,  0.027620, -0.027620, 
   0.032837,  0.032837, -0.032837, -0.032837, 
  -0.041384,  0.041384,  0.041384, -0.041384, 
   27.619921, -27.619921,  27.619921, -27.619921, 
   32.836848,  32.836848, -32.836848, -32.836848, 
  -41.383618,  41.383618,  41.383618, -41.383618;
  L << 
   0.002200, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 
  0.000000,  0.002000, 0.000000, 0.000000, 0.000000, 0.000000, 
  0.000000, 0.000000,  0.001800, 0.000000, 0.000000, 0.000000, 
   1.000000, 0.000000, 0.000000,  0.001600, 0.000000, 0.000000, 
  0.000000,  1.000000, 0.000000, 0.000000,  0.001400, 0.000000, 
  0.000000, 0.000000,  1.000000, 0.000000, 0.000000,  0.001200;

  x_hat << 0,0,0,0,0,0;
  y_hat = x_hat;
  
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
  Serial.print(get_battery_voltage());
  Serial.println("V");

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
  int status = myIMU.begin();
  if (status < 0) {
    Serial.print("Failed to connect to MPU9255: ");
    Serial.println(status);
    while (1);
  }
  myIMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  myIMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  myIMU.enableDataReadyInterrupt();
  myIMU.setSrd(9); // 1000 Hz / (1 + SRD) = 100 Hz
  myIMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

  myIMU.setMagCalX(3.3300, 1.0008);
  myIMU.setMagCalY(37.8650, 0.9320);
  myIMU.setMagCalZ(1.0100, 1.0778);
  
  Serial.println(F("READY"));
  digitalWrite(STATUS_LED, HIGH);
  last = millis();
  last_intr = millis();
}

float clamp(float sig, float cutoff) {
  return _min(_max(-cutoff, sig), cutoff);
}

float normalize_angle(float angle) {
  angle = fmod(angle, 360);
  if (angle < 0) {
    angle += 360;
  }
  return angle;
}

float sign(float x) {
  if (x > 0) {
    return 1.0;
  }
  if (x < 0) {
    return -1.0;
  }
  return 1.0;
}

float to_heading(float angle) {
  angle = normalize_angle(angle);
  if (angle > 180) {
    angle = -(360 - angle);
  }
  return angle;
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
  myIMU.readSensor(AHRSFilter);
  ax = myIMU.getAccelX_mss();
  ay = myIMU.getAccelY_mss();
  az = myIMU.getAccelZ_mss();
  gx = myIMU.getGyroX_rads() * RAD_TO_DEG;
  gy = myIMU.getGyroY_rads() * RAD_TO_DEG;
  gz = myIMU.getGyroZ_rads() * RAD_TO_DEG;
  mx = myIMU.getMagX_uT();
  my = myIMU.getMagY_uT();
  mz = myIMU.getMagZ_uT();
  dyaw = -gz;
  dpitch = -gx;
  droll = -gy;
  yaw = to_heading(AHRSFilter.getYaw());
  pitch = AHRSFilter.getPitch();
  roll = AHRSFilter.getRoll();
  float roll_sign = sign(roll);
  roll = roll_sign * abs(180.0 - abs(roll));
}

void complementary_update(float deltaa) {
  float r = 0.0;
  float gr = 1.0 - r;
  float grav;
  float amag = sqrt(ax * ax + ay * ay + az * az);
  if (amag > 0.0) {
    _ayaw += atan2(ay, ax) * RAD_TO_DEG * deltaa;
    _apitch += atan2(az, ay) * RAD_TO_DEG * deltaa;
    _aroll += atan2(ax, az) * RAD_TO_DEG * deltaa;
  }
  _gyaw += gz * deltaa;
  _gpitch += gx * deltaa;
  _groll += gy * deltaa;

  yaw = _gyaw * gr + _ayaw * r;
  pitch = _gpitch * gr + _apitch * r;
  roll = _groll * gr + _aroll * r;
}


void desaturate(struct motors *pows) {
  // Rescale all to [min(pows), max(pows)]

  // If all zeroes, then stop because we don't want constant spinning
  if (pows->fr == 0.0 && pows->fl == 0.0 && pows->rl == 0.0 && pows->rr == 0.0) {
    return;
  }
  float minpow = _min(pows->fr, _min(pows->fl, _min(pows->rl, pows->rr)));
  float maxpow = _max(pows->fr, _max(pows->fl, _max(pows->rl, pows->rr)));
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
  float min_ = _max(minpow, MIN_THROTTLE);
  float max_ = _min(maxpow, 1.0);
  float d2 = max_ - min_;
  pows->fr = pows->fr * d2 + min_;
  pows->fl = pows->fl * d2 + min_;
  pows->rl = pows->rl * d2 + min_;
  pows->rr = pows->rr * d2 + min_;
}

float get_battery_voltage() {
  return ((float) analogRead(VIN_READ)) * VOLTAGE_SCALE;
}

bool is_low_voltage() {
  float vin = get_battery_voltage();
  return vin < 10.0f;
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
  Eigen::VectorXf y(6);
  y << yaw, pitch, roll, dyaw, dpitch, droll;
  Eigen::VectorXf r(6);
  r << desired_angle.yaw, desired_angle.pitch, desired_angle.roll, 0.0, 0.0, 0.0;
  Eigen::VectorXf u_vec;

  //Observer
  x_hat = y; //A * x_hat + B * u_last + L * (y - y_hat);
  y_hat = x_hat;

  // Controller
  u_vec = K*(r - x_hat);
  for (int i = 0; i < 4; i++) {
    u_vec(i) = _min(_max(u_vec(i), 0.0), 1.0);
  }
  u_last = u_vec;
  u->fr = u_vec(0);
  u->fl = u_vec(1);
  u->rl = u_vec(2);
  u->rr = u_vec(3);
}
void add_throttle(struct motors *u, float throttle) {
  if (throttle == 0.0) {
    u->fr = 0.0;
    u->fl = 0.0;
    u->rl = 0.0;
    u->rr = 0.0;
    return;
  }
  u->fr = _max(u->fr, 0);
  u->fl = _max(u->fl, 0);
  u->rl = _max(u->rl, 0);
  u->rr = _max(u->rr, 0);
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
  unsigned long noww = micros();
  float delta = ((float) (noww - last_intr)) / 1000.0f;
  if (delta >= 2) {
    update_imu();
    last_intr = noww;
    
    int8_t in_yaw;
    int8_t in_pitch;
    int8_t in_roll;
    int8_t in_throttle;
    int8_t special;
    bool has_data = serial_read(&in_yaw, &in_pitch, &in_roll, &in_throttle, &special);
  
    if (has_data) {
      last_packet = millis();
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
  
      throttle = ((float) _min(_max(in_throttle, 0), 127)) / 127.0;
      float cyaw = 10 * ((float) in_yaw) / 127.0;
      float cpitch = 5 * ((float) in_pitch) / 127.0;
      float croll = 5 * ((float) in_roll) / 127.0;
      desired_angle.yaw += cyaw;
      desired_angle.roll = croll;
      desired_angle.pitch = cpitch;
    }
  
    if (noww - last_debug > 100) {
      /*Serial.print(F(" yaw: "));
      Serial.print(yaw);
      Serial.print(F(" pitch: "));
      Serial.print(pitch);
      Serial.print(F(" roll: "));
      Serial.println(roll);*/
      Serial.println(delta);
      if (is_low_voltage()) {
        //Serial.println("!!! LOW VOLTAGE !!!");
      }
      last_debug = noww;
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

float _min(float a, float b) {
  if (a < b) {
    return a;
  }
  return b;
}

float _max(float a, float b) {
  if (a > b) {
    return a;
  }
  return b;
}

void magcal() {
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  myIMU.calibrateMag();
  Serial.print(" Offset X: ");
  Serial.print(myIMU.getMagBiasX_uT());
  Serial.print(" Offset Y: ");
  Serial.print(myIMU.getMagBiasY_uT());
  Serial.print(" Offset Z: ");
  Serial.println(myIMU.getMagBiasZ_uT());
  Serial.print("Scale X: ");
  Serial.print(myIMU.getMagScaleFactorX());
  Serial.print(" Scale Y: ");
  Serial.print(myIMU.getMagScaleFactorY());
  Serial.print(" Scale Z: ");
  Serial.println(myIMU.getMagScaleFactorZ());
  Serial.println("Mag Calibration done!");
}

void acccal() {
  Serial.println("Accel Calibration: Move to X up (5s)");
  delay(5000);
  myIMU.calibrateAccel();
  Serial.print(" Offset X: ");
  Serial.print(myIMU.getAccelBiasX_mss());
  Serial.print(" Scale X: ");
  Serial.println(myIMU.getAccelScaleFactorX());
  
  Serial.println("Accel Calibration: Move to X down (5s)");
  delay(5000);
  myIMU.calibrateAccel();
  Serial.print(" Offset X: ");
  Serial.print(myIMU.getAccelBiasX_mss());
  Serial.print(" Scale X: ");
  Serial.println(myIMU.getAccelScaleFactorX());

  Serial.println("Accel Calibration: Move to Y up (5s)");
  delay(5000);
  myIMU.calibrateAccel();
  Serial.print(" Offset Y: ");
  Serial.print(myIMU.getAccelBiasY_mss());
  Serial.print(" Scale Y: ");
  Serial.println(myIMU.getAccelScaleFactorY());

  Serial.println("Accel Calibration: Move to Y down (5s)");
  delay(5000);  
  myIMU.calibrateAccel();
  Serial.print(" Offset Y: ");
  Serial.print(myIMU.getAccelBiasY_mss());
  Serial.print(" Scale Y: ");
  Serial.println(myIMU.getAccelScaleFactorY());

  Serial.println("Accel Calibration: Move to Z down (5s)");
  delay(5000);  
  Serial.print(" Offset Z: ");
  Serial.print(myIMU.getAccelBiasZ_mss());  
  Serial.print(" Scale Z: ");
  Serial.println(myIMU.getAccelScaleFactorZ());

  Serial.println("Accel Calibration: Move to Z up (5s)");
  delay(5000);  
  Serial.print(" Offset Z: ");
  Serial.print(myIMU.getAccelBiasZ_mss());  
  Serial.print(" Scale Z: ");
  Serial.println(myIMU.getAccelScaleFactorZ());
  
  Serial.println("Accel Calibration done!");
}
void acccal2() {
  Serial.println("Accel Calibration: Move around");
  delay(5000);
  myIMU.calibrateAccel();
  Serial.print(" Offset X: ");
  Serial.print(myIMU.getAccelBiasX_mss());
  Serial.print(" Scale X: ");
  Serial.println(myIMU.getAccelScaleFactorX());
  Serial.print(" Offset Y: ");
  Serial.print(myIMU.getAccelBiasY_mss());
  Serial.print(" Scale Y: ");
  Serial.println(myIMU.getAccelScaleFactorY());
  Serial.print(" Offset Z: ");
  Serial.print(myIMU.getAccelBiasZ_mss());  
  Serial.print(" Scale Z: ");
  Serial.println(myIMU.getAccelScaleFactorZ());
  
  Serial.println("Accel Calibration done!");
}
void esctest() {
  esc1.writeMicroseconds(power_to_us(0.2f));
  delay(500);
  esc1.writeMicroseconds(power_to_us(0.0f));
  esc2.writeMicroseconds(power_to_us(0.2f));
  delay(500);
  esc2.writeMicroseconds(power_to_us(0.0f));
  esc3.writeMicroseconds(power_to_us(0.2f));
  delay(500);
  esc3.writeMicroseconds(power_to_us(0.0f));
  esc4.writeMicroseconds(power_to_us(0.2f));
  delay(500);
  esc4.writeMicroseconds(power_to_us(0.0f));
}

