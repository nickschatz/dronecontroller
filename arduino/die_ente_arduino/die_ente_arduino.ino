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
float _ayaw, _apitch, _aroll;
float _gyaw, _gpitch, _groll;

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
  int status = myIMU.begin();
  if (status < 0) {
    Serial.print("Failed to connect to MPU9255: ");
    Serial.println(status);
    while (1);
  }
  myIMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  myIMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  myIMU.enableDataReadyInterrupt();

  myIMU.setMagCalX(-0.76, 2.00);
  myIMU.setMagCalY(14.49, 1.33);
  myIMU.setMagCalZ(35.09, 0.57);
  
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

unsigned long last_imu = millis();
void update_imu() {
  myIMU.readSensor();
  ax = myIMU.getAccelX_mss();
  ay = myIMU.getAccelY_mss();
  az = myIMU.getAccelZ_mss();
  gx = myIMU.getGyroX_rads() * RAD_TO_DEG;
  gy = myIMU.getGyroY_rads() * RAD_TO_DEG;
  gz = myIMU.getGyroZ_rads() * RAD_TO_DEG;
  mx = myIMU.getMagX_uT();
  my = myIMU.getMagY_uT();
  mz = myIMU.getMagZ_uT();
  
  unsigned long noww = micros();
  float deltaa = ((float) (noww - last_imu)) / 1000000.0f;
  last_imu = noww;
  quat_update(deltaa);
}

void quat_update(float deltaa) {
  MadgwickQuaternionUpdate(ax, ay, -az, gx * DEG_TO_RAD, gy * DEG_TO_RAD, -gz * DEG_TO_RAD, mx, my, mz, deltaa);
  yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
  pitch = asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                *(getQ()+2)));
  roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
  pitch *= RAD_TO_DEG;
  yaw   *= RAD_TO_DEG;
  yaw   -= 0.2167;
  roll *= RAD_TO_DEG;
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
  struct state x = {yaw, pitch, roll, gz, gx, gy};
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
  if (delta >= 5) {
    last_intr = now;
    dt = (float) delta / 1000.0;
    
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
      float cyaw = 10 * ((float) in_yaw) / 127.0;
      float cpitch = 5 * ((float) in_pitch) / 127.0;
      float croll = 5 * ((float) in_roll) / 127.0;
      desired_angle.yaw += cyaw;
      desired_angle.roll = croll;
      desired_angle.pitch = cpitch;
    }
  
    if (now - last_debug > 500) {
      Serial.print(F(" yaw: "));
      Serial.print(yaw);
      Serial.print(F(" pitch: "));
      Serial.print(pitch);
      Serial.print(F(" roll: "));
      Serial.println(roll);
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
