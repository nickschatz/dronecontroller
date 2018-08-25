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
float dt = 0;
float yaw, pitch, roll;
float dyaw, dpitch, droll;
float lyaw = 0, lpitch = 0, lroll = 0;
float* avgdyaw = new float[DAVG_COUNT];
float* avgdpitch = new float[DAVG_COUNT];
float* avgdroll = new float[DAVG_COUNT];
byte avgd_pt = 0;
float yaw_offset, pitch_offset, roll_offset;
float _rroll, _ryaw, _rpitch;

Eigen::MatrixXf K(4, 6);
Eigen::MatrixXf A(6, 6);
Eigen::MatrixXf B(6, 4);
Eigen::MatrixXf L(6, 6);
Eigen::VectorXf x_hat(6);
Eigen::VectorXf y_hat(6);
Eigen::VectorXf u_last(4);

HardwareTimer imuTimer(1);

float _min(float a, float b);
float _max(float a, float b);

//Max safe speed is 50%
void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  pinMode(VIN_READ, INPUT_ANALOG);

  // Setup matrices
  K << 
   0.025503,  0.020031, -0.034232,  0.003534,  0.002783, -0.002457, 
  -0.025503,  0.020031,  0.034232, -0.003534,  0.002783,  0.002457, 
   0.025503, -0.020031,  0.034232,  0.003534, -0.002783,  0.002457, 
  -0.025503, -0.020031, -0.034232, -0.003534, -0.002783, -0.002457;
  A << 
   1.000000, 0.000000, 0.000000,  0.005000, 0.000000, 0.000000, 
  0.000000,  1.000000, 0.000000, 0.000000,  0.005000, 0.000000, 
  0.000000, 0.000000,  1.000000, 0.000000, 0.000000,  0.005000, 
  0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 0.000000, 
  0.000000, 0.000000, 0.000000, 0.000000,  1.000000, 0.000000, 
  0.000000, 0.000000, 0.000000, 0.000000, 0.000000,  1.000000;
  B << 
   0.172625, -0.172625,  0.172625, -0.172625, 
   0.205230,  0.205230, -0.205230, -0.205230, 
  -0.258648,  0.258648,  0.258648, -0.258648, 
   69.049803, -69.049803,  69.049803, -69.049803, 
   82.092120,  82.092120, -82.092120, -82.092120, 
  -103.459044,  103.459044,  103.459044, -103.459044;
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
  Serial2.begin(230400);
  #if TETHER_CMD
  while (!Serial.isConnected()) {
     delay(200);
     digitalWrite(STATUS_LED, HIGH);
     delay(400);
     digitalWrite(STATUS_LED, LOW);
  }
  digitalWrite(STATUS_LED, LOW);
  #endif
  SERIALD.println(F("Starting..."));
  SERIALD.print(get_battery_voltage());
  SERIALD.println("V");

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

  SERIALD.println(F("Wrote initial to ESCs"));
  int status = myIMU.begin();
  if (status < 0) {
    SERIALD.print("Failed to connect to MPU9255: ");
    SERIALD.println(status);
    while (1);
  }
  myIMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  myIMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  myIMU.enableDataReadyInterrupt();
  myIMU.setSrd(4); // 1000 Hz / (1 + SRD) = 200 Hz
  myIMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);

  myIMU.setMagCalX(3.3300, 1.0008);
  myIMU.setMagCalY(37.8650, 0.9320);
  myIMU.setMagCalZ(1.0100, 1.0778);

  AHRSFilter.begin(1000.0, 0.1);

  imuTimer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  imuTimer.pause();
  imuTimer.setCount(0);
  imuTimer.setPeriod(1000);
  imuTimer.setCompare(TIMER_CH1, 0);
  imuTimer.attachCompare1Interrupt(update_imu);
  imuTimer.refresh();
  imuTimer.resume();

  SERIALD.println("Waiting for Madgwick to stabilize (15s)");
  delay(15000);
  clearSerial(&SERIAL);
  SERIALD.println(F("READY"));
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

unsigned long last_imu;
float imu_dt;
void update_imu() {
  unsigned long imu_now = micros();
  imu_dt = ((float) (imu_now - last_imu)) / 1000.0f;
  last_imu = imu_now;
  AHRSFilter.update(myIMU.getGyroX_rads(), myIMU.getGyroY_rads(), myIMU.getGyroZ_rads(), 
                    myIMU.getAccelX_mss(), myIMU.getAccelY_mss(), myIMU.getAccelZ_mss(), 
                    myIMU.getMagX_uT(), myIMU.getMagY_uT(), myIMU.getMagZ_uT());
  _ryaw = AHRSFilter.getYaw();
  _rpitch = -AHRSFilter.getPitch();

  _rroll = AHRSFilter.getRoll();
  float roll_sign = sign(_rroll);
  _rroll = -roll_sign * fabs(180.0 - fabs(_rroll));
  yaw = to_heading(_ryaw - yaw_offset);
  pitch = _rpitch - pitch_offset;
  roll = _rroll - roll_offset;

  float dts = imu_dt / 1000.0f;
  float _dyaw = (yaw - lyaw) / dts;
  float _dpitch = (pitch - lpitch) / dts;
  float _droll = (roll - lroll) / dts;
  dyaw = 0;
  dpitch = 0;
  droll = 0;
  avgdyaw[avgd_pt] = _dyaw;
  avgdpitch[avgd_pt] = _dpitch;
  avgdroll[avgd_pt] = _droll;
  avgd_pt = (avgd_pt + 1) % DAVG_COUNT;
  for (int i = 0; i < DAVG_COUNT; i++) {
    dyaw += avgdyaw[i];
    dpitch += avgdpitch[i];
    droll += avgdroll[i];
  }
  dyaw /= DAVG_COUNT;
  dpitch /= DAVG_COUNT;
  droll /= DAVG_COUNT;
  
  lyaw = yaw;
  lpitch = pitch;
  lroll = roll;
}

float get_battery_voltage() {
  return ((float) analogRead(VIN_READ)) * VOLTAGE_SCALE;
}

bool is_low_voltage() {
  float vin = get_battery_voltage();
  return vin < 11.1f;
}

void clearSerial(Stream * handle) {
  while (handle->read() != -1) {}
}

int8_t* read_buf = new int8_t[6];
bool serial_read(int8_t* in_yaw, int8_t* in_pitch, int8_t* in_roll, int8_t* in_throttle, int8_t* special) {
  
  if (SERIAL.available() < 6) {
    //Serial.println("No data");
    return false;
  }
  // Read bytes until the last one read is STOP and I've read at least 6 total
  unsigned long count = 0;
  byte buf_ptr = 0;
  int8_t last_read = 0;
  while (!(last_read == STOP && count >= 6)) {
    last_read = SERIAL.read();
    
    read_buf[buf_ptr] = last_read;
    buf_ptr = (buf_ptr + 1) % 6;
    count++;

    // Stop if we've recieved 12 bytes without a stop byte, because that's bad and could freeze the quad, which is generally a bad thing
    if (count > 12) {
      return false;
    }
  }
  buf_ptr = (buf_ptr == 0 ? 5 : buf_ptr - 1);
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

  // Only take the freshest data
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
    //u_vec(i) = _min(_max(u_vec(i), 0.0), 1.0);
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
unsigned long last_debug = 0;

float throttle = 0.0;

int control_ct = 0;
void loop() {
  unsigned long noww = micros();
  float delta = ((float) (noww - last_intr)) / 1000.0f;
  if (delta >= 5) {
    last_intr = noww;

    myIMU.readSensor();
    int8_t in_yaw = 0;
    int8_t in_pitch = 0;
    int8_t in_roll = 0;
    int8_t in_throttle = 0;
    int8_t special = 0;
    bool has_data = serial_read(&in_yaw, &in_pitch, &in_roll, &in_throttle, &special);
    bool debug = noww - last_debug > 200000;
  
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
      if (special & 1) {
        yaw_offset = _ryaw;
        pitch_offset = _rpitch;
        roll_offset = _rroll;
        //Serial.println(F("Zero YPR"));
      }
  
      throttle = ((float) _min(_max(in_throttle, 0), 127)) / 127.0;
      float cyaw = 10 * ((float) in_yaw) / 127.0;
      float cpitch = 5 * ((float) in_pitch) / 127.0;
      float croll = 5 * ((float) in_roll) / 127.0;
      desired_angle.yaw += cyaw;
      desired_angle.roll = croll;
      desired_angle.pitch = cpitch;
    }
  
    if (debug) {
      SERIALD.print(F(" yaw: "));
      SERIALD.print(yaw);
      SERIALD.print(F(" pitch: "));
      SERIALD.print(pitch);
      SERIALD.print(F(" roll: "));
      SERIALD.print(roll);
      SERIALD.print(" bat: ");
      SERIALD.println(get_battery_voltage());
      if (is_low_voltage()) {
        SERIALD.println("!!! LOW VOLTAGE !!!");
      }
      
    }
    if (debug) {
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
