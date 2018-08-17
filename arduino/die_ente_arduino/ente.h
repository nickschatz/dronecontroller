#ifndef _ENTE_H
#define _ENTE_H

#define STATUS_LED PC13
#define VIN_READ PA5
#define CAL_COUNTS 50
#define MIN_THROTTLE 0.2
#define TETHER_CMD 1
// 13.2 / 4095
// Small fudge for variances in resistors
#define VOLTAGE_SCALE 0.00322344322
#define STOP B11111111
#define DAVG_COUNT 50
#define RAD_TO_DEG 57.2957795
#define DEG_TO_RAD 0.017453292

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

struct motors {
  float fr;
  float fl;
  float rl;
  float rr;
};

#endif
