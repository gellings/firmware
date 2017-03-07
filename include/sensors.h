#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <turbotrig/turbovec.h>

#include <stdint.h>
#include <stdbool.h>

// global variable declarations
extern vector_t _accel;
extern vector_t _gyro;
extern float _imu_temperature;
extern uint64_t _imu_time;

extern bool _image_associated;


// function declarations
void init_sensors(void);
bool update_sensors();

void set_framerate(void);

bool start_imu_calibration(void);
bool start_gyro_calibration(void);
void start_baro_calibration(void);
void start_airspeed_calibration(void);
bool gyro_calibration_complete(void);

#ifdef __cplusplus
}
#endif
