#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#include <stdbool.h>
#include <stdint.h>

#include "flash.h"
#include "mavlink.h"
#include "mavlink_param.h"
#include "mavlink_stream.h"

#include "param.h"
#include "mixer.h"
#include "sensors.h"
//#include "rc.h" <-- I want to include this file so I can manually specify the RC type.  But I get errors if I do

// global variable definitions
params_t _params;

// local function definitions
static void init_param_int(param_id_t id, char name[PARAMS_NAME_LENGTH], int32_t value)
{
  memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
  _params.values[id] = value;
  _params.types[id] = PARAM_TYPE_INT32;
}

static void init_param_float(param_id_t id, char name[PARAMS_NAME_LENGTH], float value)
{
  memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
  _params.values[id] = *((int32_t *) &value);
  _params.types[id] = PARAM_TYPE_FLOAT;
}

// function definitions
void init_params(void)
{
  initEEPROM();
  if (!read_params())
  {
    set_param_defaults();
    write_params();
  }

  for (uint16_t id = 0; id < PARAMS_COUNT; id++)
    param_change_callback((param_id_t) id);
}

void set_param_defaults(void)
{
  /******************************/
  /*** HARDWARE CONFIGURATION ***/
  /******************************/
  init_param_int(PARAM_BOARD_REVISION, "BOARD_REV", 2); // Major board revision of naze32/flip32 | 1 | 6
  init_param_int(PARAM_BAUD_RATE, "BAUD_RATE", 921600); // Baud rate of MAVlink communication with onboard computer | 9600 | 921600

  /*****************************/
  /*** MAVLINK CONFIGURATION ***/
  /*****************************/
  init_param_int(PARAM_SYSTEM_ID, "SYS_ID", 1); // Mavlink System ID  | 1 | 255
  init_param_int(PARAM_STREAM_HEARTBEAT_RATE, "STRM_HRTBT", 1); // Rate of heartbeat streaming (Hz) | 0 | 1000

  init_param_int(PARAM_STREAM_ATTITUDE_RATE, "STRM_ATTITUDE", 100); // Rate of attitude stream (Hz) | 0 | 1000
  init_param_int(PARAM_STREAM_IMU_RATE, "STRM_IMU", 500); // Rate of IMU stream (Hz) | 0 | 1000


  /*******************************/
  /*** ESTIMATOR CONFIGURATION ***/
  /*******************************/
  init_param_int(PARAM_INIT_TIME, "FILTER_INIT_T", 3000); // Time in ms to initialize estimator | 0 | 100000
  init_param_float(PARAM_FILTER_KP, "FILTER_KP", 1.0f); // estimator proportional gain - See estimator documentation | 0 | 10.0
  init_param_float(PARAM_FILTER_KI, "FILTER_KI", 0.1f); // estimator integral gain - See estimator documentation | 0 | 1.0

  init_param_float(PARAM_GYRO_ALPHA, "GYRO_LPF_ALPHA", 0.888f); // Low-pass filter constant - See estimator documentation | 0 | 1.0
  init_param_float(PARAM_ACC_ALPHA, "ACC_LPF_ALPHA", 0.888f); // Low-pass filter constant - See estimator documentation | 0 | 1.0

  init_param_float(PARAM_ACCEL_SCALE, "ACCEL_SCALE", 1.0f); // Scale factor to apply to IMU measurements - Read-Only | 0.5 | 2.0

  init_param_float(PARAM_GYRO_X_BIAS, "GYRO_X_BIAS", 0.0f); // Constant x-bias of gyroscope readings | -1.0 | 1.0
  init_param_float(PARAM_GYRO_Y_BIAS, "GYRO_Y_BIAS", 0.0f); // Constant y-bias of gyroscope readings | -1.0 | 1.0
  init_param_float(PARAM_GYRO_Z_BIAS, "GYRO_Z_BIAS", 0.0f); // Constant z-bias of gyroscope readings | -1.0 | 1.0
  init_param_float(PARAM_ACC_X_BIAS,  "ACC_X_BIAS", 0.0f); // Constant x-bias of accelerometer readings | -2.0 | 2.0
  init_param_float(PARAM_ACC_Y_BIAS,  "ACC_Y_BIAS", 0.0f); // Constant y-bias of accelerometer readings | -2.0 | 2.0
  init_param_float(PARAM_ACC_Z_BIAS,  "ACC_Z_BIAS", 0.0f); // Constant z-bias of accelerometer readings | -2.0 | 2.0
  init_param_float(PARAM_ACC_X_TEMP_COMP,  "ACC_X_TEMP_COMP", 0.0f); // Linear x-axis temperature compensation constant | -2.0 | 2.0
  init_param_float(PARAM_ACC_Y_TEMP_COMP,  "ACC_Y_TEMP_COMP", 0.0f); // Linear y-axis temperature compensation constant | -2.0 | 2.0
  init_param_float(PARAM_ACC_Z_TEMP_COMP,  "ACC_Z_TEMP_COMP", 0.0f); // Linear z-axis temperature compensation constant | -2.0 | 2.0

  /****************************/
  /*** CAMERA CONFIGURATION ***/
  /****************************/
  init_param_float(PARAM_CAMERA_FRAME_RATE, "CAMERA_FRAME_RATE", 28.0);
}

bool read_params(void)
{
  return readEEPROM();
}

bool write_params(void)
{
  return writeEEPROM();
}

void param_change_callback(param_id_t id)
{
  switch (id)
  {
  case PARAM_SYSTEM_ID:
    mavlink_system.sysid = get_param_int(PARAM_SYSTEM_ID);
    break;
  case PARAM_STREAM_HEARTBEAT_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_HEARTBEAT, get_param_int(PARAM_STREAM_HEARTBEAT_RATE));
    break;

  case PARAM_STREAM_ATTITUDE_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_ATTITUDE, get_param_int(PARAM_STREAM_ATTITUDE_RATE));
    break;

  case PARAM_STREAM_IMU_RATE:
    mavlink_stream_set_rate(MAVLINK_STREAM_ID_IMU, get_param_int(PARAM_STREAM_IMU_RATE));
    break;

  case PARAM_CAMERA_FRAME_RATE:
    set_framerate();
    break;

  default:
    // no action needed for this parameter
    break;
  }
}

param_id_t lookup_param_id(const char name[PARAMS_NAME_LENGTH])
{
  for (uint16_t id = 0; id < PARAMS_COUNT; id++)
  {
    bool match = true;
    for (uint8_t i = 0; i < PARAMS_NAME_LENGTH; i++)
    {
      // compare each character
      if (name[i] != _params.names[id][i])
      {
        match = false;
        break;
      }

      // stop comparing if end of string is reached
      if (_params.names[id][i] == '\0')
        break;
    }

    if (match)
      return (param_id_t) id;
  }

  return PARAMS_COUNT;
}

int get_param_int(param_id_t id)
{
  return _params.values[id];
}

float get_param_float(param_id_t id)
{
  return *(float *) &_params.values[id];
}

char *get_param_name(param_id_t id)
{
  return _params.names[id];
}

param_type_t get_param_type(param_id_t id)
{
  return _params.types[id];
}

bool set_param_int(param_id_t id, int32_t value)
{
  if (id < PARAMS_COUNT && value != _params.values[id])
  {
    _params.values[id] = value;
    param_change_callback(id);
    mavlink_send_param(id);
    return true;
  }
  return false;
}

bool set_param_float(param_id_t id, float value)
{
  return set_param_int(id, *(int32_t *) &value);
}

bool set_param_by_name_int(const char name[PARAMS_NAME_LENGTH], int32_t value)
{
  param_id_t id = lookup_param_id(name);
  return set_param_int(id, value);
}

bool set_param_by_name_float(const char name[PARAMS_NAME_LENGTH], float value)
{
  return set_param_by_name_int(name, *(int32_t *) &value);
}
