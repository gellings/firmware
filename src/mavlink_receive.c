#include <breezystm32/breezystm32.h>

#include "mavlink.h"
#include "mavlink_param.h"
#include "mode.h"
#include "param.h"
#include "mux.h"
#include "sensors.h"
#include "rc.h"

#include "mavlink_receive.h"
#include "mavlink_log.h"

#include "mavlink_util.h"

// global variable definitions
mavlink_offboard_control_t mavlink_offboard_control;
uint64_t _offboard_control_time;

// local variable definitions
static mavlink_message_t in_buf;
static mavlink_status_t status;

// local function definitions
static void mavlink_handle_msg_rosflight_cmd(const mavlink_message_t *const msg)
{
  mavlink_rosflight_cmd_t cmd;
  mavlink_msg_rosflight_cmd_decode(msg, &cmd);

  uint8_t result;
  bool reboot_flag = false;
  bool reboot_to_bootloader_flag = false;

  // None of these actions can be performed if we are armed
  result = true;
  switch (cmd.command)
  {
  case ROSFLIGHT_CMD_READ_PARAMS:
    result = read_params();
    break;
  case ROSFLIGHT_CMD_WRITE_PARAMS:
    result = write_params();
    break;
  case ROSFLIGHT_CMD_SET_PARAM_DEFAULTS:
    set_param_defaults();
    break;
  case ROSFLIGHT_CMD_ACCEL_CALIBRATION:
    result = start_imu_calibration();
    break;
  case ROSFLIGHT_CMD_GYRO_CALIBRATION:
    result = start_gyro_calibration();
    break;
  case ROSFLIGHT_CMD_REBOOT:
    reboot_flag = true;
    break;
  case ROSFLIGHT_CMD_REBOOT_TO_BOOTLOADER:
    reboot_to_bootloader_flag = true;
    break;
  default:
    mavlink_log_error("unsupported ROSFLIGHT CMD %d", cmd.command);
    result = false;
    break;
  }
  uint8_t response = (result) ? ROSFLIGHT_CMD_SUCCESS : ROSFLIGHT_CMD_FAILED;

  mavlink_msg_rosflight_cmd_ack_send(MAVLINK_COMM_0, cmd.command, response);

  if (reboot_flag || reboot_to_bootloader_flag)
  {
    delay(20);
    systemReset(reboot_to_bootloader_flag);
  }
}

static void mavlink_handle_msg_timesync(const mavlink_message_t *const msg)
{
  uint64_t now_us = micros();

  mavlink_timesync_t tsync;
  mavlink_msg_timesync_decode(msg, &tsync);

  if (tsync.tc1 == 0) // check that this is a request, not a response
  {
    mavlink_msg_timesync_send(MAVLINK_COMM_0, (int64_t) now_us*1000, tsync.ts1);
  }
}

static void handle_mavlink_message(void)
{
  switch (in_buf.msgid)
  {
  case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    mavlink_handle_msg_param_request_list();
    break;
  case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    mavlink_handle_msg_param_request_read(&in_buf);
    break;
  case MAVLINK_MSG_ID_PARAM_SET:
    mavlink_handle_msg_param_set(&in_buf);
    break;
  case MAVLINK_MSG_ID_ROSFLIGHT_CMD:
    mavlink_handle_msg_rosflight_cmd(&in_buf);
    break;
  case MAVLINK_MSG_ID_TIMESYNC:
    mavlink_handle_msg_timesync(&in_buf);
    break;
  default:
    break;
  }
}

// function definitions
void mavlink_receive(void)
{
  while (serialTotalBytesWaiting(Serial1))
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, serialRead(Serial1), &in_buf, &status))
      handle_mavlink_message();
  }
}

