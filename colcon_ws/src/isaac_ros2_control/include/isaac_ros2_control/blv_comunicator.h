/**
* @file blv_comunicator.h
* @brief header file for blvr motor comunication
* @author Masaaki Hijikata <hijikata@ir.utsunomiya-u.ac.jp>, Utsunomiya Univ.
* @date 20210218
* @details  
*/

extern "C" {
#include <time.h>
#include <termios.h>
}
#include <stdint.h>
#include <string>

#ifndef __BLV_COMUNICATOR_HPP__
#define __BLV_COMUNICATOR_HPP__


class BlvComunicator
{
public:
  static constexpr char BLVD_DEFAULT_DEVICE[] = "/dev/tty_mortor";
  static const int BLVD_DEFAULT_FREQUENCY = 10;

  static const int BAUDRATE = B115200;
  static const int BLV_MAX_RPM = 4000;
  static constexpr double BLV_RATED_TORQUE = 0.64; /*Nm*/
  static const int BLV_STEP_TO_ONESHOT = 36000;

  static const int ACCESS_DELAY = 3500; /* us */
  static const int READ_TIMEOUT = 100;

  static const int MSEC = 1000000; //nanosec to msec
  static const int USEC = 1000;    //nanosec to usec

  static constexpr struct timespec BROADCAST_DELAY    = {0,  10 * MSEC};
  static constexpr struct timespec RESPONSE_DELAY     = {0,   5 * MSEC};
  static constexpr struct timespec READ_RETRY_DELAY   = {0,    5 * MSEC};
  static constexpr struct timespec CHARACTER_DELAY    = {0,   10 * USEC};

  static const int MESSAGE_BUF_SIZE = 255;

  enum class return_type : uint8_t
  {
    SUCCESS = 0,
    ERROR = 1
  };
  enum Mode
  {
    NOCONTROL,
    ABSOLUTE_POSITION,
    RELATIVE_POSITION_FROM_TARGET,
    RELATIVE_POSITION_FROM_CURRENT,
    RELATIVE_POSITIONING_SPEEDS_CONTROL_FROM_TARGET = 5,
    RELATIVE_POSITIONING_SPEEDS_CONTROL_FROM_CURRENT,
    CONTINUOUS_OPERATION_BY_RPM = 16,
    CONTINUOUS_OPERATION_BY_PUSH,
    CONTINUOUS_OPERATION_BY_TORQUE,
    PUSHING_OPERATION_FROM_ABSOLUTE_POSITION = 20,
    PUSHING_OPERATION_FROM_TARGET_POSITION,
    PUSHING_OPERATION_FROM_CURRENT_POSITION
  };

  BlvComunicator()
  : is_open(false)
  , blv_port(0) {};
  ~BlvComunicator() {};

  return_type openDevice(std::string& device);
  void        closeDevice();

  int   writeRpm(int ch, int rpm);

  int   readRpm(int ch, int *rpm);
  int   readAlarm(int ch, int *alarm);
  int   readWarning(int ch, int *warning);
  int   readTorque(int ch, int *torque);
  int   resetAlarm(int ch);

  bool is_open;

private:
  uint16_t makeCrc16(uint8_t *p, size_t len);

  int blv_port;
};

#endif /* __BLV_COMUNICATOR_HPP__ */
