/**
 * @file shm_communicator.c
 * @brief source file for shared memory communication
 * @author Masaaki Hijikata <hijimasa@gmail.com>
 * @date 20240214
 */

extern "C"
{
#include <stdio.h>
#include <fcntl.h> /* For O_RDWR */
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>
#include <string.h>
}
#include <string>

#include "isaac_ros2_control/shm_comunicator.h"

/**
 * @fn open
 * @param [in] *device
 *        device name
 * @return status
 * @retval 0 success
 * @retval -1 failure
 */
ShmComunicator::return_type
ShmComunicator::openMemory(std::string name)
{
  shm_fd = shm_open((std::string(SHM_DEFAULT_NAME) + std::string("_") + name).c_str(), O_RDWR | O_CREAT, static_cast<mode_t>(DEFAULT_PERM));
  if (shm_fd < 0)
  {
    return return_type::ERROR;
  }
  struct stat stat;
  fstat(shm_fd, &stat);
  if (static_cast<size_t>(stat.st_size) < shm_size)
  {
    if (ftruncate(shm_fd, shm_size) < 0)
    {
      return return_type::ERROR;
    }
    // To Update stat.st_size
    fstat(shm_fd, &stat);
  }
  shm_ptr = reinterpret_cast<unsigned char *>(mmap(NULL,
                                                   stat.st_size,
                                                   PROT_READ | PROT_WRITE,
                                                   MAP_SHARED,
                                                   shm_fd,
                                                   0));

  return return_type::SUCCESS;
}

/**
 * @fn close
 * @return status
 * @retval 0 success
 * @retval -1 failure
 */
void ShmComunicator::closeMemory()
{
}

/**
 * @fn writeRadps
 * @
 */
int ShmComunicator::writeRadps(int ch, float rad)
{
  reinterpret_cast<float *>(shm_ptr)[4 * ch] = rad;

  return 0;
}

/**
 * @fn readRad
 *
 */
int ShmComunicator::readRad(int ch, float *rad)
{
  *rad = reinterpret_cast<float *>(shm_ptr)[4 * ch + 1];

  return 0;
}

/**
 * @fn readRadps
 *
 */
int ShmComunicator::readRadps(int ch, float *radps)
{
  *radps = reinterpret_cast<float *>(shm_ptr)[4 * ch + 2];

  return 0;
}

/**
 * @fn readTorque
 *
 */
int ShmComunicator::readTorque(int ch, float *torque)
{
  *torque = reinterpret_cast<float *>(shm_ptr)[4 * ch + 3];

  return 0;
}
