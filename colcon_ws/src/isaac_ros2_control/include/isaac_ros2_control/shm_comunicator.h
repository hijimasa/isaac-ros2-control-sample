/**
 * @file shm_comunicator.h
 * @brief header file for shared memory comunication
 * @author Masaaki Hijikata <hijimasa@gmail.com>
 * @date 20240214
 * @details
 */

extern "C"
{
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
}
#include <stdint.h>
#include <string>

#ifndef __SHM_COMUNICATOR_HPP__
#define __SHM_COMUNICATOR_HPP__

enum PERM : mode_t
{
  PERM_USER_READ = S_IRUSR,   /*!<
                               * \~english     Owner readable
                               * \~japanese-en 所有者の読み込み許可
                               */
  PERM_USER_WRITE = S_IWUSR,  /*!<
                               * \~english     Owner writable
                               * \~japanese-en 所有者の書き込み許可
                               */
  PERM_GROUP_READ = S_IRGRP,  /*!<
                               * \~english     Group that owner belong readable
                               * \~japanese-en 所有者のグループの読み込み許可
                               */
  PERM_GROUP_WRITE = S_IWGRP, /*!<
                               * \~english     Group that owner belong writable
                               * \~japanese-en 所有者のグループの書き込み許可
                               */
  PERM_OTHER_READ = S_IROTH,  /*!<
                               * \~english     Others readable
                               * \~japanese-en その他の読み込み許可
                               */
  PERM_OTHER_WRITE = S_IWOTH, /*!<
                               * \~english     Others writable
                               * \~japanese-en その他の書き込み許可
                               */
};
const PERM DEFAULT_PERM = static_cast<PERM>(PERM_USER_READ | PERM_USER_WRITE | PERM_GROUP_READ | PERM_GROUP_WRITE | PERM_OTHER_READ | PERM_OTHER_WRITE);

class ShmComunicator
{
public:
  static constexpr char SHM_DEFAULT_NAME[] = "isaac_ros2_control_data";
  static const int INT32_SIZE = 4;
  static const int MOTOR_NUM = 256;

  enum class return_type : uint8_t
  {
    SUCCESS = 0,
    ERROR = 1
  };

  ShmComunicator()
      : shm_size(4 * INT32_SIZE * MOTOR_NUM){};
  ~ShmComunicator(){};

  return_type openMemory(std::string name);
  void closeMemory();

  int writeRadps(int ch, float radps);

  int readRad(int ch, float *rad);
  int readRadps(int ch, float *radps);
  int readTorque(int ch, float *torque);

private:
  int shm_fd;
  size_t shm_size;
  unsigned char *shm_ptr;
};

#endif /* __SHM_COMUNICATOR_HPP__ */
