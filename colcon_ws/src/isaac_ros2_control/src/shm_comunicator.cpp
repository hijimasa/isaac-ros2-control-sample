/**
* @file blvd.c
* @brief source file for blvd
* @author Chris Takahashi <takahashi@ieat-fresh.com>, i-eat Co., Ltd.
* @date 20210218
* @details 
* RS485(Modbus) command I/F for the BLV series, Oriental Motor Co.Ltd.
*/

extern "C" {
#include <stdio.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>
#include <string.h>
}
#include <string>

#include "isaac_ros2_control/shm_comunicator.h"

constexpr struct timespec BlvComunicator::BROADCAST_DELAY;
constexpr struct timespec BlvComunicator::RESPONSE_DELAY;
constexpr struct timespec BlvComunicator::READ_RETRY_DELAY;
constexpr struct timespec BlvComunicator::CHARACTER_DELAY;

uint16_t
BlvComunicator::makeCrc16(uint8_t *p, size_t len)
{
  uint16_t crc = 0xffff;

  for (size_t i = 0; i < len; i++) {
    crc = crc ^ (uint16_t)p[i];
    for (size_t read_status = 0; read_status < 8; read_status++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0xa001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/**
 * @fn open
 * @param [in] *device
 *        device name
 * @return status
 * @retval 0 success
 * @retval -1 failure
 */
BlvComunicator::return_type
BlvComunicator::openDevice(std::string& device)
{
  if ((blv_port = open(device.c_str(), O_RDWR | O_NONBLOCK)) < 0) {
    fprintf(stderr, "%s: can't open port(%s)\n", __func__, device.c_str());
    return return_type::ERROR;
  }

  struct termios p;
  tcgetattr(blv_port, &p);

  memset(&p, 0, sizeof (struct termios));
  p.c_iflag = IGNPAR;
  p.c_cflag = BAUDRATE | CS8 | CREAD | CLOCAL | PARENB;
  p.c_lflag = p.c_lflag & ~ICANON;

  tcsetattr(blv_port, TCSAFLUSH, const_cast<const termios*>(&p));

  is_open = true;

  return return_type::SUCCESS;
}

/**
 * @fn close
 * @return status
 * @retval 0 success
 * @retval -1 failure
 */
void
BlvComunicator::closeDevice()
{
  close(blv_port);
  is_open = false;
}

/**
 * @fn writeRpm
 * @
 */
int
BlvComunicator::writeRpm(int ch, int rpm)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE];

  unsigned short query_crc, response_crc;
  int write_status, read_status, communicate_check;
  int wait_counter;
  unsigned char c;
  int response_length;
  int flush_status = 0;


  flush_status = tcflush(blv_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  if(rpm < -BLV_MAX_RPM)
  {
    rpm = -BLV_MAX_RPM;
  }
  else if( rpm > BLV_MAX_RPM)
  {
    rpm = BLV_MAX_RPM;
  }

  //Below is writing direction
  int direction_data;
  if (rpm == 0)
  {
    direction_data = 0x0000;
  }
  else if (rpm > 0)
  {
    direction_data = 0x0008;
  }
  else
  {
    direction_data = 0x0010;
  }
  query[0] = (unsigned char)ch;
  query[1] = 0x10;
  query[2] = 0x00;
  query[3] = 0x7c;
  query[4] = 0x00;
  query[5] = 0x01;
  query[6] = 0x02;
  query[7] = (direction_data  >> 8) & 0x000000FF;
  query[8] = direction_data & 0x000000FF;

  query_crc = makeCrc16( query, 6);
  query[9] = (unsigned char)( query_crc & 0xff);
  query[10] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blv_port, query, 8);

  if( write_status < 37 )
  {
    fprintf(stderr,"%s:%s:%d write query is failed (ch: %d)\n",
                    __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if(query[0] == 0x00) // query is broadcast. NO responce
  {
    nanosleep(&BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep(&RESPONSE_DELAY, NULL);

  read_status = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if( read(blv_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr,"%s:%s:%d: read response timeout. (ch: %d)\n",
                      __FILE__, __func__, __LINE__, ch);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
      wait_counter = 0;
    }
  }
  
  if(response[1] == 0x90)
  {
    response_length = 5;
  }
  else
  {
    response_length = 8;
  }

  while(read_status < response_length)
  {
    if( read(blv_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr,"%s:%s:%d: read response timeout. (ch: %02x, response[0]: %02x, respomse[1]: %02x)\n",
                      __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter ++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
      //printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status-2] != (unsigned char)(response_crc & 0xff) ||  response[read_status-1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr,"%s:%s*%d: responce crc error. (crc_low = %02x, responce[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                    __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 * 1) & 0xff), response[read_status-1]);
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Get exception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  communicate_check = 0;
  for(int i = 0; i < read_status-2; i++)
  {
    if( query[i] != response[i])
    {
      communicate_check ++;
    }
  }

  if(communicate_check > 0)
  {
    fprintf(stderr, "%s:%s:%d: communicate was fail\n", __FILE__, __func__, __LINE__);
    fprintf(stderr, "query  responce\n");

    for(int i = 0; i < read_status-2; i++)
    {
      fprintf(stderr, "   %02x        %02x\n", query[i], response[i]);
    }
    return -1;
  }
  nanosleep(&RESPONSE_DELAY, NULL);


  // Below is writing rpm
  if (rpm < 0)
  {
    rpm = -1 * rpm;
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x06;
  query[2] = 0x04;
  query[3] = 0x80;
  query[4] = 0x00;
  query[5] = 0x01;
  query[6] = 0x02;
  query[7] = (rpm  >> 8) & 0x000000FF;
  query[8] = rpm & 0x000000FF;

  query_crc = makeCrc16( query, 6);
  query[9] = (unsigned char)( query_crc & 0xff);
  query[10] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blv_port, query, 8);

  if( write_status < 37 )
  {
    fprintf(stderr,"%s:%s:%d write query is failed (ch: %d)\n",
                    __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if(query[0] == 0x00) // query is broadcast. NO responce
  {
    nanosleep(&BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep(&RESPONSE_DELAY, NULL);

  read_status = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if( read(blv_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr,"%s:%s:%d: read response timeout. (ch: %d)\n",
                      __FILE__, __func__, __LINE__, ch);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
      wait_counter = 0;
    }
  }
  
  if(response[1] == 0x90)
  {
    response_length = 5;
  }
  else
  {
    response_length = 8;
  }

  while(read_status < response_length)
  {
    if( read(blv_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr,"%s:%s:%d: read response timeout. (ch: %02x, response[0]: %02x, respomse[1]: %02x)\n",
                      __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter ++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
      //printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status-2] != (unsigned char)(response_crc & 0xff) ||  response[read_status-1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr,"%s:%s*%d: responce crc error. (crc_low = %02x, responce[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                    __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 * 1) & 0xff), response[read_status-1]);
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Get exception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  communicate_check = 0;
  for(int i = 0; i < read_status-2; i++)
  {
    if( query[i] != response[i])
    {
      communicate_check ++;
    }
  }

  if(communicate_check > 0)
  {
    fprintf(stderr, "%s:%s:%d: communicate was fail\n", __FILE__, __func__, __LINE__);
    fprintf(stderr, "query  responce\n");

    for(int i = 0; i < read_status-2; i++)
    {
      fprintf(stderr, "   %02x        %02x\n", query[i], response[i]);
    }
    return -1;
  }
  nanosleep(&RESPONSE_DELAY, NULL);

  return 0;
}


/**
 * @fn readAlarm
 * 
 */
int
BlvComunicator::readAlarm(int ch, int *alarm)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter;
  int response_length;

  int flush_status = 0;

  flush_status = tcflush(blv_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x03;
  query[2] = 0x00;
  query[3] = 0xAC;
  query[4] = 0x00;
  query[5] = 0x02;

  query_crc = makeCrc16( query, 6);
  query[6] = (unsigned char)( query_crc & 0xff);
  query[7] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blv_port, query, 8);

  if( write_status < 8)
  {
    fprintf(stderr, "%s:%s:%d: write query is failed (ch: %d)\n", __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if( query[0] == 0x00) // query is broadcast. NO responce.
  {
    nanosleep( &BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep ( &BROADCAST_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if(read(blv_port, &c, 1) < 1 )
    {
      if( wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n", __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }

  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
                                                      //read_status, response[0], response[1]);

  if(response[1] == 0x83)
  {
    response_length = 5;
  }
  else
  {
    response_length = 9;
  }

  while (read_status < response_length)
  {
    if(read(blv_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read respons timeout. (ch: %02x, response[0]: %02x response[1]: %02x)\n",
                          __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
      //printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status - 2] != (unsigned char)(response_crc&0xff) || response[read_status - 1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr, "%s:%s:%d: responce crc error. crc_raw = %02x, response[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                      __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 *1) & 0xff), response[read_status-1] );
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Getexception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  if(response[2] != 0x04)
  {
    fprintf(stderr, "%s:%s:%d: Byte Length Error. (Byte Lenght: %02x)\n", __FILE__, __func__, __LINE__, response[2]);
    tcflush(blv_port, TCIFLUSH);
    return -1;
  }
  
  unsigned short high_data, low_data;

  high_data = (response[3] << 8) + response[4];
  low_data  = (response[5] << 8) + response[6];

  *alarm = (high_data << 16) + low_data;

  return 0;

}

/**
 * @fn readRpm
 * 
 */
int
BlvComunicator::readRpm(int ch, int *rpm)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter;
  int response_length;

  int flush_status = 0;

  flush_status = tcflush(blv_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x03;
  query[2] = 0x00;
  query[3] = 0xCE;
  query[4] = 0x00;
  query[5] = 0x02;

  query_crc = makeCrc16( query, 6);
  query[6] = (unsigned char)( query_crc & 0xff);
  query[7] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blv_port, query, 8);

  if( write_status < 8)
  {
    fprintf(stderr, "%s:%s:%d: write query is failed (ch: %d)\n", __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if( query[0] == 0x00) // query is broadcast. NO responce.
  {
    nanosleep( &BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep ( &BROADCAST_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if(read(blv_port, &c, 1) < 1 )
    {
      if( wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n", __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }

  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
                                                      //read_status, response[0], response[1]);

  if(response[1] == 0x83)
  {
    response_length = 5;
  }
  else
  {
    response_length = 9;
  }

  while (read_status < response_length)
  {
    if(read(blv_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read respons timeout. (ch: %02x, response[0]: %02x response[1]: %02x)\n",
                          __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
     // printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status - 2] != (unsigned char)(response_crc&0xff) || response[read_status - 1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr, "%s:%s:%d: responce crc error. crc_raw = %02x, response[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                      __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 *1) & 0xff), response[read_status-1] );
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Getexception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  if(response[2] != 0x04)
  {
    fprintf(stderr, "%s:%s:%d: Byte Length Error. (Byte Lenght: %02x)\n", __FILE__, __func__, __LINE__, response[2]);
    tcflush(blv_port, TCIFLUSH);
    return -1;
  }
  
  unsigned short high_data, low_data;

  high_data = (response[3] << 8) + response[4];
  low_data  = (response[5] << 8) + response[6];
  *rpm = (high_data << 16) + low_data;

  return 0;

}

/**
 * @fn readTorque
 * 
 */
int
BlvComunicator::readTorque(int ch, int *torque)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter;
  int response_length;

  int flush_status = 0;

  flush_status = tcflush(blv_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x03;
  query[2] = 0x01;
  query[3] = 0x08;
  query[4] = 0x00;
  query[5] = 0x02;

  query_crc = makeCrc16( query, 6);
  query[6] = (unsigned char)( query_crc & 0xff);
  query[7] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blv_port, query, 8);

  if( write_status < 8)
  {
    fprintf(stderr, "%s:%s:%d: write query is failed (ch: %d)\n", __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if( query[0] == 0x00) // query is broadcast. NO responce.
  {
    nanosleep( &BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep ( &BROADCAST_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if(read(blv_port, &c, 1) < 1 )
    {
      if( wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n", __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }

  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
                                                      //read_status, response[0], response[1]);

  if(response[1] == 0x83)
  {
    response_length = 5;
  }
  else
  {
    response_length = 9;
  }

  while (read_status < response_length)
  {
    if(read(blv_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read respons timeout. (ch: %02x, response[0]: %02x response[1]: %02x)\n",
                          __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
      //printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status - 2] != (unsigned char)(response_crc&0xff) || response[read_status - 1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr, "%s:%s:%d: responce crc error. crc_raw = %02x, response[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                      __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 *1) & 0xff), response[read_status-1] );
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Getexception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  if(response[2] != 0x04)
  {
    fprintf(stderr, "%s:%s:%d: Byte Length Error. (Byte Lenght: %02x)\n", __FILE__, __func__, __LINE__, response[2]);
    tcflush(blv_port, TCIFLUSH);
    return -1;
  }
  
  unsigned short high_data, low_data;

  high_data = (response[3] << 8) + response[4];
  low_data  = (response[5] << 8) + response[6];
  *torque = (high_data << 16) + low_data;

  return 0;

}

/**
 * @fn readWarinig
 * 
 */
int
BlvComunicator::readWarning(int ch, int *warning)
{
  unsigned char query[MESSAGE_BUF_SIZE], response[MESSAGE_BUF_SIZE], c;
  unsigned short query_crc, response_crc;
  int write_status, read_status, wait_counter;
  int response_length;

  int flush_status = 0;

  flush_status = tcflush(blv_port, TCIOFLUSH);
  if( flush_status < 0 )
  {
    printf("flush status %d\n", flush_status);
  }

  query[0] = (unsigned char)ch;
  query[1] = 0x03;
  query[2] = 0x00;
  query[3] = 0x80;
  query[4] = 0x00;
  query[5] = 0x02;

  query_crc = makeCrc16( query, 6);
  query[6] = (unsigned char)( query_crc & 0xff);
  query[7] = (unsigned char)( query_crc >> (8 * 1) & 0xff);

  write_status = write(blv_port, query, 8);

  if( write_status < 8)
  {
    fprintf(stderr, "%s:%s:%d: write query is failed (ch: %d)\n", __FILE__, __func__, __LINE__, ch);
    return -1;
  }

  if( query[0] == 0x00) // query is broadcast. NO responce.
  {
    nanosleep( &BROADCAST_DELAY, NULL);
    return 0;
  }

  nanosleep ( &BROADCAST_DELAY, NULL);

  read_status  = 0;
  wait_counter = 0;
  while ( read_status < 2 )
  {
    if(read(blv_port, &c, 1) < 1 )
    {
      if( wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read response timeout. (ch: %d, read_status:%d)\n", __FILE__, __func__, __LINE__, ch, read_status);
        return -1;
      }
      wait_counter ++;
      nanosleep(&READ_RETRY_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      //printf("read char = %02x\n", c);
      read_status++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
  }

  //printf("read 2 char (read_status = %d) res[0]:%02x res[1]:%02x \n",
                                                      //read_status, response[0], response[1]);

  if(response[1] == 0x83)
  {
    response_length = 5;
  }
  else
  {
    response_length = 9;
  }

  while (read_status < response_length)
  {
    if(read(blv_port, &c, 1) < 1)
    {
      if(wait_counter > READ_TIMEOUT)
      {
        fprintf(stderr, "%s:%s:%d: read respons timeout. (ch: %02x, response[0]: %02x response[1]: %02x)\n",
                          __FILE__, __func__, __LINE__, ch, response[0], response[1]);
        return -1;
      }
      wait_counter++;
      nanosleep(&CHARACTER_DELAY, NULL);
    }
    else
    {
      response[read_status] = c;
      read_status++;
      wait_counter = 0;
      //printf("read char %d\n", read_status);
    }
    nanosleep(&CHARACTER_DELAY, NULL);
  }

  response_crc = makeCrc16(response, read_status-2);
  if(response[read_status - 2] != (unsigned char)(response_crc&0xff) || response[read_status - 1] != (unsigned char)(response_crc >> (8 * 1) & 0xff))
  {
    fprintf(stderr, "%s:%s:%d: responce crc error. crc_raw = %02x, response[read_status-2] = %02x, crc_high = %02x, response[read_status-1] = %02x\n",
                      __FILE__, __func__, __LINE__, (unsigned char)(response_crc & 0xff), response[read_status-2], (unsigned char)(response_crc >> (8 *1) & 0xff), response[read_status-1] );
    return -1;
  }

  if(response[1] == 0x90)
  {
    fprintf(stderr, "%s:%s:%d: Getexception response. (exception code: %02x, data: %02x)\n", __FILE__, __func__, __LINE__, response[1], response[2]);
    return -1;
  }

  if(response[2] != 0x04)
  {
    fprintf(stderr, "%s:%s:%d: Byte Length Error. (Byte Lenght: %02x)\n", __FILE__, __func__, __LINE__, response[2]);
    tcflush(blv_port, TCIFLUSH);
    return -1;
  }
  
  unsigned short high_data, low_data;

  high_data = (response[3] << 8) + response[4];
  low_data  = (response[5] << 8) + response[6];
  *warning = (high_data << 16) + low_data;

  return 0;

}

