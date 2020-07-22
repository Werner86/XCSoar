/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Device/Driver/AR62xx.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "NMEA/Info.hpp"
#include "RadioFrequency.hpp"
#include "Thread/Cond.hxx"
#include "Thread/Mutex.hxx"
#include "Util/CharUtil.hxx"
#include "Util/StaticFifoBuffer.hxx"
#include "Util/Compiler.h"
#include "LogFile.hpp"
#include <cstdint>
#include <stdio.h>

/** max command length to send to AR62xx*/
const int MAX_CMD_LEN = 128;

/**
 * 1 = active station of AR62xx
 * 0 = passive station of AR62xx
 */
const int ACTIVE_STATION = 1;
const int PASSIVE_STATION = 0;

/** the lowest frequency which can be set in AR62xx */
const unsigned MIN_FREQUENCY = 118000;

/** the highest frequency which can be set in AR62xx */
const unsigned MAX_FREQUENCY = 137000;


/** the highest frequency id which can be set in AR62xx.
 * this number is used to calculate the particular frequency-id of each frequency.
 * 
 * explanation: the AR62xx uses an integer value for each frequency starting at 118000
 * e.g. frequency-id 0    => 118.000 kHz
 *      frequency-id 1    => 118.005 kHz
 *      frequency-id 3040 => 137.000 kHz
 */
const int MAX_FREQUENCY_ID = 3040;

/** bitmasks to get the frequency out of a frequency-id */
const int FREQUENCY_BITMASK = 0xFFF0;

/** bitmasks to get the channel out of a frequency-id */
const int CHANNEL_BITMASK = 0xF;

//TODO should be removed as we (could) have a byte-order-problem when using this
typedef union {
  uint16_t intVal16;
  uint8_t intVal8[2];
} IntConvertStruct;

class AR62xxDevice final : public AbstractDevice
{
  /** command acknowledged character. */
  static constexpr char ACK = 0x06;

  /** command not acknowledged character */
  static constexpr char NAK = 0x15;

  /** No response received yet. */
  static constexpr char NO_RSP = 0;

private:
  /** Port the radio is connected to. */
  Port &port;

  /** Last response received from the radio. */
  uint8_t response;

  /** active station frequency */
  RadioFrequency active_frequency;

  /** passive (or standby) station frequency */
  RadioFrequency passive_frequency;

  /** flag if we are sending commands to AR62xx
   * if yes we should not read in parallel
   * true = we are currently in send-mode
   * false = we can read from AR62xx
   */
  bool is_sending = false;

  bool Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env);

  uint16_t FrequencyToId(RadioFrequency frequency);

  RadioFrequency IdToFrequency(uint16_t frequency_id);

  int SetStation(uint8_t *command, int active_passive, RadioFrequency frequency, const TCHAR *station);

  void SaveFrequencies(const char *string, size_t len);

  uint8_t GetNthByte(unsigned number, int n);

  unsigned BytesToUnsigned(uint8_t first, uint8_t second);

public:
  explicit AR62xxDevice(Port &_port);

  virtual bool PutActiveFrequency(RadioFrequency frequency,
                                  const TCHAR *name,
                                  OperationEnvironment &env) override;
  virtual bool PutStandbyFrequency(RadioFrequency frequency,
                                   const TCHAR *name,
                                   OperationEnvironment &env) override;
  virtual bool DataReceived(const void *data, size_t length, struct NMEAInfo &info) override;
};

/** constructor */
AR62xxDevice::AR62xxDevice(Port &_port) : port(_port)
{
  /** initilize with ACKNOWLEDGE-FLAG */
  response = ACK;
}

/** get the n-th byte of an unsigned INT */
uint8_t AR62xxDevice::GetNthByte(unsigned number, int n)
{
  return number >> (8 * n) & 0xff;
}

/** build an unsigned-int with two bytes */
unsigned AR62xxDevice::BytesToUnsigned(uint8_t first, uint8_t second)
{
  return second << 8 | first;
}

bool AR62xxDevice::DataReceived(const void *_data, size_t length, struct NMEAInfo &info)
{
  /** check that data is not empty (null) */
  assert(_data != nullptr);

  /** the length of the data has to be greater than zero */
  assert(length > 0);

  /** cast data to int */
  const uint8_t *data = (const uint8_t *)_data;

  /** parse data and save active and passive frequency */
  SaveFrequencies((const char *)data, length);

  /** send nmea info that we are receiving data */
  info.alive.Update(info.clock);

  /** return true which shows we received data */
  return true;
}

bool AR62xxDevice::Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env)
{
  //TODO check if this can be replaced with (and all other be removed)
  //return port.FullWrite(msg, msg_size, env, std::chrono::milliseconds(250));

  /** Number of tries to send a message will be decreased on every retry */
  unsigned retries = 3;

  /** check that msg is not empty */
  assert(msg_size > 0);

  /* Mutex to be locked to access response. */
  Mutex response_mutex;

  /** Condition to signal that a response was received from the radio. */
  Cond rx_cond;

  do
  {
    {
      const std::lock_guard<Mutex> lock(response_mutex);

      /** initialize response with "No response received yet" */
      response = NO_RSP;
    }

    /** set sending-flat */
    is_sending = true;

    /** message NOT sent */
    if (!port.FullWrite(msg, msg_size, env, std::chrono::milliseconds(250)))
    {
      /** if message could not be sent set response to "command not acknowledged character" */
      response = NAK;
    }

    /** message sent */
    else
    {
      /** if message could be sent set response to "command acknowledged character" */
      response = ACK;
    }

    /** Wait for the response */
    uint8_t _response;
    {
      std::unique_lock<Mutex> lock(response_mutex);

      /** wait for the response */
      rx_cond.wait_for(lock, std::chrono::milliseconds(250));
      _response = response;
    }

    /** reset flag is_sending */
    is_sending = false;

    /** ACK received */
    if (_response == ACK)
    {
      /** ACK received, finish, all went well */
      return true;
    }

    /** No ACK received, retry, possibly an error occurred */
    retries--;
  } while (retries);

  /** returns false if message could not be sent */
  return false;
}

RadioFrequency AR62xxDevice::IdToFrequency(uint16_t frequency_id)
{
  unsigned frequency_range = MAX_FREQUENCY - MIN_FREQUENCY;
  unsigned frequency_encoded = frequency_id & FREQUENCY_BITMASK;
  unsigned channel_encoded = frequency_id & CHANNEL_BITMASK;

  unsigned frequency = MIN_FREQUENCY + frequency_encoded * frequency_range / MAX_FREQUENCY_ID;

  if (channel_encoded <= 3)
    frequency += channel_encoded * 5;
  else if (channel_encoded <= 7)
    frequency += channel_encoded * 5 + 5;
  else if (channel_encoded <= 11)
    frequency += channel_encoded * 5 + 10;
  else if (channel_encoded <= 15)
    frequency += channel_encoded * 5 + 15;

  RadioFrequency radio_frequency = RadioFrequency();
  radio_frequency.SetKiloHertz(frequency);

  return radio_frequency;
}

uint16_t AR62xxDevice::FrequencyToId(RadioFrequency radio_frequency)
{
  unsigned frequency = radio_frequency.GetKiloHertz();

  /** check if frequency is not in range */
  if (frequency < MIN_FREQUENCY || frequency > MAX_FREQUENCY)
    return 0;

  unsigned frequency_range = MAX_FREQUENCY - MIN_FREQUENCY;
  unsigned frequency_offset = frequency - MIN_FREQUENCY;

  /** generate frequency-id */
  uint16_t frequency_id = std::round(frequency_offset * MAX_FREQUENCY_ID / frequency_range);

  frequency_id &= FREQUENCY_BITMASK;

  /** get channel out of frequency */
  uint8_t channel = frequency % 100;

  /** encode channel and add to frequency */
  if (channel <= 15)
    frequency_id += channel / 5;
  else if (channel <= 40)
    frequency_id += channel / 5 - 1;
  else if (channel <= 65)
    frequency_id += channel / 5 - 2;
  else if (channel <= 90)
    frequency_id += channel / 5 - 3;

  return (frequency_id);
}

static uint16_t CRCBitwise(uint8_t *data, size_t len)
{
  uint16_t crc = 0x0000;
  size_t j;
  int i;
  for (j = len; j > 0; j--)
  {
    crc ^= (uint16_t)(*data++) << 8;
    for (i = 0; i < 8; i++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x8005;
      else
        crc <<= 1;
    }
  }
  return (crc);
}

int AR62xxDevice::SetStation(uint8_t *command, int active_passive, RadioFrequency frequency, const TCHAR *station)
{
  if (command == NULL || station == NULL)
  {
    return false;
  }

  /** command length */
  unsigned int len = 0;

  uint16_t current_active_frequency = FrequencyToId(active_frequency);
  uint16_t current_passive_frequency = FrequencyToId(passive_frequency);

  //TODO check if this can be removed
  IntConvertStruct ActiveFreqIdx;
  ActiveFreqIdx.intVal16 = current_active_frequency;

  //TODO check if this can be removed
  IntConvertStruct PassiveFreqIdx;
  PassiveFreqIdx.intVal16 = current_passive_frequency;

  /** add header */
  command[len++] = 0xA5;

  /** add prot-ID */
  command[len++] = 0x14;

  command[len++] = 5;

  /** converting the frequency which is to be changed */
  switch (active_passive)
  {
  case ACTIVE_STATION:
    current_active_frequency = FrequencyToId(active_frequency);
    //TODO check if this can be removed
    ActiveFreqIdx.intVal16 = current_active_frequency;
    break;
  default:
  case PASSIVE_STATION:
    current_passive_frequency = FrequencyToId(passive_frequency);
    //TODO check if this can be removed
    PassiveFreqIdx.intVal16 = current_passive_frequency;
    break;
  }

  /** setting frequencies -command byte in the protocol of the radio */
  command[len++] = 22;

  //TODO check if this can be used
  //command[len++] = getByte(current_active_frequency, 1);
  //command[len++] = getByte(current_active_frequency, 0);

  //TODO check if this can be removed
  command[len++] = ActiveFreqIdx.intVal8[1];
  command[len++] = ActiveFreqIdx.intVal8[0];

  //TODO check if this can be used
  //command[len++] = getByte(current_passive_frequency, 1);
  //command[len++] = getByte(current_passive_frequency, 0);

  //TODO check if this can be removed
  command[len++] = PassiveFreqIdx.intVal8[1];
  command[len++] = PassiveFreqIdx.intVal8[0];

  /** Creating the binary value */

  //TODO check if this can be removed
  IntConvertStruct crc;

  uint16_t crc_value = CRCBitwise(command, len);

  //TODO check if this can be removed
  crc.intVal16 = crc_value;

  //TODO check if this can be used
  command[len++] = GetNthByte(crc_value, 1);
  command[len++] = GetNthByte(crc_value, 0);

  //TODO check if this can be removed
  command[len++] = crc.intVal8[1];
  command[len++] = crc.intVal8[0];

  return len;
}

void AR62xxDevice::SaveFrequencies(const char *string, size_t len)
{
  size_t cnt = 0;
  uint16_t calc_crc = 0;
  static uint16_t rec_buf_len = 0;
  int command_length = 0;
#define REC_BUFSIZE 127
  static uint8_t command[REC_BUFSIZE];

  if (string == NULL)
    return;
  if (len == 0)
    return;

  while (cnt < len)
  {
    /** check for header*/
    if ((uint8_t)string[cnt] == 0xA5)
      rec_buf_len = 0;
    if (rec_buf_len >= REC_BUFSIZE)
      rec_buf_len = 0;
    assert(rec_buf_len < REC_BUFSIZE);

    command[rec_buf_len++] = (uint8_t)string[cnt++];
    if (rec_buf_len == 2)
    {
      if (!(command[rec_buf_len - 1] == 0x14))
      {
        rec_buf_len = 0;
      }
    }

    if (rec_buf_len >= 3)
    {
      command_length = command[2];

      /** all received */
      if (rec_buf_len >= (command_length + 5))
      {
        //TODO check if this can be removed
        IntConvertStruct crc;
        crc.intVal8[1] = command[command_length + 3];
        crc.intVal8[0] = command[command_length + 4];
        uint16_t command_crc = crc.intVal16;

        //TODO check if this can be used
        //uint16_t command_crc = bytes2Unsigned(command[CommandLength + 4],command[CommandLength + 3]);

        calc_crc = CRCBitwise(command, command_length + 3);
        if ((calc_crc == command_crc || command_crc == 0) && !is_sending && command != NULL)
        {

          switch ((unsigned char)(command[3] & 0x7F))
          {

          /** Frequency settings, always for both frequencies (active and passive) */
          case 22:
            if (0 != calc_crc)
            {
              uint16_t frequency_uint16;

              //TODO check if this can be removed
              IntConvertStruct frequency;
              frequency.intVal8[1] = command[4];
              frequency.intVal8[0] = command[5];
              frequency_uint16 = frequency.intVal16;

              //TODO check if this can be used
              frequency_uint16 = BytesToUnsigned(command[5], command[4]);

              active_frequency = IdToFrequency(frequency_uint16);

              //TODO check if this can be removed
              frequency.intVal8[1] = command[6];
              frequency.intVal8[0] = command[7];
              frequency_uint16 = frequency.intVal16;

              //TODO check if this can be used
              frequency_uint16 = BytesToUnsigned(command[7], command[6]);

              passive_frequency = IdToFrequency(frequency_uint16);
            }
            break;
          default:
            break;
          }
        }
        rec_buf_len = 0;
      }
    }
  }
}

bool AR62xxDevice::PutActiveFrequency(RadioFrequency frequency,
                                      const TCHAR *name,
                                      OperationEnvironment &env)
{
  uint8_t szTmp[MAX_CMD_LEN];
  int len = SetStation(szTmp, ACTIVE_STATION, frequency, (const TCHAR *)name);
  return Send((uint8_t *)&szTmp, len, env);
}

bool AR62xxDevice::PutStandbyFrequency(RadioFrequency frequency,
                                       const TCHAR *name,
                                       OperationEnvironment &env)
{
  uint8_t szTmp[MAX_CMD_LEN] = {};
  int len = SetStation(szTmp, PASSIVE_STATION, frequency, (const TCHAR *)name);
  return Send((uint8_t *)&szTmp, len, env);
}

static Device *AR62xxCreateOnPort(const DeviceConfig &config, Port &comPort)
{
  return new AR62xxDevice(comPort);
}

const struct DeviceRegister ar62xx_driver = {
    _T("AR62xx"),
    _T("AR62xx"),
    DeviceRegister::NO_TIMEOUT | DeviceRegister::RAW_GPS_DATA,
    AR62xxCreateOnPort,
};