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
 * active / passive station
 * the ar62xx can hold 2 stations in parallel
 * 1 = active station of AR62xx
 * 0 = passive station of AR62xx
 */
const int ACTIVE_STATION = 1;
const int PASSIVE_STATION = 0;

/** the lowest frequency which can be set in AR62xx */
const unsigned MIN_FREQUENCY = 118000;

/** the highest frequency which can be set in AR62xx */
const unsigned MAX_FREQUENCY = 137000;

/** the range of all available frequencies */
const unsigned FREQUENCY_RANGE = MAX_FREQUENCY - MIN_FREQUENCY;

/** 
 * max frequency id
 * the highest frequency ID which can be set in AR62xx.
 * this number is used to calculate the particular frequency-id of each frequency.
 * 
 * explanation: the AR62xx uses an integer value for each 8.33/25 kHz frequency starting at 118.000 kHz
 * e.g. frequency-id 0    => 118.000 kHz
 *      frequency-id 1    => 118.005 kHz
 *      frequency-id 3040 => 137.000 kHz
 */
const int MAX_FREQUENCY_ID = 3040;

/** bitmasks to get the frequency out of a frequency-id */
const int FREQUENCY_BITMASK = 0xFFF0;

/** bitmasks to get the channel out of a frequency-id */
const int CHANNEL_BITMASK = 0xF;

class AR62xxDevice final : public AbstractDevice
{

public:
  explicit AR62xxDevice(Port &_port);

private:
  /** Port the radio is connected to. */
  Port &port;

  /** active station frequency*/
  RadioFrequency active_frequency;

  /** passive (or standby) station frequency*/
  RadioFrequency passive_frequency;

  /**
   * @brief Send - Sends a message to the AR62xx
   * 
   * @param msg message
   * @param msg_size size of the message
   * @param env operation environment
   * @return true if message has been sent
   * @return false if message could not be sent
   */
  bool Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env);

  /**
   * @brief converts a frequency to an AR62xx-ID
   * 
   * explanation: the AR62xx uses an integer value for each 8.33/25 kHz frequency starting at 118.000 kHz
   * e.g. frequency-id 0    => 118.000 kHz
   *      frequency-id 1    => 118.005 kHz
   *      frequency-id 3040 => 137.000 kHz
   * 
   * @param frequency frequency in kHz
   * @return uint16_t frequency-id
   */
  uint16_t FrequencyToAR62xxId(RadioFrequency frequency);

  /**
   * @brief converts an AR62xx-ID to a real frequency in kHz
   * 
   * @param frequency_id frequency-id
   * @return RadioFrequency frequency in kHz
   */
  RadioFrequency AR62xxIdToFrequency(uint16_t frequency_id);

  /**
   * @brief generates a command to set a station in the AR62xx
   * 
   * @param command command-output to set the station
   * @param station_id the id of the station which should be set
   *        1 = active station of AR62xx
   *        0 = passive station of AR62xx
   * @param frequency frequency in kHz to set
   * @param station name of the station to set
   *        (unused)
   * @return int command-length
   */
  int GenerateSetStationCommand(uint8_t *command, int station_id, RadioFrequency frequency, const TCHAR *station);

  /**
   * @brief parses the received data
   * this is needed to save the active and passive station
   * 
   * @param string recived data
   * @param len length of data
   * @return true if data could be parsed
   * @return false if data could not be parsed
   */
  bool ParseReceivedData(const char *string, size_t len);

  /**
   * @brief save frequencies
   * 
   * @param sz_command command which contains the frequencies
   * @param len length of the command
   * @param crc CRC
   */
  void SaveAR62xxFrequencies(uint8_t *sz_command, int len, uint16_t crc);

  /**
   * @brief Get the n-th Byte of an unsigned integer
   * 
   * @param number unsigned integer
   * @param n byte-number
   * @return uint8_t byte
   */
  uint8_t GetNthByte(unsigned number, int n);

  /**
   * @brief Converts to uint8_t-bytes to an unsigned integer
   * 
   * @param first first byte
   * @param second second byte
   * @return unsigned unsigned integer
   */
  unsigned BytesToUnsigned(uint8_t first, uint8_t second);

public:
  virtual bool PutActiveFrequency(RadioFrequency frequency,
                                  const TCHAR *name,
                                  OperationEnvironment &env) override;
  virtual bool PutStandbyFrequency(RadioFrequency frequency,
                                   const TCHAR *name,
                                   OperationEnvironment &env) override;

  /**
   * @brief checks if data is received
   * 
   * @param _data data which is received
   * @param length length of the data
   * @param info nmea info
   * @return true data has been received
   * @return false no data has been received
   */
  virtual bool DataReceived(const void *data, size_t length, struct NMEAInfo &info) override;
};

/**
 * @brief Construct a new AR62xxDevice::AR62xxDevice object
 * 
 * @param _port the port to connect to
 */
AR62xxDevice::AR62xxDevice(Port &_port) : port(_port) {}

/**
 * @brief Get the n-th Byte of an unsigned integer
 * 
 * @param number unsigned integer
 * @param n byte-number
 * @return uint8_t byte
 */
uint8_t AR62xxDevice::GetNthByte(unsigned number, int n)
{
  return number >> (8 * n) & 0xff;
}

/**
 * @brief Converts to uint8_t-bytes to an unsigned integer
 * 
 * @param first first byte
 * @param second second byte
 * @return unsigned unsigned integer
 */
unsigned AR62xxDevice::BytesToUnsigned(uint8_t first, uint8_t second)
{
  return second << 8 | first;
}

/**
 * @brief checks if data is received
 * 
 * @param _data data which is received
 * @param length length of the data
 * @param info nmea info
 * @return true data has been received
 * @return false no data has been received
 */
bool AR62xxDevice::DataReceived(const void *_data, size_t length, struct NMEAInfo &info)
{
  /** check we received data */
  if (_data == nullptr || length <= 0)
    return false;

  /** get uint8_t of data */
  const uint8_t *data = (const uint8_t *)_data;

  /** try to parse data, if it can be parsed it will return true*/
  bool data_is_ok = ParseReceivedData((const char *)data, length);

  /** send nmea info that we are receiving data */
  info.alive.Update(info.clock);

  return data_is_ok;
}

/**
   * @brief Send - Sends a message to the AR62xx
   * 
   * @param msg message
   * @param msg_size size of the message
   * @param env operation environment
   * @return true if message has been sent
   * @return false if message could not be sent
   */
bool AR62xxDevice::Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env)
{
  /** Number of tries to send a message
   * will be decreased on every retry */
  unsigned retries = 5;

  /** flag if message has been sent */
  bool message_sent = false;

  /** loop until retries are reached of message has been sent */
  do
  {
    /** try to send message */
    message_sent = port.FullWrite(msg, msg_size, env, std::chrono::milliseconds(250));

    retries--;
  } while (retries > 0 && !message_sent);

  return message_sent;
}

RadioFrequency AR62xxDevice::AR62xxIdToFrequency(uint16_t frequency_id)
{
  /** get frequency-part out of frequency-id */
  unsigned frequency_encoded = frequency_id & FREQUENCY_BITMASK;

  /** get channel-part out of frequency-id */
  unsigned channel_encoded = frequency_id & CHANNEL_BITMASK;

  /** calculate the frequency */
  unsigned frequency = MIN_FREQUENCY + frequency_encoded * FREQUENCY_RANGE / MAX_FREQUENCY_ID;

  /** add channel to frequency */
  if (channel_encoded <= 3)
    frequency += channel_encoded * 5;
  else if (channel_encoded <= 7)
    frequency += channel_encoded * 5 + 5;
  else if (channel_encoded <= 11)
    frequency += channel_encoded * 5 + 10;
  else if (channel_encoded <= 15)
    frequency += channel_encoded * 5 + 15;

  /** generate RadioFrequency object */
  RadioFrequency radio_frequency = RadioFrequency();
  radio_frequency.SetKiloHertz(frequency);

  return radio_frequency;
}

/**
 * @brief converts a frequency to an AR62xx-ID
 * 
 * explanation: the AR62xx uses an integer value for each 8.33/25 kHz frequency starting at 118.000 kHz
 * e.g. frequency-id 0    => 118.000 kHz
 *      frequency-id 1    => 118.005 kHz
 *      frequency-id 3040 => 137.000 kHz
 * 
 * @param frequency frequency in kHz
 * @return uint16_t frequency-id
 */
uint16_t AR62xxDevice::FrequencyToAR62xxId(RadioFrequency freq)
{
  /** get frequency in kiloherz */
  unsigned frequency = freq.GetKiloHertz();

  /** check if frequency is not in range */
  if (frequency < MIN_FREQUENCY || frequency > MAX_FREQUENCY)
    return 0;

  /** get offset of frequency to the lowest frequency which is possible */
  unsigned frequency_offset = frequency - MIN_FREQUENCY;

  /** generate frequency-id */
  uint16_t frequency_id = std::round(frequency_offset * MAX_FREQUENCY_ID / FREQUENCY_RANGE);

  /** append frequency bitmask as channel has to be added separately */
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

/**
 * @brief bitwise cyclic redundancy check - algorithm
 * 
 * @param data data which should be checked
 * @param len length of data
 * @return uint16_t checksum
 */
static uint16_t CRCBitwise(uint8_t *data, size_t len)
{
  uint16_t crc = 0x0000;
  for (size_t j = len; j > 0; j--)
  {
    crc ^= (uint16_t)(*data++) << 8;
    for (int i = 0; i < 8; i++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x8005;
      else
        crc <<= 1;
    }
  }
  return (crc);
}

/**
   * @brief generates a command to set a station in the AR62xx
   * 
   * @param command command-output to set the station
   * @param station_id the id of the station which should be set
   *        1 = active station of AR62xx
   *        0 = passive station of AR62xx
   * @param frequency frequency in kHz to set
   * @param station name of the station to set
   *        (unused)
   * @return int command-length
   */
int AR62xxDevice::GenerateSetStationCommand(uint8_t *command, int active_passive, RadioFrequency frequency, const TCHAR *station)
{
  /** length of the command */
  unsigned int command_length = 0;

  /** convert the active-frequency to an AR62xx-ID */
  uint16_t active_frequency_idx = FrequencyToAR62xxId(active_frequency);

  /** convert the passive-frequency to an AR62xx-ID */
  uint16_t passive_frequency_idx = FrequencyToAR62xxId(passive_frequency);

  /** command-start */
  command[command_length++] = 0xA5;

  /** add prot-ID */
  command[command_length++] = 0x14;
  command[command_length++] = 5;

  /** set active station */
  if (active_passive == ACTIVE_STATION)
    active_frequency_idx = FrequencyToAR62xxId(frequency);

  /** set passive station */
  else
    passive_frequency_idx = FrequencyToAR62xxId(frequency);

  /** setting frequencies */
  command[command_length++] = 22;
  command[command_length++] = GetNthByte(active_frequency_idx, 1);
  command[command_length++] = GetNthByte(active_frequency_idx, 0);
  command[command_length++] = GetNthByte(passive_frequency_idx, 1);
  command[command_length++] = GetNthByte(passive_frequency_idx, 0);

  /** Creating the binary value */
  uint16_t command_crc = CRCBitwise(command, command_length);
  command[command_length++] = GetNthByte(command_crc, 1);
  command[command_length++] = GetNthByte(command_crc, 0);
  return command_length;
}

/**
   * @brief parses the received data
   * this is needed to save the active and passive station
   * 
   * @param string recived data
   * @param len length of data
   * @return true if data could be parsed
   * @return false if data could not be parsed
   */
bool AR62xxDevice::ParseReceivedData(const char *string, size_t len)
{
  /** check if there is data */
  if (string == NULL || len == 0)
    return false;

  size_t idx = 0;
  uint16_t crc = 0;
  static uint16_t rec_buf_len = 0;
  int command_length = 0;
#define REC_BUFSIZE 127
  static uint8_t command[REC_BUFSIZE];

  /** loop as long we have data */
  while (idx < len)
  {
    /** check if we received a header*/
    if ((uint8_t)string[idx] == 0xA5)
      rec_buf_len = 0;

    /** check if the buffer-size is reached */
    if (rec_buf_len >= REC_BUFSIZE)
      rec_buf_len = 0;
    assert(rec_buf_len < REC_BUFSIZE);

    /** write command in buffer */
    command[rec_buf_len++] = (uint8_t)string[idx++];
    if (rec_buf_len == 2)
    {
      /** check if command is PROT-ID */
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
        /** generate uint16_t out of commands */
        uint16_t crc_value = BytesToUnsigned(command[command_length + 4], command[command_length + 3]);

        /** generate crc */
        crc = CRCBitwise(command, command_length + 3);

        /** check crc */
        if (crc == crc_value || crc_value == 0)
        {
          /** extract and save active and passive frequency */
          SaveAR62xxFrequencies(command, command_length + 5, crc);
        }
        rec_buf_len = 0;
      }
    }
  }
  return true;
}

/**
   * @brief save frequencies
   * 
   * @param sz_command command which contains the frequencies
   * @param len length of the command
   * @param crc CRC
   */
void AR62xxDevice::SaveAR62xxFrequencies(uint8_t *sz_command, int len, uint16_t crc)
{
  /** check if the command is valid */
  if (sz_command == NULL || len == 0)
    return;

  /** command 22 = frequency-settings
   *  Frequency settings, always for both frequencies (active and passive) */
  if ((unsigned char)(sz_command[3] & 0x7F) == 22)
  {
    active_frequency = AR62xxIdToFrequency(BytesToUnsigned(sz_command[5], sz_command[4]));
    passive_frequency = AR62xxIdToFrequency(BytesToUnsigned(sz_command[7], sz_command[6]));
  }
}

bool AR62xxDevice::PutActiveFrequency(RadioFrequency frequency,
                                      const TCHAR *name,
                                      OperationEnvironment &env)
{
  /** command */
  uint8_t szTmp[MAX_CMD_LEN];

  /** generate the command to set the active station */
  int len = GenerateSetStationCommand(szTmp, ACTIVE_STATION, frequency, (const TCHAR *)name);

  /** send the command */
  bool isSend = Send((uint8_t *)&szTmp, len, env);

  /** return if command could be send */
  return isSend;
}

bool AR62xxDevice::PutStandbyFrequency(RadioFrequency frequency,
                                       const TCHAR *name,
                                       OperationEnvironment &env)
{
  /** command */
  uint8_t szTmp[MAX_CMD_LEN] = {};

  /** generate the command to set the active station */
  int len = GenerateSetStationCommand(szTmp, PASSIVE_STATION, frequency, (const TCHAR *)name);

  /** send the command */
  bool isSend = Send((uint8_t *)&szTmp, len, env);

  /** return if command could be send */
  return isSend;
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