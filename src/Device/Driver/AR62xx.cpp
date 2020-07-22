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

class AR62xxDevice final : public AbstractDevice
{
  static constexpr char ACK = 0x06; //!< command acknowledged character.

public:
  explicit AR62xxDevice(Port &_port);

private:
  Port &port;       //!< Port the radio is connected to.
  uint8_t response; //!< Last response received from the radio.
  Cond rx_cond;     //!< Condition to signal that a response was received from the radio.

  RadioFrequency active_frequency;  //!< active station frequency
  RadioFrequency passive_frequency; //!< passive (or standby) station frequency
  bool parameter_changed;           //!< Parameter Changed Flag TRUE = parameter changed)

  bool is_sending = false;

  bool Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env);

  bool AR620xPutFreqActive(RadioFrequency frequency, const TCHAR *station_name, OperationEnvironment &env);

  bool AR620xPutFreqStandby(RadioFrequency frequency, const TCHAR *station_name, OperationEnvironment &env);

  uint16_t ConvertFrequencyToAR62FrequencyId(RadioFrequency frequency);

  RadioFrequency ConvertAR62FrequencyIDToFrequency(uint16_t frequency_id);

  int SetAR620xStation(uint8_t *command, int active_passive, RadioFrequency frequency, const TCHAR *station);

  bool AR620xParseString(const char *string, size_t len);

  int AR620x_Convert_Answer(uint8_t *sz_command, int len, uint16_t crc);

  uint8_t GetNthByte(unsigned number, int n);

  unsigned BytesToUnsigned(uint8_t first, uint8_t second);

public:
  virtual bool PutActiveFrequency(RadioFrequency frequency,
                                  const TCHAR *name,
                                  OperationEnvironment &env) override;
  virtual bool PutStandbyFrequency(RadioFrequency frequency,
                                   const TCHAR *name,
                                   OperationEnvironment &env) override;
  virtual bool DataReceived(const void *data, size_t length, struct NMEAInfo &info) override;
};

AR62xxDevice::AR62xxDevice(Port &_port) : port(_port)
{
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

  assert(_data != nullptr);                                        //!< check that data is not empty (null)
  assert(length > 0);                                              //< the length of the data has to be greater than zero
  const uint8_t *data = (const uint8_t *)_data;                    //!< cast data to int
  bool data_is_ok = AR620xParseString((const char *)data, length); //!< if data can be parsed return true
  info.alive.Update(info.clock);                                   //!< send nmea info that we are receiving data
  return data_is_ok;
}

bool AR62xxDevice::Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env)
{
  unsigned retries = 5; //!< Number of tries to send a message will be decreased on every retry
  bool message_sent = false;

  do
  {
    message_sent = port.FullWrite(msg, msg_size, env, std::chrono::milliseconds(250));
    retries--;
  } while (retries > 0 && !message_sent);

  return message_sent;
}

RadioFrequency AR62xxDevice::ConvertAR62FrequencyIDToFrequency(uint16_t frequency_id)
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

uint16_t AR62xxDevice::ConvertFrequencyToAR62FrequencyId(RadioFrequency freq)
{
  unsigned frequency = freq.GetKiloHertz();

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

int AR62xxDevice::SetAR620xStation(uint8_t *command, int active_passive, RadioFrequency frequency, const TCHAR *station)
{
  unsigned int len = 0;
  assert(station != NULL);
  assert(command != NULL);
  if (command == NULL)
  {
    return false;
  }
  uint16_t active_frequency_idx = ConvertFrequencyToAR62FrequencyId(active_frequency);
  uint16_t passive_frequency_idx = ConvertFrequencyToAR62FrequencyId(passive_frequency);
  command[len++] = 0xA5;

  //add prot-ID
  command[len++] = 0x14;

  command[len++] = 5;

  //!< converting the frequency which is to be changed
  switch (active_passive)
  {
  case ACTIVE_STATION:
    active_frequency_idx = ConvertFrequencyToAR62FrequencyId(frequency);
    break;
  default:
  case PASSIVE_STATION:
    passive_frequency_idx = ConvertFrequencyToAR62FrequencyId(frequency);
    break;
  }

  //!< setting frequencies -command byte in the protocol of the radio
  command[len++] = 22;
  command[len++] = GetNthByte(active_frequency_idx, 1);
  command[len++] = GetNthByte(active_frequency_idx, 0);
  command[len++] = GetNthByte(passive_frequency_idx, 1);
  command[len++] = GetNthByte(passive_frequency_idx, 0);

  //!< Creating the binary value
  uint16_t command_crc = CRCBitwise(command, len);
  command[len++] = GetNthByte(command_crc, 1);
  command[len++] = GetNthByte(command_crc, 0);
  return len;
}

bool AR62xxDevice::AR620xPutFreqActive(RadioFrequency frequency, const TCHAR *station_name, OperationEnvironment &env)
{
  int len;
  uint8_t szTmp[MAX_CMD_LEN];
  len = SetAR620xStation(szTmp, ACTIVE_STATION, frequency, station_name);
  bool isSend = Send((uint8_t *)&szTmp, len, env);
  return isSend;
}

bool AR62xxDevice::AR620xPutFreqStandby(RadioFrequency frequency, const TCHAR *station_name, OperationEnvironment &env)
{
  int len;
  uint8_t szTmp[MAX_CMD_LEN] = {};
  len = SetAR620xStation(szTmp, PASSIVE_STATION, frequency, station_name);
  bool isSend = Send((uint8_t *)&szTmp, len, env);
  return isSend;
}

bool AR62xxDevice::AR620xParseString(const char *string, size_t len)
{
  size_t cnt = 0;
  uint16_t CalCRC = 0;
  static uint16_t Recbuflen = 0;
  int CommandLength = 0;
#define REC_BUFSIZE 127
  static uint8_t command[REC_BUFSIZE];

  if (string == NULL)
    return 0;
  if (len == 0)
    return 0;

  while (cnt < len)
  {
    if ((uint8_t)string[cnt] == 0xA5)
      Recbuflen = 0;
    if (Recbuflen >= REC_BUFSIZE)
      Recbuflen = 0;
    assert(Recbuflen < REC_BUFSIZE);

    command[Recbuflen++] = (uint8_t)string[cnt++];
    if (Recbuflen == 2)
    {
      if (!(command[Recbuflen - 1] == 0x14))
      {
        Recbuflen = 0;
      }
    }

    if (Recbuflen >= 3)
    {
      CommandLength = command[2];

      // all received
      if (Recbuflen >= (CommandLength + 5))
      {
        uint16_t crc_value = BytesToUnsigned(command[CommandLength + 4], command[CommandLength + 3]);

        CalCRC = CRCBitwise(command, CommandLength + 3);
        if (CalCRC == crc_value || crc_value == 0)
        {
          if (!is_sending)
          {
            AR620x_Convert_Answer(command, CommandLength + 5, CalCRC);
          }
        }
        Recbuflen = 0;
      }
    }
  }
  return parameter_changed;
}

int AR62xxDevice::AR620x_Convert_Answer(uint8_t *sz_command, int len, uint16_t crc)
{
  if (sz_command == NULL)
    return 0;
  if (len == 0)
    return 0;

  assert(sz_command != NULL);

  switch ((unsigned char)(sz_command[3] & 0x7F))
  {
  //!< Frequency settings, always for both frequencies (active and passive)
  case 22:
    active_frequency = ConvertAR62FrequencyIDToFrequency(BytesToUnsigned(sz_command[5], sz_command[4]));
    passive_frequency = ConvertAR62FrequencyIDToFrequency(BytesToUnsigned(sz_command[7], sz_command[6]));
    break;
  default:
    break;
  }

  return 0;
}

bool AR62xxDevice::PutActiveFrequency(RadioFrequency frequency,
                                      const TCHAR *name,
                                      OperationEnvironment &env)
{
  return AR620xPutFreqActive(frequency, (const TCHAR *)name, env);
}

bool AR62xxDevice::PutStandbyFrequency(RadioFrequency frequency,
                                       const TCHAR *name,
                                       OperationEnvironment &env)
{
  return AR620xPutFreqStandby(frequency, (const TCHAR *)name, env);
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