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

constexpr uint8_t HEADER_ID = 0xA5;
#define MAX_CMD_LEN 128
#define ACTIVE_STATION 1
#define PASSIVE_STATION 0

typedef union {
  uint16_t intVal16;
  uint8_t intVal8[2];
} IntConvertStruct;

class AR62xxDevice final : public AbstractDevice
{
  static constexpr char ACK = 0x06; //!< command acknowledged character.

public:
  explicit AR62xxDevice(Port &_port);

private:
  Port &port;       //!< Port the radio is connected to.
  uint8_t response; //!< Last response received from the radio.
  Cond rx_cond;     //!< Condition to signal that a response was received from the radio.

  IntConvertStruct crc;
  IntConvertStruct frequency;
  IntConvertStruct status;

  double active_frequency;  //!< active station frequency
  double passive_frequency; //!< passive (or standby) station frequency
  bool parameter_changed;   //!< Parameter Changed Flag TRUE = parameter changed)

  bool is_sending = false;

  bool Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env);

  bool AR620xPutFreqActive(RadioFrequency frequency, const TCHAR *station_name, OperationEnvironment &env);

  bool AR620xPutFreqStandby(RadioFrequency frequency, const TCHAR *station_name, OperationEnvironment &env);

  uint16_t ConvertFrequencyToAR62FrequencyId(double frequency);

  double ConvertAR62FrequencyIDToFrequency(uint16_t frequency_id);

  int SetAR620xStation(uint8_t *command, int active_passive, RadioFrequency frequency, const TCHAR *station);

  bool AR620xParseString(const char *string, size_t len);

  int AR620x_Convert_Answer(uint8_t *sz_command, int len, uint16_t crc);

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
  unsigned retries = 3; //!< Number of tries to send a message will be decreased on every retry
  assert(msg_size > 0); //!< check that msg is not empty
  Mutex response_mutex; //!< Mutex to be locked to access response.

  do
  {
    {
      const std::lock_guard<Mutex> lock(response_mutex);

      //initialize response with "No response received yet"
      response = 0; 
    }

    is_sending = true; //!< set sending-flat

    //check if message sent
    if (!port.FullWrite(msg, msg_size, env, std::chrono::milliseconds(250)))
    {
      //if message could not be sent set response to "command not acknowledged character"
      response = 0x15; 
    }

    //message sent
    else
    {
      response = ACK; //!< if message could be sent set response to "command acknowledged character"
    }

    //!< Wait for the response
    uint8_t _response;
    {
      std::unique_lock<Mutex> lock(response_mutex);
      rx_cond.wait_for(lock, std::chrono::milliseconds(250)); //!< wait for the response
      _response = response;
    }
    is_sending = false; //!< reset flag is_sending

    //!< ACK received
    if (_response == ACK)
    {
      // ACK received, finish, all went well
      return true;
    }

    //!< No ACK received, retry, possibly an error occurred
    retries--;
  } while (retries);

  //!< returns false if message could not be sent
  return false;
}

double AR62xxDevice::ConvertAR62FrequencyIDToFrequency(uint16_t frequency_id)
{
  double min_frequency = 118.000;                                                                         //!< the lowest frequency-number which can be set in AR62xx
  double max_frequency = 137.000;                                                                         //!< the highest frequency-number which can be set in AR62xx
  double frequency_range = max_frequency - min_frequency;                                                 //!< the frequeny-range which can be set in the AR62xx
  int frequency_bitmask = 0xFFF0;                                                                         //!< bitmask to get the frequency
  int channel_bitmask = 0xF;                                                                              //!< bitmask to get the chanel
  double raster = 3040.0;                                                                                 //!< raster-length
  double radio_frequency = min_frequency + (frequency_id & frequency_bitmask) * frequency_range / raster; //!< calculate frequency

  //!< get the channel out of the frequence_id
  switch (frequency_id & channel_bitmask)
  {
  case 0:
    radio_frequency += 0.000;
    break;
  case 1:
    radio_frequency += 0.005;
    break;
  case 2:
    radio_frequency += 0.010;
    break;
  case 3:
    radio_frequency += 0.015;
    break;
  case 4:
    radio_frequency += 0.025;
    break;
  case 5:
    radio_frequency += 0.030;
    break;
  case 6:
    radio_frequency += 0.035;
    break;
  case 7:
    radio_frequency += 0.040;
    break;
  case 8:
    radio_frequency += 0.050;
    break;
  case 9:
    radio_frequency += 0.055;
    break;
  case 10:
    radio_frequency += 0.060;
    break;
  case 11:
    radio_frequency += 0.065;
    break;
  case 12:
    radio_frequency += 0.075;
    break;
  case 13:
    radio_frequency += 0.080;
    break;
  case 14:
    radio_frequency += 0.085;
    break;
  case 15:
    radio_frequency += 0.090;
    break;
  }
  return radio_frequency;
}

uint16_t AR62xxDevice::ConvertFrequencyToAR62FrequencyId(double frequency)
{

  double min_frequency = 118.000;                         //!< the lowest frequency-number which can be set in AR62xx
  double max_frequency = 137.000;                         //!< the highest frequency-number which can be set in AR62xx
  double frequency_range = max_frequency - min_frequency; //!< the frequeny-range which can be set in the AR62xx
  int frequency_bitmask = 0xFFF0;                         //!< bitmask to get the frequency
  double raster = 3040.0;                                 //!< raster-length

  uint16_t frequency_id = (((frequency)-min_frequency) * raster / frequency_range + 0.5);
  frequency_id &= frequency_bitmask;
  uint8_t channel = ((int)(frequency * 1000.0 + 0.5)) - (((int)(frequency * 10.0)) * 100);
  switch (channel)
  {
  case 0:
    frequency_id += 0;
    break;
  case 5:
    frequency_id += 1;
    break;
  case 10:
    frequency_id += 2;
    break;
  case 15:
    frequency_id += 3;
    break;
  case 25:
    frequency_id += 4;
    break;
  case 30:
    frequency_id += 5;
    break;
  case 35:
    frequency_id += 6;
    break;
  case 40:
    frequency_id += 7;
    break;
  case 50:
    frequency_id += 8;
    break;
  case 55:
    frequency_id += 9;
    break;
  case 60:
    frequency_id += 10;
    break;
  case 65:
    frequency_id += 11;
    break;
  case 75:
    frequency_id += 12;
    break;
  case 80:
    frequency_id += 13;
    break;
  case 85:
    frequency_id += 14;
    break;
  case 90:
    frequency_id += 15;
    break;
  case 100:
    frequency_id += 0;
    break;
  default:
    break;
  }
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
  //!< converting both actual frequencies
  IntConvertStruct ActiveFreqIdx;

  ActiveFreqIdx.intVal16 = ConvertFrequencyToAR62FrequencyId(active_frequency);
  IntConvertStruct PassiveFreqIdx;
  PassiveFreqIdx.intVal16 = ConvertFrequencyToAR62FrequencyId(passive_frequency);
  command[len++] = HEADER_ID;

  //add prot-ID
  command[len++] = 0x14;

  command[len++] = 5;

  double freq = frequency.GetKiloHertz() / 1000.0;

  //!< converting the frequency which is to be changed
  switch (active_passive)
  {
  case ACTIVE_STATION:
    ActiveFreqIdx.intVal16 = ConvertFrequencyToAR62FrequencyId(freq);
    break;
  default:
  case PASSIVE_STATION:
    PassiveFreqIdx.intVal16 = ConvertFrequencyToAR62FrequencyId(freq);
    break;
  }

  //!< setting frequencies -command byte in the protocol of the radio
  command[len++] = 22;
  command[len++] = ActiveFreqIdx.intVal8[1];
  command[len++] = ActiveFreqIdx.intVal8[0];
  command[len++] = PassiveFreqIdx.intVal8[1];
  command[len++] = PassiveFreqIdx.intVal8[0];

  //!< Creating the binary value
  crc.intVal16 = CRCBitwise(command, len);
  command[len++] = crc.intVal8[1];
  command[len++] = crc.intVal8[0];
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
    if ((uint8_t)string[cnt] == HEADER_ID)
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
        crc.intVal8[1] = command[CommandLength + 3];
        crc.intVal8[0] = command[CommandLength + 4];
        CalCRC = CRCBitwise(command, CommandLength + 3);
        if (CalCRC == crc.intVal16 || crc.intVal16 == 0)
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

  static uint16_t uiLastChannelCRC = 0;
  static uint16_t uiVersionCRC = 0;

  int processed = 0;

  assert(sz_command != NULL);

  switch ((unsigned char)(sz_command[3] & 0x7F))
  {
  case 0:
    if (uiVersionCRC != crc)
    {
      uiVersionCRC = crc;
    }
    break;

  //!< Frequency settings, always for both frequencies (active and passive)
  case 22:
    if (uiLastChannelCRC != crc)
    {
      uiLastChannelCRC = crc;
      parameter_changed = true;
      frequency.intVal8[1] = sz_command[4];
      frequency.intVal8[0] = sz_command[5];
      active_frequency = ConvertAR62FrequencyIDToFrequency(frequency.intVal16);

      frequency.intVal8[1] = sz_command[6];
      frequency.intVal8[0] = sz_command[7];
      passive_frequency = ConvertAR62FrequencyIDToFrequency(frequency.intVal16);
      parameter_changed = true;
    }
    break;
  default:
    break;
  }

  //!< return the number of converted characters
  return processed;
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