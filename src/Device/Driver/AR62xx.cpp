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

#define MAX_CMD_LEN 128
#define ACTIVE_STATION 1
#define PASSIVE_STATION 0

typedef union {
  uint16_t intVal16;
  uint8_t intVal8[2];
} IntConvertStruct;

class AR62xxDevice final : public AbstractDevice
{

public:
  explicit AR62xxDevice(Port &_port);

private:
  Port &port;   //!< Port the radio is connected to.
  RadioFrequency active_frequency;  //!< active station frequency
  RadioFrequency passive_frequency; //!< passive (or standby) station frequency

  bool Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env);

  uint16_t ConvertFrequencyToAR62FrequencyId(RadioFrequency frequency);

  int SetAR620xStation(uint8_t *command, int active_passive, RadioFrequency frequency, const TCHAR *station);

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
}

bool AR62xxDevice::DataReceived(const void *_data, size_t length, struct NMEAInfo &info)
{
  return true;
}

bool AR62xxDevice::Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env)
{
  return port.FullWrite(msg, msg_size, env, std::chrono::milliseconds(250));
}



uint16_t AR62xxDevice::ConvertFrequencyToAR62FrequencyId(RadioFrequency freq)
{
  int frequency_bitmask = 0xFFF0; //!< bitmask to get the frequency

  int min_frequency = 118000;
  int max_frequency = 137000;
  int frequency_range = max_frequency - min_frequency;
  int raster = 3040;
  uint16_t frequency_id = (freq.GetKiloHertz()-min_frequency) * raster / frequency_range + 0.5;
  

  frequency_id &= frequency_bitmask;

  uint8_t channel = freq.GetKiloHertz() % 100;

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
  unsigned int command_length = 0;
  assert(station != NULL);
  assert(command != NULL);

  if (command == NULL)
    return false;

  //!< converting both actual frequencies
  IntConvertStruct ActiveFreqIdx;

  ActiveFreqIdx.intVal16 = ConvertFrequencyToAR62FrequencyId(active_frequency);
  IntConvertStruct PassiveFreqIdx;
  PassiveFreqIdx.intVal16 = ConvertFrequencyToAR62FrequencyId(passive_frequency);

  //add header-id
  command[command_length++] = 0xA5;

  //add prot-ID
  command[command_length++] = 0x14;

  command[command_length++] = 5;

  //!< converting the frequency which is to be changed
  switch (active_passive)
  {
  case ACTIVE_STATION:
    ActiveFreqIdx.intVal16 = ConvertFrequencyToAR62FrequencyId(frequency);
    break;
  default:
  case PASSIVE_STATION:
    PassiveFreqIdx.intVal16 = ConvertFrequencyToAR62FrequencyId(frequency);
    break;
  }

  //!< setting frequencies -command byte in the protocol of the radio
  command[command_length++] = 22;
  command[command_length++] = ActiveFreqIdx.intVal8[1];
  command[command_length++] = ActiveFreqIdx.intVal8[0];
  command[command_length++] = PassiveFreqIdx.intVal8[1];
  command[command_length++] = PassiveFreqIdx.intVal8[0];

  //!< Creating the binary value
  IntConvertStruct crc;
  crc.intVal16 = CRCBitwise(command, command_length);
  command[command_length++] = crc.intVal8[1];
  command[command_length++] = crc.intVal8[0];
  return command_length;
}

bool AR62xxDevice::PutActiveFrequency(RadioFrequency frequency,
                                      const TCHAR *name,
                                      OperationEnvironment &env)
{
  uint8_t szTmp[MAX_CMD_LEN];
  int len = SetAR620xStation(szTmp, ACTIVE_STATION, frequency, (const TCHAR *)name);
  return Send((uint8_t *)&szTmp, len, env);
}

bool AR62xxDevice::PutStandbyFrequency(RadioFrequency frequency,
                                       const TCHAR *name,
                                       OperationEnvironment &env)
{
  uint8_t szTmp[MAX_CMD_LEN] = {};
  int len = SetAR620xStation(szTmp, PASSIVE_STATION, frequency, (const TCHAR *)name);
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