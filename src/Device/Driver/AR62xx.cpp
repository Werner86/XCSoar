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

/*
 * The driver is derived from the KRT2-driver. This version has implemented two methods yet:
 * setting the active frequency
 * setting the passive frequency
 * 
 * TODO: setting/reading dual scan on/off
 * TODO: reading frequency change on the radio
 * TODO: setting/reading squelsh settings
 *
 */

#include "LogFile.hpp"

#include "Device/Driver/AR62xx.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "NMEA/Info.hpp"
#include "RadioFrequency.hpp"
#include "Thread/Cond.hxx"
#include "Thread/Mutex.hxx"
#include "Util/CharUtil.hxx"
#include "Util/StaticFifoBuffer.hxx"

#include <stdint.h>
#include <stdio.h>
#include <chrono>
#include <cstdlib>

/*
 * Constants for AR62xx binary protocol
 */
constexpr uint8_t HEADER_ID = 0xA5;
#define PROT_ID 0x14;
#define QUERY BIT(7)
#define DUAL BIT(8)
#define SQUELCH BIT(7)
#define MAX_CMD_LEN 128
#define ACTIVE_STATION 1
#define PASSIVE_STATION 0
#define BIT(n) (1 << (n))
#define NAME_SIZE 30

union IntConvertStruct {
  uint16_t int_val_16;
  uint8_t int_val_8[2];
};

struct Radio_t
{
  //frequency of the currently active station
  double active_frequency;

  //passive (or standby) station frequency
  double passive_frequency;

  //passive (or standby) station name
  TCHAR passive_name[NAME_SIZE];

  //active station name
  TCHAR active_name[NAME_SIZE];

  //radio volume (the loudness of the radio)
  int radio_volume;

  //the squelch-level of the radio
  int squelch;

  //the intercom volume (loudness) of the radio
  int vox;

  //parameter changed flag (TRUE = parameter changed)
  bool changed;

  //Radio Installed d Flag (TRUE = Radio found)
  bool enabled;

  //Dual Channel mode active flag(TRUE = on)
  bool dual;

  //8,33kHz Radio enabled (TRUE = 8,33kHz)
  bool enabled_8_33;

  //Radio reception active (TRUE = reception)
  bool reception_active;

  //Radio transmission active (TRUE = transmission)
  bool transmission_active;

  //Radio reception on active station (TRUE = reception)
  bool reception_on_active_station;

  //Radio reception on passive (standby) station
  bool reception_standby;

  //Battery low flag (TRUE = Batt low)
  bool low_bat;

  //Timeout while transmission (2Min)
  bool transmission_timeout;
};

IntConvertStruct crc;
IntConvertStruct frequency;
IntConvertStruct status;

volatile bool sending = false;

/**
 * AR62xx device class.
 *
 * This class provides the interface to communicate with the AR62xx radio.
 * The driver retransmits messages in case of a failure.
 */
class AR62xxDevice final : public AbstractDevice
{
  //!< Command timeout in millis.
  static constexpr unsigned CMD_TIMEOUT = 250;

  //!< Number of tries to send a command.
  static constexpr unsigned NR_RETRIES = 3;

  //!< Command start character.
  static constexpr char START_CHAR = 0x02;

  //!< Command acknowledged character.
  static constexpr char ACKNOWLEDGED = 0x06;

  //!< Command not acknowledged character.
  static constexpr char NOT_ACKNOWLEDGED = 0x15;

  //!< No response received yet.
  static constexpr char NO_RESPONSE_YET = 0;

  //!< Max. radio station name length.
  static constexpr size_t MAX_NAME_LENGTH = 10;

  //! Port the radio is connected to.
  Port &port;

  //! Buffer which receives the messages send from the radio.
  StaticFifoBuffer<uint8_t, 256u> rx_buf;

  //! Last response received from the radio.
  uint8_t response;

  //! Condition to signal that a response was received from the radio.
  Cond rx_cond;

  //! Mutex to be locked to access response.
  std::unique_lock<Mutex> response_mutex;

  // RadioPara aus LK8000
  Radio_t RadioPara;

public:
  /**
   * Constructor of the radio device class.
   *
   * @param _port Port the radio is connected to.
   */
  AR62xxDevice(Port &_port);

private:
  /**
   * Sends a message to the radio.
   *
   * @param msg Message to be send to the radio.
   */
  bool Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env);

  /*
   * Called by "PutActiveFrequency"
   * doing the real work
   */
  bool AR620xPutFreqActive(double frequency, const TCHAR *stationName, OperationEnvironment &env);

  /*
   * Called by "PutStandbyFrequency"
   * doing the real work
   */
  bool AR620xPutFreqStandby(double Freq, const TCHAR *StationName, OperationEnvironment &env);

  /*
   * Creates the correct index for a given readable frequency
   */
  uint16_t FrequencyToIdx(double frequency);

  /*
   * Creates a correct frequency-number given by the index
   */
  double IdxToFrequency(uint16_t frequency_Idx);

  /*
   * This function sets the station name and frequency on the AR62xx
   */
  int SetAR620xStation(uint8_t *Command, int active_passive, double frequency, const TCHAR *station);

  /*
   * Parses the messages which XCSoar receives from the radio.
   */
  bool AR620xParseString(const char *String, size_t len);

  int AR620x_Convert_Answer(uint8_t *szCommand, int len, uint16_t CRC);

public:
  /**
   * Sets the active frequency on the radio.
   */
  virtual bool PutActiveFrequency(RadioFrequency frequency,
                                  const TCHAR *name,
                                  OperationEnvironment &env) override;
  /**
   * Sets the standby frequency on the radio.
   */
  virtual bool PutStandbyFrequency(RadioFrequency frequency,
                                   const TCHAR *name,
                                   OperationEnvironment &env) override;
  /**
   * Receives and handles data from the radio.
   *
   * The function parses messages send by the radio.
   * Because all control characters (e.g. HEADER_ID, PROTOKOLL_ID, STX, ACK, NAK, ...)
   * can be part of the payload of the messages, it is important
   * to separate the messages to distinguish control characters
   * from payload characters.
   *
   * If a response to a command is received, the function notifies
   * the sender. This could trigger a retransmission in case of a
   * failure.
   */
  virtual bool DataReceived(const void *data, size_t length, struct NMEAInfo &info) override;
};

/*
 * Constructor
 * Port on which the radio is connected
 */
AR62xxDevice::AR62xxDevice(Port &_port) : port(_port)
{
  response = ACKNOWLEDGED;
  RadioPara.enabled_8_33 = true;
}

/**
 * Receives and handles data from the radio.
 *
 * The function parses messages send by the radio.
 * Because all control characters (e.g. HEADER_ID, PROTOKOLL_ID, STX, ACK, NAK, ...)
 * can be part of the payload of the messages, it is important
 * to separate the messages to distinguish control characters
 * from payload characters.
 *
 * If a response to a command is received, the function notifies
 * the sender. This could trigger a retransmission in case of a
 * failure.
 *
 * The initial frequency settings of the radio a delivered by this method and stored in the data struct "info"
 * every time connection is established
 */
bool AR62xxDevice::DataReceived(const void *_data, size_t length, struct NMEAInfo &info)
{
  assert(_data != nullptr);
  assert(length > 0);
  const uint8_t *data = (const uint8_t *)_data;
  bool data_received = AR620xParseString((const char *)data, length);
  info.alive.Update(info.clock);
  return data_received;
}

/*
 * Writes the message to the serial port on which the radio is connected
 */
bool AR62xxDevice::Send(const uint8_t *msg, unsigned msg_size, OperationEnvironment &env)
{
  //! Number of tries to send a message
  // i.e. 3 retries
  unsigned retries = NR_RETRIES; 
  assert(msg_size > 0);
  do
  {
    sending = true;
    response_mutex.lock();
    response = NO_RESPONSE_YET;
    response_mutex.unlock();
    // Send the message
    if (!port.FullWrite(msg, msg_size, env, std::chrono::seconds(CMD_TIMEOUT)))
    {
      sending = false;
      response = NOT_ACKNOWLEDGED;
    }
    else
    {
      response = ACKNOWLEDGED;
    }
    // Wait for the response
    response_mutex.lock();
    rx_cond.wait_for(response_mutex, std::chrono::seconds(CMD_TIMEOUT));

    auto _response = response;
    response_mutex.unlock();
    sending = false;
    if (_response == ACKNOWLEDGED)
      // ACK received, finish, all went well
      return true;

    // No ACK received, retry, possibly an error occurred
    retries--;
  } while (retries);
  return false;
}

/*
 * Creates a correct frequency-number to show
 */
double AR62xxDevice::IdxToFrequency(uint16_t frequency_idx)
{
  double min_frequency = 118.000;
  double max_frequency = 137.000;
  double frequencyRange = max_frequency - min_frequency;

  //set first 3 digits of frequency (xxx.000)
  double frequency = min_frequency + (frequency_idx & 0xFFF0) * frequencyRange / 3040.0;

  //set last 3 digits of frequency (000.xxx)
  switch (frequency_idx & 0xF)
  {
  case 0:
    frequency += 0.000;
    break;
  case 1:
    frequency += 0.005;
    break;
  case 2:
    frequency += 0.010;
    break;
  case 3:
    frequency += 0.015;
    break;
  case 4:
    frequency += 0.025;
    break;
  case 5:
    frequency += 0.030;
    break;
  case 6:
    frequency += 0.035;
    break;
  case 7:
    frequency += 0.040;
    break;
  case 8:
    frequency += 0.050;
    break;
  case 9:
    frequency += 0.055;
    break;
  case 10:
    frequency += 0.060;
    break;
  case 11:
    frequency += 0.065;
    break;
  case 12:
    frequency += 0.075;
    break;
  case 13:
    frequency += 0.080;
    break;
  case 14:
    frequency += 0.085;
    break;
  case 15:
    frequency += 0.090;
    break;
  }
  return frequency;
}

/*
 * Creates the correct index for a given readable frequency
 */
uint16_t AR62xxDevice::FrequencyToIdx(double frequency)
{
  uint16_t frequency_idx = (int)(((frequency)-118.0) * 3040 / (137.00 - 118.0) + 0.5);
  frequency_idx &= 0xFFF0;
  uint8_t uiFrac = ((int)(frequency * 1000.0 + 0.5)) - (((int)(frequency * 10.0)) * 100);
  switch (uiFrac)
  {
  case 0:
    frequency_idx += 0;
    break;
  case 5:
    frequency_idx += 1;
    break;
  case 10:
    frequency_idx += 2;
    break;
  case 15:
    frequency_idx += 3;
    break;
  case 25:
    frequency_idx += 4;
    break;
  case 30:
    frequency_idx += 5;
    break;
  case 35:
    frequency_idx += 6;
    break;
  case 40:
    frequency_idx += 7;
    break;
  case 50:
    frequency_idx += 8;
    break;
  case 55:
    frequency_idx += 9;
    break;
  case 60:
    frequency_idx += 10;
    break;
  case 65:
    frequency_idx += 11;
    break;
  case 75:
    frequency_idx += 12;
    break;
  case 80:
    frequency_idx += 13;
    break;
  case 85:
    frequency_idx += 14;
    break;
  case 90:
    frequency_idx += 15;
    break;
  case 100:
    frequency_idx += 0;
    break;
  default:
    break;
  }
  return (frequency_idx);
}

/*
 * Creates the binary value for the protocol for the radio
 */
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

/*****************************************************************************
 * This function sets the station name and frequency on the AR62xx
 *
 * Active_Passive Active or passive station switch
 * fFrequency     station frequency
 * Station        station Name string
 *
 * The AR62xx always sends and receives both frequencies. the one which remains unchanged
 * is controlled by the "Active_Passive"-flag
 *
 * Station is not used in the moment, AR62xx does not read it yet
 *
 *****************************************************************************/
int AR62xxDevice::SetAR620xStation(uint8_t *Command, int Active_Passive, double fFrequency, const TCHAR *Station)
{
  unsigned int len = 0;
  assert(Station != NULL);
  assert(Command != NULL);
  if (Command == NULL)
  {
    return false;
  }
  //converting both actual frequencies
  IntConvertStruct ActiveFreqIdx;
  ActiveFreqIdx.int_val_16 = FrequencyToIdx(RadioPara.active_frequency);
  IntConvertStruct PassiveFreqIdx;
  PassiveFreqIdx.int_val_16 = FrequencyToIdx(RadioPara.passive_frequency);
  Command[len++] = HEADER_ID;
  Command[len++] = PROT_ID;
  Command[len++] = 5;
  switch (Active_Passive)
  {
    //converting the frequency which is to be changed
  case ACTIVE_STATION:
    ActiveFreqIdx.int_val_16 = FrequencyToIdx(fFrequency);
    break;
  default:
  case PASSIVE_STATION:
    PassiveFreqIdx.int_val_16 = FrequencyToIdx(fFrequency);
    break;
  }

  //setting frequencies -command byte in the protocol of the radio
  Command[len++] = 22;
  Command[len++] = ActiveFreqIdx.int_val_8[1];
  Command[len++] = ActiveFreqIdx.int_val_8[0];
  Command[len++] = PassiveFreqIdx.int_val_8[1];
  Command[len++] = PassiveFreqIdx.int_val_8[0];

  //Creating the binary value
  crc.int_val_16 = CRCBitwise(Command, len);
  Command[len++] = crc.int_val_8[1];
  Command[len++] = crc.int_val_8[0];
  return len;
}

/*
 * Called by "PutActiveFrequency"
 * doing the real work
 */
bool AR62xxDevice::AR620xPutFreqActive(double Freq, const TCHAR *StationName, OperationEnvironment &env)
{
  int len;
  uint8_t szTmp[MAX_CMD_LEN];
  len = SetAR620xStation(szTmp, ACTIVE_STATION, Freq, StationName);
  bool isSend = Send((uint8_t *)&szTmp, len /*sizeof(szTmp)*/, env); //len seems to be ok!
  return isSend;
}

/*
 * Called by "PutStandbyFrequency"
 * doing the real work
 */
bool AR62xxDevice::AR620xPutFreqStandby(double Freq, const TCHAR *StationName, OperationEnvironment &env)
{
  int len;
  uint8_t szTmp[MAX_CMD_LEN] = {};
  len = SetAR620xStation(szTmp, PASSIVE_STATION, Freq, StationName);
  return Send((uint8_t *)&szTmp, len, env);
}

/*
 * Parses the messages which XCSoar receives from the radio.
 */
bool AR62xxDevice::AR620xParseString(const char *String, size_t len)
{
  size_t cnt = 0;
  uint16_t CalCRC = 0;
  static uint16_t Recbuflen = 0;
  int CommandLength = 0;
#define REC_BUFSIZE 127
  static uint8_t Command[REC_BUFSIZE];

  if (String == NULL)
    return 0;
  if (len == 0)
    return 0;

  while (cnt < len)
  {
    if ((uint8_t)String[cnt] == HEADER_ID)
      Recbuflen = 0;
    if (Recbuflen >= REC_BUFSIZE)
      Recbuflen = 0;
    assert(Recbuflen < REC_BUFSIZE);

    Command[Recbuflen++] = (uint8_t)String[cnt++];
    if (Recbuflen == 2)
    {
      if (!(Command[Recbuflen - 1] == 0x14))
      {
        Recbuflen = 0;
      }
    }

    if (Recbuflen >= 3)
    {
      CommandLength = Command[2];
      if (Recbuflen >= (CommandLength + 5))
      { // all received
        crc.int_val_8[1] = Command[CommandLength + 3];
        crc.int_val_8[0] = Command[CommandLength + 4];
        CalCRC = CRCBitwise(Command, CommandLength + 3);
        if (CalCRC == crc.int_val_16 || crc.int_val_16 == 0)
        {
          if (!sending)
          {
            AR620x_Convert_Answer(Command, CommandLength + 5, CalCRC);
          }
        }
        Recbuflen = 0;
      }
    }
  }
  return RadioPara.changed;
}

/*****************************************************************************
 * this function converts a KRT answer sting to a NMEA answer
 *
 * szAnswer       NMEA Answer
 * Answerlen      number of valid characters in the NMEA answerstring
 * szCommand      AR620x binary code to be converted, representing the state of a function (dual scan, squelsh, act. freq., pass. freq., ...)
 * len            length of the AR620x binary code to be converted
 ****************************************************************************/
int AR62xxDevice::AR620x_Convert_Answer(uint8_t *szCommand, int len, uint16_t CRC)
{
  if (szCommand == NULL)
    return 0;
  if (len == 0)
    return 0;

  static uint16_t uiLastChannelCRC = 0;
  static uint16_t uiVolumeCRC = 0;
  static uint16_t uiVersionCRC = 0;
  static uint16_t uiStatusCRC = 0;
  static uint16_t uiSquelchCRC = 0;
  static uint16_t uiRxStatusCRC = 0;
#ifdef RADIO_VOLTAGE
  static uint16_t uiVoltageCRC = 0;
#endif
  uint32_t ulState;
  int processed = 0;

  assert(szCommand != NULL);

  switch ((unsigned char)(szCommand[3] & 0x7F))
  {
  case 0:
    if (uiVersionCRC != CRC)
    {
      uiVersionCRC = CRC;
      //LogFormat(_T("AR620x Version %u"), CRC);
    }
    break;
  case 3: //Volume settings
    if (uiVolumeCRC != CRC)
    {
      uiVolumeCRC = CRC;
      RadioPara.changed = true;
      RadioPara.radio_volume = (50 - (int)szCommand[4]) / 5;
    }
    break;
  case 4: //Squelsh settings
    if (uiSquelchCRC != CRC)
    {
      uiSquelchCRC = CRC;
      RadioPara.changed = true;
      RadioPara.squelch = (int)(szCommand[4] - 6) / 2 + 1; // 6 + (Squelch-1)*2
    }
    break;
  case 12: //Dual scan settings
    if (uiStatusCRC != CRC)
    {
      uiStatusCRC = CRC;
      RadioPara.changed = true;
      status.int_val_8[1] = szCommand[4];
      status.int_val_8[0] = szCommand[5];
      if (status.int_val_16 & DUAL)
        RadioPara.dual = true;
      else
        RadioPara.dual = false;
    }
    break;
#ifdef RADIO_VOLTAGE
  case 21: // actual current of the radio
    if (uiVoltageCRC != CRC)
    {
      uiVoltageCRC = CRC;
      GPS_INFO.ExtBatt2_Voltage = 8.5 + szCommand[4] * 0.1;
      RadioPara.Changed = true;
    }
    break;
#endif
  case 22: //Frequency settings, always for both frequencies (active and passive)
    if (uiLastChannelCRC != CRC)
    {
      uiLastChannelCRC = CRC;
      RadioPara.changed = true;
      frequency.int_val_8[1] = szCommand[4];
      frequency.int_val_8[0] = szCommand[5];
      RadioPara.active_frequency = IdxToFrequency(frequency.int_val_16);

      frequency.int_val_8[1] = szCommand[6];
      frequency.int_val_8[0] = szCommand[7];
      RadioPara.passive_frequency = IdxToFrequency(frequency.int_val_16);
      RadioPara.changed = true;
    }
    break;
  case 64: // general state information
    if (uiRxStatusCRC != CRC)
    {
      uiRxStatusCRC = CRC;
      ulState = szCommand[4] << 24 | szCommand[5] << 16 | szCommand[6] << 8 | szCommand[7];
      RadioPara.transmission_active = ((ulState & (BIT(5) | BIT(6))) > 0) ? true : false;
      RadioPara.reception_on_active_station = ((ulState & BIT(7)) > 0) ? true : false;
      RadioPara.reception_standby = ((ulState & BIT(8)) > 0) ? true : false;
      RadioPara.reception_active = (RadioPara.reception_on_active_station || RadioPara.reception_standby);
      RadioPara.changed = true;
    }
    break;
  default:
    break;
  }

  /* return the number of converted characters */
  return processed; 
}

/**
 * Sets the active frequency on the radio.
 */
bool AR62xxDevice::PutActiveFrequency(RadioFrequency frequency,
                                      const TCHAR *name,
                                      OperationEnvironment &env)
{
  unsigned int ufreq = frequency.GetKiloHertz();
  double freq = ufreq / 1000.0;
  bool done = AR620xPutFreqActive(freq, (const TCHAR *)name, env);
  return done;
}

/**
 * Sets the standby (passive) frequency on the radio.
 */
bool AR62xxDevice::PutStandbyFrequency(RadioFrequency frequency,
                                       const TCHAR *name,
                                       OperationEnvironment &env)
{
  unsigned int ufreq = frequency.GetKiloHertz();
  double freq = ufreq / 1000.0;
  bool done = AR620xPutFreqStandby(freq, (const TCHAR *)name, env);
  return done;
}

/*
 * Assign the selected port on Object construction
 */
static Device *AR62xxCreateOnPort(const DeviceConfig &config, Port &comPort)
{
  return new AR62xxDevice(comPort);
}

/*
 * Driver registration in XCSoar, connect to a serial port
 */
const struct DeviceRegister ar62xx_driver = {
    _T("AR62xx"),
    _T("AR62xx"),
    DeviceRegister::NO_TIMEOUT | DeviceRegister::RAW_GPS_DATA,
    AR62xxCreateOnPort,
};
