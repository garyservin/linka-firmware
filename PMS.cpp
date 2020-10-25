#include "Arduino.h"
#include "PMS.h"

/* From https://github.com/SwapBap/PMS */

PMS::PMS(Stream& stream, bool fake = false)
{
  this->_stream = &stream;
  this->_fake = fake;
}

// Standby mode. For low power consumption and prolong the life of the sensor.
void PMS::sleep()
{
  uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73 };
  if (!_fake) {
    _stream->write(command, sizeof(command));
  }
}

// Operating mode. Stable data should be got at least 30 seconds after the sensor wakeup from the sleep mode because of the fan's performance.
void PMS::wakeUp()
{
  uint8_t command[] = { 0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74 };
  if (!_fake) {
    _stream->write(command, sizeof(command));
  }
}

// Active mode. Default mode after power up. In this mode sensor would send serial data to the host automatically.
void PMS::activeMode()
{
  uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 };
  if (!_fake) {
    _stream->write(command, sizeof(command));
  }
  _mode = MODE_ACTIVE;
}

// Passive mode. In this mode sensor would send serial data to the host only for request.
void PMS::passiveMode()
{
  uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70 };
  if (!_fake) {
    _stream->write(command, sizeof(command));
  }
  _mode = MODE_PASSIVE;
}

// Request read in Passive Mode.
void PMS::requestRead()
{
  if (_mode == MODE_PASSIVE)
  {
    uint8_t command[] = { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 };
    if (!_fake) {
      _stream->write(command, sizeof(command));
    }
  }
}

// Non-blocking function for parse response.
bool PMS::read(DATA& data)
{
  _data = &data;
  loop();

  return _status == STATUS_OK;
}

// Blocking function for parse response. Default timeout is 1s.
bool PMS::readUntil(DATA& data, uint16_t timeout)
{
  _data = &data;
  create_fake_data();
  uint32_t start = millis();
  do
  {
    loop();
    if (_status == STATUS_OK) break;
  } while (millis() - start < timeout);

  return _status == STATUS_OK;
}

void PMS::loop()
{
  _status = STATUS_WAITING;
  if (_fake || _stream->available())
  {
    uint8_t ch;
    if (! _fake)
    {
      ch = _stream->read();
    }
    else
    {
      ch = _fake_data[_index];
    }

    switch (_index)
    {
      case 0:
        if (ch != 0x42)
        {
          return;
        }
        _calculatedChecksum = ch;
        break;

      case 1:
        if (ch != 0x4D)
        {
          _index = 0;
          return;
        }
        _calculatedChecksum += ch;
        break;

      case 2:
        _calculatedChecksum += ch;
        _frameLen = ch << 8;
        break;

      case 3:
        _frameLen |= ch;
        // Unsupported sensor, different frame length, transmission error e.t.c.
        if (_frameLen != 2 * 9 + 2 && _frameLen != 2 * 13 + 2)
        {
          _index = 0;
          return;
        }
        _calculatedChecksum += ch;
        break;

      default:
        if (_index == _frameLen + 2)
        {
          _checksum = ch << 8;
        }
        else if (_index == _frameLen + 2 + 1)
        {
          _checksum |= ch;
          if (_calculatedChecksum == _checksum)
          {
            _status = STATUS_OK;

            // Standard Particles, CF=1.
            _data->PM_SP_UG_1_0 = makeWord(_payload[0], _payload[1]);
            _data->PM_SP_UG_2_5 = makeWord(_payload[2], _payload[3]);
            _data->PM_SP_UG_10_0 = makeWord(_payload[4], _payload[5]);

            // Atmospheric Environment.
            _data->PM_AE_UG_1_0 = makeWord(_payload[6], _payload[7]);
            _data->PM_AE_UG_2_5 = makeWord(_payload[8], _payload[9]);
            _data->PM_AE_UG_10_0 = makeWord(_payload[10], _payload[11]);

            // Total particles
            _data->PM_TOTALPARTICLES_0_3 = makeWord(_payload[12], _payload[13]);
            _data->PM_TOTALPARTICLES_0_5 = makeWord(_payload[14], _payload[15]);
            _data->PM_TOTALPARTICLES_1_0 = makeWord(_payload[16], _payload[17]);
            _data->PM_TOTALPARTICLES_2_5 = makeWord(_payload[18], _payload[19]);
            _data->PM_TOTALPARTICLES_5_0 = makeWord(_payload[20], _payload[21]);
            _data->PM_TOTALPARTICLES_10_0 = makeWord(_payload[22], _payload[23]);
          }

          _index = 0;
          return;
        }
        else
        {
          _calculatedChecksum += ch;
          uint8_t payloadIndex = _index - 4;

          // Payload is common to all sensors (first 2x6 bytes).
          if (payloadIndex < sizeof(_payload))
          {
            _payload[payloadIndex] = ch;
          }
        }

        break;
    }

    _index++;
  }
}

void PMS::create_fake_data()
{
  uint16_t calculatedChecksum = 0x00;

  randomSeed(analogRead(0));
  uint16_t pms_1_0 = random(0, 500);
  uint16_t pms_2_5 = random(0, 500);
  uint16_t pms_10 = random(0, 500);
  
  for (uint8_t index = 0; index < 32; index++)
  {
    switch (index)
    {
      case 0:
        _fake_data[index] = 0x42;
        calculatedChecksum += _fake_data[index];
        break;

      case 1:
        _fake_data[index] = 0x4D;
        calculatedChecksum += _fake_data[index];
        break;

      case 2:
        _fake_data[index] = 0x00;
        calculatedChecksum += _fake_data[index];
        break;

      case 3:
        _fake_data[index] = 28;
        calculatedChecksum += _fake_data[index];
        break;

      case 10:
        _fake_data[index] = (pms_1_0 >> 8) & 0xFF;
        calculatedChecksum += _fake_data[index];
        break;

      case 11:
        _fake_data[index] = pms_1_0 & 0xFF;
        calculatedChecksum += _fake_data[index];
        break;
        
      case 12:
        _fake_data[index] = (pms_2_5 >> 8) & 0xFF;
        calculatedChecksum += _fake_data[index];
        break;

      case 13:
        _fake_data[index] = pms_2_5 & 0xFF;
        calculatedChecksum += _fake_data[index];
        break;
      
      case 14:
        _fake_data[index] = (pms_10 >> 8) & 0xFF;
        calculatedChecksum += _fake_data[index];
        break;

      case 15:
        _fake_data[index] = pms_10 & 0xFF;
        calculatedChecksum += _fake_data[index];
        break;
      
      case 30:
        _fake_data[index] = (calculatedChecksum >> 8) & 0xFF;
        break;

      case 31:
        _fake_data[index] = calculatedChecksum & 0xFF;
        break;

      default:
        _fake_data[index] = 0;
        calculatedChecksum += _fake_data[index];
        break;
    }
  }
}
