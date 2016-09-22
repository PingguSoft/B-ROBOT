/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 see <http://www.gnu.org/licenses/>
*/

#ifndef _ESP8266_H_
#define _ESP8266_H_
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "common.h"
#include "config.h"
#include "utils.h"
#include "SerialProtocol.h"


class ESP8266
{
public:
    void begin(SerialProtocol *serial);

private:
    void startUDPClient(void);
    void startTCPServer(void);
    u8 *wait(const __FlashStringHelper *resp, u8 *respBuf, u8 reqSize, u16 timeout);
    u8 *send(const __FlashStringHelper *cmd, const __FlashStringHelper *resp, u8 *respBuf, u8 reqSize, u16 timeout);
    u8 *send(u8 *cmd, const __FlashStringHelper *resp, u8 *respBuf, u8 reqSize, u16 timeout);
    SerialProtocol *mSerial;
};

#endif
