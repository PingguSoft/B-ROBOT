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

#include <stdarg.h>
#include "esp8266.h"

void ESP8266::begin(SerialProtocol *serial)
{
    u8  bufMac[2];
    u8  buf[50];

    mIdx = 0;
    mSerial = serial;

    delay(3000);
    mSerial->clearRX();

    mSerial->write(F("+++"), 3);
    delay(100);

    send(F("AT"), F("OK"), NULL, 0, 1000);
    send(F("AT+RST"), F("OK"), NULL, 0, 2000);
    wait(F("ready"), NULL, 5, 6000);
    send(F("ATE0"), F("OK"), NULL, 0, 1000);
    send(F("AT+GMR"), F("OK"), NULL, 0, 5000);
    send(F("AT+CIPSTAMAC?"), F("+CIPSTAMAC:"), buf, 30, 3000);
    bufMac[0] = buf[27];
    bufMac[1] = buf[28];
    send(F("AT+CWQAP"), F("OK"), NULL, 0, 3000);
    send(F("AT+CWMODE=2"), F("OK"), NULL, 0, 3000);

    strcpy_P((char*)buf, PSTR("AT+CWSAP=\"B-ROBOT_XX\",\"12345678\",5,3"));
    buf[18] = bufMac[0];
    buf[19] = bufMac[1];
    send(buf, F("OK"), NULL, 0, 3000);

    send(F("AT+CIPMUX=0"), F("OK"), NULL, 0, 3000);
    send(F("AT+CIPMODE=1"), F("OK"), NULL, 0, 3000);
    send(F("AT+CIPSTART=\"UDP\",\"192.168.4.2\",2223,2222,0"), F("OK"), NULL, 0, 3000);
    send(F("AT+CIPSEND"), F(">"), NULL, 0, 2000);
}

u8 *ESP8266::wait(const __FlashStringHelper *resp, u8 *respBuf, u8 reqSize, u16 timeout)
{
    u8      respLen = strlen_P((PGM_P)resp);
    u8      bufTmp[reqSize];
    u8      c;
    u8      pos = 0;
    u8      found = FALSE;
    u8      *buf = (respBuf == NULL) ? bufTmp : respBuf;
    u32     tsStart = millis();

    while (TRUE) {
        if (millis() - tsStart > timeout) {
            LOG(F("TIMEOUT!!!\n"));
            return NULL;
        }

        if (mSerial->available()) {
            c = mSerial->read();
            LOG(F("%c"), c);

            if (found && pos < reqSize) {
                buf[pos++] = c;
                if (pos == reqSize) {
                    LOG(F(" COMPLETE!\n"));
                    return buf;
                }
            } else {
                if (pos < respLen && c == pgm_read_byte((PGM_P)resp + pos)) {
                    buf[pos++] = c;
                    if (pos == respLen) {
                        LOG(F("=>"));
                        LOG(resp);
                        LOG(F(" FOUND!\n"));
                        found = TRUE;
                        if (pos == reqSize) {
                            return buf;
                        }
                    }
                } else {
                    pos = 0;
                }
            }
        } else {
            delay(5);
        }
    }
}

u8 *ESP8266::send(const __FlashStringHelper *cmd, const __FlashStringHelper *resp, u8 *respBuf, u8 reqSize, u16 timeout)
{
    mIdx++;
    LOG(cmd);
    mSerial->write(cmd, strlen_P((PGM_P)cmd));
    mSerial->write(F("\r\n"), 2);
    if (reqSize == 0) {
        reqSize = strlen_P((char*)resp);
    }
    return wait(resp, respBuf, reqSize, timeout);
}

u8 *ESP8266::send(u8 *cmd, const __FlashStringHelper *resp, u8 *respBuf, u8 reqSize, u16 timeout)
{
    mIdx++;
    LOG((char*)cmd);
    mSerial->write(cmd, strlen(cmd));
    mSerial->write(F("\r\n"), 2);
    if (reqSize == 0) {
        reqSize = strlen_P((PGM_P)resp);
    }
    return wait(resp, respBuf, reqSize, timeout);
}
