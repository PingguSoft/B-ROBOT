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


#ifndef _SERIAL_PROTOCOL_H_
#define _SERIAL_PROTOCOL_H_
#include <stdarg.h>
#include "common.h"
#include "config.h"
#include "utils.h"

#define MAX_PACKET_SIZE 64

class SerialProtocol
{

public:
    typedef enum {
        MSP_SET_USER_BUTTON = 51,
        MSP_IDENT  = 100,
        MSP_STATUS = 101,
        MSP_ALTITUDE = 109,
        MSP_ANALOG = 110,
        MSP_MISC   = 114,
        MSP_SET_RAW_RC = 200,
        MSP_SET_MISC   = 207,
    } MSP_T;

    SerialProtocol();
    ~SerialProtocol();

    void begin(u32 baud, u8 config=SERIAL_8N1);

    static void clearTX(void);
    static void clearRX(void);
    static u8   available(void);
    static u8   read(void);
    static u8   read(u8 *buf);
    static u8   read(u8 *buf, u8 size);
    static void write(const __FlashStringHelper *buf, u8 size);
    static void write(char *buf, u8 size);


    static void printf(char *fmt, ... );
    static void printf(const __FlashStringHelper *fmt, ... );
    static void dumpHex(char *name, u8 *data, u16 cnt);

    u8   handleMSP(void);
    u8   handleOSC(void);
    void registerCallback(s8 (*callback)(u8 cmd, u8 *data, u8 size, u8 *res));

private:
    typedef enum
    {
        STATE_IDLE,
        STATE_HEADER_START,
        STATE_HEADER_M,
        STATE_HEADER_ARROW,
        STATE_HEADER_SIZE,
        STATE_HEADER_CMD
    } STATE_T;
    //

    // variables
    u8   mRxPacket[MAX_PACKET_SIZE];

    u8   mState;
    u8   mOffset;
    u8   mDataSize;
    u8   mCheckSum;
    u8   mCmd;
    u8   mCheckSumTX;
    s8  (*mCallback)(u8 cmd, u8 *data, u8 size, u8 *res);

    void putMSPChar2TX(u8 data);
    void sendMSPResp(bool ok, u8 cmd, u8 *data, u8 size);
    void evalMSPCommand(u8 cmd, u8 *data, u8 size);

    void evalOSCCommand(u8 cmd, u8 *data, u8 size);
};

#endif
