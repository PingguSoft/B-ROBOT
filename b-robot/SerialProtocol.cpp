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

// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "SerialProtocol.h"

#if defined(__AVR_ATmega328P__)
#define UCSRB   UCSR0B
#define UDR     UDR0
#define UDRIE   UDRIE0
#define RXEN    RXEN0
#define TXEN    TXEN0
#define RXCIE   RXCIE0
#define UCSRA   UCSR0A
#define U2X     U2X0
#define UBRRL   UBRR0L
#define UBRRH   UBRR0H
#define UCSRC   UCSR0C
#define RXC     RXC0
#elif defined(__AVR_ATmega32U4__)
#define UCSRB   UCSR1B
#define UDR     UDR1
#define UDRIE   UDRIE1
#define RXEN    RXEN1
#define TXEN    TXEN1
#define RXCIE   RXCIE1
#define UCSRA   UCSR1A
#define U2X     U2X1
#define UBRRL   UBRR1L
#define UBRRH   UBRR1H
#define UCSRC   UCSR1C
#define RXC     RXC1
#endif


#define MAX_BUF_SIZE 128

struct ringBuf {
    volatile u8 buffer[MAX_BUF_SIZE];
    volatile u8 head;
    volatile u8 tail;
};

struct ringBuf mRxRingBuf = { {0}, 0, 0 };
struct ringBuf mTxRingBuf = { {0}, 0, 0 };

static void putChar(struct ringBuf *buf, u8 data)
{
    u8 head;

    head = buf->head;
    buf->buffer[head] = data;
    if (++head >= MAX_BUF_SIZE)
        head = 0;
    buf->head = head;
}

static u8 getChar(struct ringBuf *buf)
{
    u8 tail = buf->tail;
    u8 ch   = buf->buffer[tail];
    if (buf->head != tail) {
        if (++tail >= MAX_BUF_SIZE)
            tail = 0;
        buf->tail = tail;
    }
    return ch;
}

static void putChar2TX(u8 data)
{
    u8 head;
    u8 tail;
    u8 free;

    do {
        head = mTxRingBuf.head;
        tail = mTxRingBuf.tail;

        if (head >= tail)
            free = MAX_BUF_SIZE - 1 - head + tail;
        else
            free = tail - head - 1;

        if (free == 0)
            delayMicroseconds(10);
    } while (free == 0);

    putChar(&mTxRingBuf, data);
}

static __inline void flushTX(void)
{
    UCSRB |= BV(UDRIE);
}

static u8 sAvailable(struct ringBuf *buf)
{
    return ((u8)(buf->head - buf->tail)) % MAX_BUF_SIZE;
}


#if defined(__AVR_ATmega328P__) && !__STD_SERIAL__
ISR(USART_RX_vect)
{
    putChar(&mRxRingBuf, UDR);
}

ISR(USART_UDRE_vect)
{
    struct ringBuf *buf = &mTxRingBuf;

    u8 tail = buf->tail;
    if (buf->head != tail) {
        UDR = buf->buffer[tail];
        if (++tail >= MAX_BUF_SIZE)
            tail = 0;
        buf->tail = tail;
    }

    // disable transmitter UDRE interrupt
    if (tail == buf->head)
        UCSRB &= ~BV(UDRIE);
}
#elif defined(__AVR_ATmega32U4__)
ISR(USART1_RX_vect)
{
    putChar(&mRxRingBuf, UDR);
}

ISR(USART1_UDRE_vect)
{
    struct ringBuf *buf = &mTxRingBuf;

    u8 tail = buf->tail;
    if (buf->head != tail) {
        UDR = buf->buffer[tail];
        if (++tail >= MAX_BUF_SIZE)
            tail = 0;
        buf->tail = tail;
    }

    // disable transmitter UDRE interrupt
    if (tail == buf->head)
        UCSRB &= ~BV(UDRIE);
}
#endif

SerialProtocol::SerialProtocol()
{
}

SerialProtocol::~SerialProtocol()
{
    UCSRB &= ~(BV(RXEN) | BV(TXEN) | BV(RXCIE) | BV(UDRIE));
}

void SerialProtocol::begin(u32 baud, u8 config)
{
    memset(&mRxRingBuf, 0, sizeof(mRxRingBuf));
    memset(&mTxRingBuf, 0, sizeof(mTxRingBuf));

    u16 ubrr = (((F_CPU) + 8UL * (baud)) / (16UL * (baud)) -1UL);
    u8  use_2x = 0;
    u8  baud_tol = 2;

    if (100 * (F_CPU) > (16 * ((ubrr) + 1)) * (100 * (baud) + (baud) * (baud_tol))) {
        use_2x = 1;
    } else if (100 * (F_CPU) < (16 * ((ubrr) + 1)) * (100 * (baud) - (baud) * (baud_tol))) {
        use_2x = 1;
    }

    cli();
    if (use_2x) {
        ubrr =  (((F_CPU) + 4UL * (baud)) / (8UL * (baud)) -1UL);
        if (100 * (F_CPU) > (8 * ((ubrr) + 1)) * (100 * (baud) + (baud) * (baud_tol))) {
            // Baud rate achieved is higher than allowed !!!
        } else if (100 * (F_CPU) < (8 * ((ubrr) + 1)) * (100 * (baud) - (baud) * (baud_tol))) {
            // Baud rate achieved is lower than allowed !!!
        }
        UCSRA |= BV(U2X);
    } else {
        UCSRA &= ~BV(U2X);
    }

    UBRRL = (ubrr & 0xff);
    UBRRH = (ubrr >> 8);
	UCSRC = config;

    while (UCSRA & BV(RXC) )                      //flush receive buffer
		u8 data = UDR;

	//enable reception and RC complete interrupt
	UCSRB = BV(RXEN) | BV(RXCIE) | BV(TXEN);    //rx enable and interrupt
    sei();
}

void SerialProtocol::clearTX(void)
{
    cli();
    UCSRB &= ~BV(UDRIE);
    memset(&mTxRingBuf, 0, sizeof(mTxRingBuf));
    sei();
}

void SerialProtocol::clearRX(void)
{
    cli();
    memset(&mRxRingBuf, 0, sizeof(mRxRingBuf));
    sei();
}

u8 SerialProtocol::available(void)
{
    return sAvailable(&mRxRingBuf);
}

u8 SerialProtocol::read(void)
{
    return getChar(&mRxRingBuf);
}

u8 SerialProtocol::read(u8 *buf)
{
    return read(buf, 255);
}

u8 SerialProtocol::read(u8 *buf, u8 size)
{
    u8 rsize = min(sAvailable(&mRxRingBuf), size);

    for (u8 i = 0; i < rsize; i++)
        *buf++ = getChar(&mRxRingBuf);

    return rsize;
}

void SerialProtocol::write(const __FlashStringHelper *buf, u8 size)
{
    for (u8 i = 0; i < size; i++)
        putChar2TX(pgm_read_byte((PGM_P)buf + i));
    flushTX();
}

void SerialProtocol::write(char *buf, u8 size)
{
    for (u8 i = 0; i < size; i++)
        putChar2TX(*(buf + i));
    flushTX();
}

/*
*****************************************************************************************
* Utility functions for debugging
*****************************************************************************************
*/
void SerialProtocol::printf(const __FlashStringHelper *fmt, ...)
{
    char buf[128]; // resulting string limited to 128 chars

    va_list args;
    va_start (args, fmt);

#ifdef __AVR__
    vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args); // progmem for AVR
#else
    vsnprintf(buf, sizeof(buf), (const char *)fmt, args); // for the rest of the world
#endif
    va_end(args);

    for (u8 i = 0; i < strlen(buf); i++)
        putChar2TX(buf[i]);
    flushTX();
}

void SerialProtocol::printf(char *fmt, ...)
{
    char buf[128]; // resulting string limited to 128 chars
    va_list args;

    va_start (args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    for (u8 i = 0; i < strlen(buf); i++)
        putChar2TX(buf[i]);
    flushTX();
}

void SerialProtocol::dumpHex(char *name, u8 *data, u16 cnt)
{
    u8  i;
    u8  b;
    u16 addr = 0;

    LOG("-- %s buf size : %d -- \n", name, cnt);
    while (cnt) {
        LOG("%08x - ", addr);

        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = *(data + i);
            LOG("%02x ", b);
        }

        LOG(" : ");
        for (i = 0; (i < 16) && (i < cnt); i ++) {
            b = *(data + i);
            if ((b > 0x1f) && (b < 0x7f))
                LOG("%c", b);
            else
                LOG(".");
        }
        LOG("\n");
        data += i;
        addr += i;
        cnt  -= i;
    }
}

void SerialProtocol::registerCallback(s8 (*callback)(u8 cmd, u8 *data, u8 size, u8 *res))
{
    mState = STATE_IDLE;
    mCallback = callback;
}

/*
*****************************************************************************************
* MSP
*****************************************************************************************
*/
#if (__FEATURE_CONTROLLER__ == __CON_MSP__)
void SerialProtocol::putMSPChar2TX(u8 data)
{
    mCheckSumTX ^= data;
    putChar2TX(data);
}

void SerialProtocol::sendMSPResp(bool ok, u8 cmd, u8 *data, u8 size)
{
#if __FEATURE_WIFI_DEFAULT__
    printf(F("AT+CIPSEND=0,%d\r\n"), 6 + size);
    delay(2);
#endif

    putMSPChar2TX('$');
    putMSPChar2TX('M');
    putMSPChar2TX((ok ? '>' : '!'));
    mCheckSumTX = 0;
    putMSPChar2TX(size);
    putMSPChar2TX(cmd);
    for (u8 i = 0; i < size; i++)
        putMSPChar2TX(*data++);
    putMSPChar2TX(mCheckSumTX);
    flushTX();
}

void SerialProtocol::evalMSPCommand(u8 cmd, u8 *data, u8 size)
{
    u8  buf[22];
    u16 *rc;

    memset(&buf, 0, sizeof(buf));

    switch (cmd) {
        case MSP_IDENT:
            buf[0] = 240;
            buf[1] = 3;
            sendMSPResp(TRUE, cmd, buf, 7);
            break;

        case MSP_MISC:
            rc = (u16*)buf;
            rc[2] = 2000;
            rc[3] = 1000;
            rc[4] = 1000;
            buf[18] = 100;
            buf[19] = 110;
            buf[20] = 105;
            buf[21] = 100;
            sendMSPResp(TRUE, cmd, buf, 22);
            break;

        default:
            if (mCallback) {
                s8 ret = (*mCallback)(cmd, data, size, buf);
                if (ret >= 0) {
                    sendMSPResp(TRUE, cmd, buf, ret);
                }
            }
            break;
    }
}

u8 SerialProtocol::handleMSP(void)
{
    u8 ret = 0;
    u8 rxSize = available();

    if (rxSize == 0)
        return ret;

    while (rxSize--) {
        u8 ch = read();

        switch (mState) {
            case STATE_IDLE:
                if (ch == '$')
                    mState = STATE_HEADER_START;
                break;

            case STATE_HEADER_START:
                mState = (ch == 'M') ? STATE_HEADER_M : STATE_IDLE;
                break;

            case STATE_HEADER_M:
                mState = (ch == '<') ? STATE_HEADER_ARROW : STATE_IDLE;
                break;

            case STATE_HEADER_ARROW:
                if (ch > MAX_PACKET_SIZE) { // now we are expecting the payload size
                    mState = STATE_IDLE;
                    continue;
                }
                mDataSize = ch;
                mCheckSum = ch;
                mOffset   = 0;
                mState    = STATE_HEADER_SIZE;
                break;

            case STATE_HEADER_SIZE:
                mCmd       = ch;
                mCheckSum ^= ch;
                mState     = STATE_HEADER_CMD;
                break;

            case STATE_HEADER_CMD:
                if (mOffset < mDataSize) {
                    mCheckSum           ^= ch;
                    mRxPacket[mOffset++] = ch;
                } else {
                    if (mCheckSum == ch) {
                        ret = mCmd;
                        evalMSPCommand(ret, mRxPacket, mDataSize);
                    }
                    mState = STATE_IDLE;
                    //rxSize = 0;             // no more than one command per cycle
                }
                break;
        }
    }
    return ret;
}

#elif (__FEATURE_CONTROLLER__ == __CON_OSC__)

/*
*****************************************************************************************
* OSC
*****************************************************************************************
*/
void SerialProtocol::evalOSCCommand(u8 cmd, u8 *data, u8 size)
{
    u8  buf[22];
    u16 *rc;

    memset(&buf, 0, sizeof(buf));

    switch (cmd) {
        default:
            if (mCallback) {
                s8 ret = (*mCallback)(cmd, data, size, buf);
                if (ret >= 0) {

                }
            }
            break;
    }
}

u8 SerialProtocol::handleOSC(void)
{
    u8 ret = 0;
    u8 rxSize = available();

    if (rxSize == 0)
        return ret;

    while (rxSize--) {
        u8 ch = read();

        switch (mState) {
            case STATE_IDLE:
                if (ch == '/') {
                    mState = STATE_HEADER_START;
                }
                break;

            case STATE_HEADER_START:
                if ('0' <= ch && ch <= '9') {
                    mState = STATE_HEADER_M;
                    mCmd   = ch - '0';
                } else {
                    mState = STATE_IDLE;
                }
                break;

            case STATE_HEADER_M:
                mState = (ch == '/') ? STATE_HEADER_ARROW : STATE_IDLE;
                break;

            case STATE_HEADER_ARROW:
                if (ch == 'f' || ch == 't' || ch == 'p' || ch == 'x') {
                    if (ch == 'x') {
                        mDataSize = 24 - 3;
                    } else {
                        mDataSize = 20 - 3;
                    }
                    mOffset   = 0;
                    mState    = STATE_HEADER_CMD;
                    mRxPacket[mOffset++] = ch;
                } else {
                    mState = STATE_IDLE;
                    continue;
                }
                break;

            case STATE_HEADER_CMD:
                if (mOffset < mDataSize) {
                    mRxPacket[mOffset++] = ch;
                    if (mOffset == mDataSize) {
                        ret = mCmd;
                        evalOSCCommand(ret, mRxPacket, mDataSize);
                        mState = STATE_IDLE;
                        //rxSize = 0;             // no more than one command per cycle
                    }
                }
                break;
        }
    }
    return ret;
}
#endif
