/*
  mcp_can_wrapper.h
  2012 Copyright (c) Medea Games.  All right reserved.

  Author:bulkow (brian@bulkowski.org)
  2018-08-12

  Contributor:

  bbulkow
  carolyn wales

  This is a C wrapper for Corey J Fowler's Arduino CAN bus library

  The MIT License (MIT)

  Copyright (c) 2013 Seeed Technology Inc.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/
#ifndef _MCP_CAN_WRAP_H_
#define _MCP_CAN_WRAP_H_

#define SPI_CS_PIN 53

// redefines from mcp_can_defs.h, which we need but can't include

#define CAN_WRAPPER_4K096BPS 0
#define CAN_WRAPPER_5KBPS 1
#define CAN_WRAPPER_10KBPS 2
#define CAN_WRAPPER_20KBPS 3
#define CAN_WRAPPER_31K25BPS 4
#define CAN_WRAPPER_33K3BPS 5
#define CAN_WRAPPER_40KBPS 6
#define CAN_WRAPPER_50KBPS 7
#define CAN_WRAPPER_80KBPS 8
#define CAN_WRAPPER_100KBPS 9
#define CAN_WRAPPER_125KBPS 10
#define CAN_WRAPPER_200KBPS 11
#define CAN_WRAPPER_250KBPS 12
#define CAN_WRAPPER_500KBPS 13
#define CAN_WRAPPER_1000KBPS 14

#define CAN_WRAPPER_OK (0)
#define CAN_WRAPPER_FAILINIT (1)
#define CAN_WRAPPER_FAILTX (2)
#define CAN_WRAPPER_MSGAVAIL (3)
#define CAN_WRAPPER_NOMSG (4)
#define CAN_WRAPPER_CTRLERROR (5)

#define CAN_WRAPPER_MODE_NORMAL 0x00
#define CAN_WRAPPER_MODE_SLEEP 0x20
#define CAN_WRAPPER_MODE_LOOPBACK 0x40
#define CAN_WRAPPER_MODE_LISTENONLY 0x60

#ifdef __cplusplus
extern "C" {
#endif

unsigned char mcp_can_begin(
    unsigned char speedset,
    unsigned char pin); // does not expose underlying clockset - assumes 16Mhz
unsigned char mcp_can_check_receive(void);
unsigned char mcp_can_receive(unsigned long *id, unsigned char *len,
                              unsigned char *buf);
unsigned char mcp_can_send(unsigned long id, unsigned char ext,
                           unsigned char len, unsigned char *buf);
unsigned char mcp_can_check_error(void);
unsigned char mcp_can_set_mode(unsigned char mode);

#ifdef __cplusplus
}
#endif

#endif // MCP_CAN_WRAPPER
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
