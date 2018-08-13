/*
  mcp_can_wrapper.h
  2012 Copyright (c) Medea Games.  All right reserved.

  Author:bulkow (brian@bulkowski.org)
  2018-08-12

  Contributor:

  bbulkow

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

#include "mcp_can.h"
#include "mcp_can_dfs.h"


extern "C" {

byte mcp_can_begin(byte speedset);   // does not expose underlying clockset - assumes 16Mhz
byte mcp_can_check_receive(void);
unsigned long mcp_can_get_can_id(void);

}

#endif // MCP_CAN_WRAPPER
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
