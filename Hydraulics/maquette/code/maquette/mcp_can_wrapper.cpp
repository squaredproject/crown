/*
  mcp_can_wrapper.cpp
  2012 Copyright (c) Medea Games.  All right reserved.

  Author:bulkow (brian@bulkowski.org)
  2018-08-12

  Contributor:

  Brian Bulkowski

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
#include "mcp_can_wrapper.h"
#include "mcp_can.h"
// Debugging for this project...
//#include "putstr.h"

MCP_CAN *CAN = NULL;

//MCP_CAN CAN(SPI_CS_PIN);


byte mcp_can_begin(byte speedset, byte pin) {
  if (!CAN) {
    CAN = new MCP_CAN(pin);
  }
  return( CAN->begin(MCP_ANY, speedset, MCP_8MHZ) );  // Usig 8MHz here because that's the crystal speed on the boards I'm using...
}

byte mcp_can_check_receive(void) {
  return( CAN->checkReceive() );
}

/*
unsigned long mcp_can_get_can_id(void) {
  return( CAN->getCanId() );
}
*/

byte mcp_can_send(unsigned long id, byte ext, byte len, const byte * buf) {
  return( CAN->sendMsgBuf(id,ext,len,buf) );
}

byte mcp_can_receive(unsigned long *id, unsigned char *len, byte *buf){
  return( CAN->readMsgBuf(id, len, buf) );
}

byte mcp_can_check_error(void){
  return( CAN->checkError());
}

byte mcp_can_get_error(void) {
  return ( CAN->getError() );
}

byte mcp_can_set_mode(unsigned char mode) {
  return( CAN->setMode(mode));
}




