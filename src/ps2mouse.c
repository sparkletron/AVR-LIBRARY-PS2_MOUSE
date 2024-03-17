/*******************************************************************************
 * @file    ps2mouse.c
 * @author  Jay Convertino(electrobs@gmail.com)
 * @date    2024.03.16
 * @brief   ps2 mouse driver
 * @version 0.0.0
 *
 * @TODO
 *  - Cleanup interface
 *
 * @license mit
 *
 * Copyright 2024 Johnathan Convertino
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/


#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <avr/common.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ps2mouse.h"

volatile int toggle = 0;

struct s_ps2 g_ps2;

enum callbackStates {waiting, no_cmd, resend_cmd, ack_cmd, ready_cmd, mouse_id};

struct s_ps2mouse
{
  volatile uint8_t lastCMD;
  volatile uint8_t id;
  volatile enum callbackStates callbackState;
} g_ps2mouse;

//helper functions
//sends commands
void sendCommand(uint8_t command);
//sends data, (does not update LAST_CMD);
void sendData(uint8_t data);
//waits for callback to return, returns state of the callback for error
//handling (none implimented at this time).
enum callbackStates waitingForCallback();
//wait for idle state in rx/tx irq
void waitForDataIdle();
//wait for device ready AA, 00
void waitForDevReady();
//wait for command ack, eventually add timeouts to these methods
void waitForCMDack();
//wait for mouse id state from callback
void waitForMouseID();
//Converts PS2 data to raw data, this performs checks on the data.
//If it returns 0 then the data is invalid.
uint8_t convertToRaw(uint16_t ps2data);
//generate odd parity
uint8_t oddParityGen(uint16_t data);
//convert 8 bit data into a 11 bit packet
uint16_t dataToPacket(uint8_t data);
//copys data passed to it to the internal send buffer.
void copyPacketToBuffer(uint16_t packet);
//start trasmission of data to the mouse.
void startTransmit();
//CALLBACK ROUTINES
//set the id of the mouse if last command was get id and ready command received.
void setID(uint16_t ps2Data);
//check the response to mouse command sent and perform needed operations
void checkMouseResponse(uint16_t ps2Data);
//Default callback for recv that processes data and then hands it off to the user callback.
void extractData(uint16_t ps2data);

void initPS2mouse(t_PS2userRecvCallback PS2recvCallback, void (*setPS2_PORT_Device)(struct s_ps2 *p_device), volatile uint8_t *p_port, uint8_t clkPin, uint8_t dataPin)
{
  uint8_t tmpSREG = 0;

  tmpSREG = SREG;
  cli();

  if(p_port == NULL) return;

  if(PS2recvCallback == NULL) return;

  if(setPS2_PORT_Device == NULL) return;

  setPS2_PORT_Device(&g_ps2);

  memset(&g_ps2mouse, 0, sizeof(g_ps2mouse));

  memset(&g_ps2, 0, sizeof(g_ps2));

  g_ps2.clkPin = clkPin;
  g_ps2.dataPin = dataPin;
  g_ps2.p_port = p_port;
  g_ps2.lastAckState = ack;
  g_ps2.dataState = idle;
  g_ps2.userRecvCallback = PS2recvCallback;
  g_ps2.recvCallback = NULL;

  *(g_ps2.p_port - 1) &= ~(1 << g_ps2.clkPin);
  *(g_ps2.p_port - 1) &= ~(1 << g_ps2.dataPin);

  if(g_ps2.p_port == &PORTB)
  {
    PCICR |= 1 << PCIE0;
    PCMSK0 |= 1 << g_ps2.clkPin;
  }
  else if(g_ps2.p_port == &PORTC)
  {
    PCICR |= 1 << PCIE1;
    PCMSK1 |= 1 << g_ps2.clkPin;
  }
  else
  {
    PCICR |= 1 << PCIE2;
    PCMSK2 |= 1 << g_ps2.clkPin;
  }

  SREG = tmpSREG;

  sei();

  //initialize mouse using PC init method
  resetPS2mouse();

  enablePS2mouse();

  obtainPS2mouseID();
}

void resetPS2mouse()
{
  sendCommand(CMD_RESET);

  waitForCMDack();

  waitForDevReady();

  waitForMouseID();
}

void disablePS2mouse()
{
  sendCommand(CMD_DISABLE);
}

void enablePS2mouse()
{
  sendCommand(CMD_ENABLE);
}

void setPS2sampleRate(uint8_t rate)
{
  sendCommand(CMD_SET_RATE);

  waitForCMDack();

  sendData(rate);

  waitForCMDack();
}

void obtainPS2mouseID()
{
  sendCommand(CMD_READ_ID);

  waitForMouseID();
}

uint8_t getPS2mouseID()
{
  return g_ps2mouse.id;
}

//helper functions
void sendCommand(uint8_t command)
{
  g_ps2mouse.lastCMD = command;

  sendData(command);
}

void sendData(uint8_t data)
{
  uint8_t tmpSREG = 0;
  uint16_t tempConv = 0;

  tmpSREG = SREG;

  waitForDataIdle();

  cli();

  tempConv = dataToPacket(data);

  copyPacketToBuffer(tempConv);

  startTransmit();

  SREG = tmpSREG;

  waitingForCallback();
}

enum callbackStates waitingForCallback()
{
  while(g_ps2mouse.callbackState == waiting);

  return g_ps2mouse.callbackState;
}

void waitForDataIdle()
{
  while(g_ps2.dataState != idle);
}

void waitForDevReady()
{
  g_ps2mouse.callbackState = waiting;

  g_ps2.recvCallback = &checkMouseResponse;

  while(g_ps2mouse.callbackState != ready_cmd);
}

void waitForMouseID()
{
  while(g_ps2mouse.callbackState != mouse_id);
}

//FA is ACK
void waitForCMDack()
{
  while(g_ps2mouse.callbackState != ack_cmd);
}

uint8_t convertToRaw(uint16_t ps2data)
{
  uint8_t tmpParity = 0;

  tmpParity = oddParityGen((uint8_t)((ps2data >> DATA_BIT0_POS) & 0x00FF));

  if(tmpParity != (uint8_t)((ps2data >> PARITY_BIT_POS) & 0x0001)) return 0;

  return (uint8_t)((ps2data >> DATA_BIT0_POS) & 0x00FF);
}

uint8_t oddParityGen(uint16_t data)
{
  //setting to 1 generates odd parity. 0 for even.
  uint8_t tempParity = 1;
  uint8_t index = 0;

  for(index = 0; index < sizeof(uint16_t)*8; index++)
  {
    tempParity ^= (data >> index) & 0x01;
  }

  return tempParity;
}

//convert data to packet
uint16_t dataToPacket(uint8_t data)
{
  uint8_t parity = 0;
  uint16_t tempConv = 0;

  //start value is 0, tempConv set to 0, no need to do again.
  tempConv |= ((uint16_t)data) << DATA_BIT0_POS;

  parity = oddParityGen(data);

  tempConv |= ((uint16_t)parity) << PARITY_BIT_POS;

  tempConv |= ((uint16_t)STOP_BIT_VALUE) << STOP_BIT_POS;

  return tempConv;
}

//copy to send buffer
void copyPacketToBuffer(uint16_t packet)
{
  g_ps2.buffer = packet;
}

//start transmit routine.
void startTransmit()
{
  g_ps2.dataState = send;

  g_ps2mouse.callbackState = waiting;

  g_ps2.recvCallback = &checkMouseResponse;

  //set clock pin low for at least 100ms
  *(g_ps2.p_port - 1) |= 1 << g_ps2.clkPin;

  *g_ps2.p_port &= ~(1 << g_ps2.clkPin);

  _delay_ms(110);

  //set data pin to output and set low
  *(g_ps2.p_port - 1) |= 1 << g_ps2.dataPin;

  *g_ps2.p_port &= ~(1 << g_ps2.dataPin);

  //release clock pin by setting to input
  *(g_ps2.p_port - 1) &= ~(1 << g_ps2.clkPin);

  g_ps2.index++;

  _delay_us(10);
}

void setID(uint16_t ps2Data)
{
  uint8_t convData = 0;

  convData = convertToRaw(ps2Data);

  g_ps2.recvCallback = &extractData;

  g_ps2mouse.id = convData;

  g_ps2mouse.callbackState = mouse_id;
}

void checkMouseResponse(uint16_t ps2Data)
{
  uint8_t convData = 0;

  convData = convertToRaw(ps2Data);

  g_ps2.recvCallback = &extractData;

  switch(convData)
  {
    case CMD_DEV_RDY:
      g_ps2.recvCallback = &setID;
      g_ps2mouse.callbackState = ready_cmd;
      break;
    case CMD_RESEND:
      g_ps2mouse.callbackState = resend_cmd;
      break;
    case CMD_ACK:
      if(g_ps2mouse.lastCMD == CMD_READ_ID)
      {
        g_ps2.recvCallback = &setID;
      }
      g_ps2mouse.callbackState = ack_cmd;
      break;
    default:
      g_ps2mouse.callbackState = no_cmd;
      break;
  }
}

void extractData(uint16_t ps2data)
{
  uint8_t rawPS2data = 0;

  rawPS2data = convertToRaw(ps2data);

  g_ps2.userRecvCallback(rawPS2data);
}
