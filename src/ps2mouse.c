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

struct s_ps2 g_ps2;

struct s_ps2mouse
{
  volatile uint8_t id;
} g_ps2mouse;

//helper functions
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
  g_ps2.recvCallback = &extractData;

  g_ps2.responseCallback = &checkMouseResponse;
  g_ps2.callUserCallback = &extractData;

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
  sendCommand(&g_ps2, CMD_RESET);

  waitForCMDack(&g_ps2);

  waitForDevReady(&g_ps2);

  waitForDevID(&g_ps2);
}

void disablePS2mouse()
{
  sendCommand(&g_ps2, CMD_DISABLE);
}

void enablePS2mouse()
{
  sendCommand(&g_ps2, CMD_ENABLE);
}

void setPS2sampleRate(uint8_t rate)
{
  sendCommand(&g_ps2, CMD_SET_RATE);

  waitForCMDack(&g_ps2);

  sendData(&g_ps2, rate);

  waitForCMDack(&g_ps2);
}

void obtainPS2mouseID()
{
  sendCommand(&g_ps2, CMD_READ_ID);

  waitForDevID(&g_ps2);
}

uint8_t getPS2mouseID()
{
  return g_ps2mouse.id;
}

//callback functions
void setID(uint16_t ps2Data)
{
  uint8_t convData = 0;

  convData = convertToRaw(ps2Data);

  g_ps2.recvCallback = &extractData;

  g_ps2mouse.id = convData;

  g_ps2.callbackState = dev_id;
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
      g_ps2.callbackState = ready_cmd;
      break;
    case CMD_RESEND:
      g_ps2.callbackState = resend_cmd;
      break;
    case CMD_ACK:
      if(g_ps2.lastCMD == CMD_READ_ID)
      {
        g_ps2.recvCallback = &setID;
      }
      if(g_ps2.lastCMD == CMD_RESET)
      {
        g_ps2.recvCallback = &checkMouseResponse;
      }
      g_ps2.callbackState = ack_cmd;
      break;
    default:
      g_ps2.callbackState = no_cmd;
      break;
  }
}

void extractData(uint16_t ps2data)
{
  uint8_t rawPS2data = 0;

  rawPS2data = convertToRaw(ps2data);

  g_ps2.userRecvCallback(rawPS2data);
}
