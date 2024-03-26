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

struct s_ps2mouse
{
  volatile uint8_t id;
};

//helper functions
//CALLBACK ROUTINES
//set the id of the mouse if last command was get id and ready command received.
void setID(void *p_data, uint16_t ps2Data);
//check the response to mouse command sent and perform needed operations
void checkMouseResponse(void *p_data, uint16_t ps2Data);
//Default callback for recv that processes data and then hands it off to the user callback.
void extractData(void *p_data, uint16_t ps2data);

void initPS2mouse(struct s_ps2 *p_ps2mouse, t_PS2userRecvCallback PS2recvCallback, void (*setPS2_PORT_Device)(struct s_ps2 *p_device), volatile uint8_t *p_port, uint8_t clkPin, uint8_t dataPin)
{
  uint8_t tmpSREG = 0;

  tmpSREG = SREG;
  cli();

  if(p_ps2mouse == NULL) return;

  if(p_port == NULL) return;

  if(PS2recvCallback == NULL) return;

  if(setPS2_PORT_Device == NULL) return;

  memset(p_ps2mouse, 0, sizeof(struct s_ps2));

  p_ps2mouse->p_device = malloc(sizeof(struct s_ps2mouse));

  ((struct s_ps2mouse *)(p_ps2mouse->p_device))->id = 0;

  p_ps2mouse->clkPin = clkPin;
  p_ps2mouse->dataPin = dataPin;
  p_ps2mouse->p_port = p_port;
  p_ps2mouse->lastAckState = ack;
  p_ps2mouse->dataState = idle;
  p_ps2mouse->userRecvCallback = PS2recvCallback;
  p_ps2mouse->recvCallback = NULL;

  p_ps2mouse->responseCallback = &checkMouseResponse;
  p_ps2mouse->callUserCallback = &extractData;

  setPS2_PORT_Device(p_ps2mouse);

  *(p_ps2mouse->p_port - 1) &= ~(1 << p_ps2mouse->clkPin);
  *(p_ps2mouse->p_port - 1) &= ~(1 << p_ps2mouse->dataPin);

  if(p_ps2mouse->p_port == &PORTB)
  {
    PCICR |= 1 << PCIE0;
    PCMSK0 |= 1 << p_ps2mouse->clkPin;
  }
  else if(p_ps2mouse->p_port == &PORTC)
  {
    PCICR |= 1 << PCIE1;
    PCMSK1 |= 1 << p_ps2mouse->clkPin;
  }
  else
  {
    PCICR |= 1 << PCIE2;
    PCMSK2 |= 1 << p_ps2mouse->clkPin;
  }

  SREG = tmpSREG;

  sei();

  //initialize mouse using PC init method
  resetPS2mouse(p_ps2mouse);

  enablePS2mouse(p_ps2mouse);

  obtainPS2mouseID(p_ps2mouse);
}

void resetPS2mouse(struct s_ps2 *p_ps2mouse)
{
  if(p_ps2mouse == NULL) return;

  sendCommand(p_ps2mouse, CMD_RESET);

  waitForCMDack(p_ps2mouse);

  waitForDevReady(p_ps2mouse);

  waitForDevID(p_ps2mouse);
}

void disablePS2mouse(struct s_ps2 *p_ps2mouse)
{
  if(p_ps2mouse == NULL) return;

  sendCommand(p_ps2mouse, CMD_DISABLE);
}

void enablePS2mouse(struct s_ps2 *p_ps2mouse)
{
  if(p_ps2mouse == NULL) return;

  sendCommand(p_ps2mouse, CMD_ENABLE);
}

void setPS2sampleRate(struct s_ps2 *p_ps2mouse, uint8_t rate)
{
  if(p_ps2mouse == NULL) return;

  sendCommand(p_ps2mouse, CMD_SET_RATE);

  waitForCMDack(p_ps2mouse);

  sendData(p_ps2mouse, rate);

  waitForCMDack(p_ps2mouse);
}

void obtainPS2mouseID(struct s_ps2 *p_ps2mouse)
{
  if(p_ps2mouse == NULL) return;

  sendCommand(p_ps2mouse, CMD_READ_ID);

  waitForDevID(p_ps2mouse);
}

uint8_t getPS2mouseID(struct s_ps2 *p_ps2mouse)
{
  if(p_ps2mouse == NULL) return 0;

  return ((struct s_ps2mouse *)(p_ps2mouse->p_device))->id;
}

//callback functions
void setID(void *p_data, uint16_t ps2Data)
{
  uint8_t convData = 0;

  struct s_ps2 *p_ps2 = NULL;

  if(p_data == NULL) return;

  p_ps2 = (struct s_ps2 *)p_data;

  convData = convertToRaw(ps2Data);

  p_ps2->recvCallback = &extractData;

  ((struct s_ps2mouse *)(p_ps2->p_device))->id = convData;

  p_ps2->callbackState = dev_id;
}

void checkMouseResponse(void *p_data, uint16_t ps2Data)
{
  uint8_t convData = 0;

  struct s_ps2 *p_ps2 = NULL;

  if(p_data == NULL) return;

  p_ps2 = (struct s_ps2 *)p_data;

  convData = convertToRaw(ps2Data);

  p_ps2->recvCallback = &extractData;

  switch(convData)
  {
    case CMD_DEV_RDY:
      p_ps2->recvCallback = &setID;
      p_ps2->callbackState = ready_cmd;
      break;
    case CMD_RESEND:
      p_ps2->callbackState = resend_cmd;
      break;
    case CMD_ACK:
      if(p_ps2->lastCMD == CMD_READ_ID)
      {
        p_ps2->recvCallback = &setID;
      }
      if(p_ps2->lastCMD == CMD_RESET)
      {
        p_ps2->recvCallback = &checkMouseResponse;
      }
      p_ps2->callbackState = ack_cmd;
      break;
    default:
      p_ps2->callbackState = no_cmd;
      break;
  }
}

void extractData(void *p_data, uint16_t ps2data)
{
  uint8_t rawPS2data = 0;

  struct s_ps2 *p_ps2 = NULL;

  if(p_data == NULL) return;

  p_ps2 = (struct s_ps2 *)p_data;

  rawPS2data = convertToRaw(ps2data);

  p_ps2->userRecvCallback(rawPS2data);
}
