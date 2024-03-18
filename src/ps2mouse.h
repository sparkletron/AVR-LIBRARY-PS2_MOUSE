/*******************************************************************************
 * @file    ps2mouse.h
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

#ifndef _PS2_MOUSE
#define _PS2_MOUSE

#include <inttypes.h>
#include "ps2base.h"
#include "ps2mouseDefines.h"

/**
 * \brief initialize PS2 keyboard
 *
 * \param PS2recvCallback Callback to a user supplied function to parse mouse data.
 * \param setPS2_PORT_device A function pointer to a IRQ port data setter.
 * \param p_port Gets the address of a port to be used for the clk and data pin.
 * \param clkPin Define which pin used for the clock
 * \param dataPin Define the pin used for data.
 */
void initPS2mouse(t_PS2userRecvCallback PS2recvCallback, void (*setPS2_PORT_Device)(struct s_ps2 *p_device), volatile uint8_t *p_port, uint8_t clkPin, uint8_t dataPin);

/**
 * \brief Reset PS2 mouse.
 */
void resetPS2mouse();

/**
 * \brief Disable PS2 mouse.
 */
void disablePS2mouse();

/**
 * \brief Enable PS2 mouse.
 */
void enablePS2mouse();

/**
 * \brief set PS2 mouse sample rate.
 */
void setPS2sampleRate(uint8_t rate);

/**
 * \brief obtain PS2 mouse id to internal struct.
 */
void obtainPS2mouseID();

/**
 * \brief get PS2 mouse id from internal struct.
 */
uint8_t getPS2mouseID();

#endif
