/*******************************************************************************
 * @file    ps2mouseDefines.h
 * @author  Jay Convertino(electrobs@gmail.com)
 * @date    2024.03.16
 * @brief   defines for PS2
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


#ifndef _PS2_MOUSE_DEFINES
#define _PS2_MOUSE_DEFINES

//mouse commands
#define CMD_SET_REM_MODE  0xF0
#define CMD_WRAP_MODE     0xEE
#define CMD_RST_WRAP_MODE 0xEC
#define CMD_READ_DATA     0xEB
#define CMD_STREAM_MODE   0xEA
#define CMD_STATUS_REQ    0xE9
#define CMD_SET_RES       0xE8
#define CMD_SET_SCALE_21  0xE7
#define CMD_SET_SCALE_11  0xE6

#endif
