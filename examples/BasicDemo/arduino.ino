/*
 * Copyright (c) 2019,2020 Gregory Tomasch and Simon D. Levy.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch, Simon D. Levy, and their successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifdef __MK20DX256__
#include <i2c_t3.h>
#elif defined(ARDUINO)
#include <Wire.h>
#endif

void serial_begin(void)
{
    Serial.begin(115200);
}

void serial_print(const char * s)
{
    Serial.print(s);
}

void serial_print(float x)
{
    Serial.print(x, 4);
}

void serial_print(uint8_t n)
{
    Serial.print(n);
}

void delay_msec(uint32_t msec)
{
    delay(msec);
}

uint32_t get_millis(void)
{
    return millis();
}

void setup_interrupt(uint8_t pin, void (*handler)(void))
{
    pinMode(pin, INPUT);
    attachInterrupt(pin, handler, RISING);
}

void i2c_begin(void)
{
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_100);
#else
    Wire.begin();
    delay_msec(100);
    Wire.setClock(100000); 
#endif

}

void i2c_set_clock(uint32_t speed)
{
    Wire.setClock(speed);
}
