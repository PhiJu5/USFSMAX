# 
# Makefile for USFSMAX WiringPi examples
#
# Copyright (c) 2020 Simon D. Levy
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal with the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimers.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimers in the
#     documentation and/or other materials provided with the distribution.
#  3. The names of Gregory Tomasch, Simon D. Levy, and their successors
#     may not be used to endorse or promote products derived from this Software
#     without specific prior written permission.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# WITH THE SOFTWARE.

# Change this to whereever you installed CrossPlatformI2C
CPDB = $(HOME)/Desktop/CrossPlatformDataBus

ALL = main

CPSRC  = $(CPDB)/src
CPEXT  = $(CPDB)/extras
CPCMN  = $(CPEXT)/common
I2CSRC = $(CPEXT)/i2c/wiringpi/src

USRC   = $(HOME)/Desktop/USFSMAX/src

all: $(ALL)

main: main.o WiringPiI2C.o main.o
	g++ -std=c++11 -o main main.o WiringPiI2C.o -lwiringPi

main.o: main.cpp 
	g++ -std=c++11 -Wall -I$(CPSRC) -I$(USRC) -c main.cpp

WiringPiI2C.o: $(I2CSRC)/WiringPiI2C.cpp 
	g++ -std=c++11 -Wall -I$(CPSRC) -c $(I2CSRC)/WiringPiI2C.cpp

run: min
	./main

edit:
	vim main.cpp

clean:
	rm -f $(ALL) *.o *~
