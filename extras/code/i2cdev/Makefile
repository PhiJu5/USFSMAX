# 
# Makefile for USFSMAXMAX i2cdev examples
#
# Copyright (C) 2020 Simon D. Levy, Greg Tomasch. All rights reserved.
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

# Change this to whereever you installed CrossPlatformDataBus
CPDB = $(HOME)/Documents/Arduino/libraries/CrossPlatformDataBus
  
TARG = BasicDemo

USRC = ../../../src

all: $(TARG)

I2CINC = $(CPDB)/src
I2CSRC = $(CPDB)/extras/i2c/i2cdev/src

BasicDemo: BasicDemo.o USFSMAX.o USFSMAX_Basic.o I2CDevI2C.o main.o serial.o
	g++ -std=c++11 USFSMAX.o USFSMAX_Basic.o I2CDevI2C.o main.o 

BasicDemo.o: 
	g++ -x c++ -std=c++11 -Wall -I$(USRC) -c ../../../examples/BasicDemo/BasicDemo.ino

serial.o: serial.cpp 
	g++ -std=c++11 -Wall -c serial.cpp

USFSMAX.o: $(USRC)/USFSMAX.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(USRC) -c $(USRC)/USFSMAX.cpp

USFSMAX_Basic.o: $(USRC)/USFSMAX_Basic.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -I$(USRC) -c $(USRC)/USFSMAX_Basic.cpp

I2CDevI2C.o: $(I2CSRC)/I2CDevI2C.cpp 
	g++ -std=c++11 -Wall -I$(I2CINC) -c $(I2CSRC)/I2CDevI2C.cpp

main.o: main.cpp 
	g++ -std=c++11 -Wall -c main.cpp

run: $(TARG)
	./$(TARG)

clean:
	rm -f $(TARG) *.o *~
