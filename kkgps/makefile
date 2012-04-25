#
# Copyright (C) 2003,2004 Krzysztof Kamieniecki (krys@kamieniecki.com)
#
# This file is part of kkGPS.
#
# kkGPS is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# kkGPS is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with kkGPS; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

CXX = g++

CXXFLAGS = -fmessage-length=0 -Wall -g -O2
#CXXFLAGS = -fmessage-length=0 -Wall -g 

##########################################
#SELECT correlator type
#uncomment next line for generic c++ correlator
CORROBJ = gps_corr_generic.o
#uncomment next line for SSE1 optimized correlator
#CORROBJ = gps_corr_sse1.o
#uncomment next line for G4 altivec optimized correlator
#CORROBJ = gps_corr_g4_altivec.o
##########################################


all: kkgps testunit testorbit #async_parse 

clean:
	rm -f main.o testunit.o testorbit.o gps_corr.o gps_msg.o gps_nav.o \
	gps_display.o gps_receiver.o gps_corr_generic.o gps_corr_sse1.o async_parse.o \
	gps_corr_g4_altivec.o

kkgps: main.o gps_corr.o gps_msg.o gps_nav.o gps_display.o gps_receiver.o $(CORROBJ)
	$(CXX) -o kkgps main.o gps_corr.o gps_msg.o gps_nav.o gps_display.o gps_receiver.o $(CORROBJ)

testunit: testunit.o gps_corr.o gps_msg.o gps_nav.o $(CORROBJ)
	$(CXX) -o testunit testunit.o gps_corr.o gps_msg.o gps_nav.o $(CORROBJ)

testorbit: testorbit.o gps_corr.o gps_msg.o gps_nav.o $(CORROBJ)
	$(CXX) -o testorbit testorbit.o gps_corr.o gps_msg.o gps_nav.o $(CORROBJ)

async_parse: async_parse.o
	$(CXX) -o async_parse async_parse.o


main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c main.cpp
	
testunit.o: testunit.cpp
	$(CXX) $(CXXFLAGS) -c testunit.cpp
	
testorbit.o: testorbit.cpp
	$(CXX) $(CXXFLAGS) -c testorbit.cpp

gps_corr.o: gps_corr.cpp
	$(CXX) $(CXXFLAGS) -c gps_corr.cpp
	
gps_corr_generic.o: gps_corr_generic.cpp
	$(CXX) $(CXXFLAGS) -c gps_corr_generic.cpp

gps_corr_sse1.o: gps_corr_sse1.cpp
	$(CXX) $(CXXFLAGS) -msse -c gps_corr_sse1.cpp
	
gps_corr_g4_altivec.o: gps_corr_g4_altivec.cpp
	$(CXX) $(CXXFLAGS) -altivec -c gps_corr_g4_altivec.cpp

gps_msg.o: gps_msg.cpp
	$(CXX) $(CXXFLAGS) -c gps_msg.cpp

gps_nav.o: gps_nav.cpp
	$(CXX) $(CXXFLAGS) -c gps_nav.cpp
	
gps_display.o: gps_display.cpp
	$(CXX) $(CXXFLAGS) -c gps_display.cpp

gps_receiver.o: gps_receiver.cpp
	$(CXX) $(CXXFLAGS) -c gps_receiver.cpp

async_parse.o: async_parse.cpp
	$(CXX) $(CXXFLAGS) -c async_parse.cpp

asm_gps_corr: gps_corr.cpp
	$(CXX) $(CXXFLAGS) -S -c gps_corr.cpp
