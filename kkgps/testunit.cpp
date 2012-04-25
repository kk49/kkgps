//---------------------------------------------------------------------------
//Copyright (C) 2003,2004 Krzysztof Kamieniecki (krys@kamieniecki.com)
/*
  This file is part of kkGPS.

  kkGPS is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
 
  kkGPS is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with kkGPS; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
#include <cmath>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cassert>
#include <ctime>
#include <fstream>

#include "gps_corr.h"
#include "gps_corr_generic.h"
#include "gps_msg.h"
#include "gps_nav.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
#define M_SIGN(V_) (V_ == 0)?(0):((V_ < 0)?(-1):(1))
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
void
testMatrixInvert()
{
  //A <row1> <row2> <row3> <row4>
  f64 A[16] = {1,1,0,0, 0,0,1,1, 0,1,1,0, 0,1,0,0};
  f64 const originalX[4] = {1,2,3,4};
  f64 b[4] = {0,0,0,0};
  
  for(u32 i = 0; i < 4; ++i)
  {
    b[0] += A[0*4 + i] * originalX[i];
    b[1] += A[1*4 + i] * originalX[i];
    b[2] += A[2*4 + i] * originalX[i];
    b[3] += A[3*4 + i] * originalX[i];
  }
  
  
  f64 iA[16];
  
  u32 const repCount = 1000000;
  clock_t const b1 = std::clock();
  for(u32 i = 0; i < repCount; ++i) Invert4x4Matrix(A,iA);
  clock_t const e1 = std::clock();

  std::cout << repCount << " 4x4 matrix inverts @ " 
    << 1.0 * repCount / (e1 - b1) * CLOCKS_PER_SEC << " reps / second\n";

  f64 x[4] = {0,0,0,0};
  for(u32 i = 0; i < 4; ++i)
  {
    x[0] += iA[0*4 + i] * b[i];
    x[1] += iA[1*4 + i] * b[i];
    x[2] += iA[2*4 + i] * b[i];
    x[3] += iA[3*4 + i] * b[i];
  }

  std::cout << "Ax = b Test\n";

  std::cout << "  A =";
  for(u32 i = 0; i < 16; ++i) std::cout << " " << A[i];
  std::cout << "\n";

  std::cout << "  b =";
  for(u32 i = 0; i < 4; ++i) std::cout << " " << b[i];
  std::cout << "\n";

  std::cout << "  iA    =";
  for(u32 i = 0; i < 16; ++i) std::cout << " " << iA[i];
  std::cout << "\n";

  std::cout << "  x =";
  for(u32 i = 0; i < 4; ++i) std::cout << " " << x[i];
  std::cout << "\n";
    
  std::cout << "  x - originalX =";
  for(u32 i = 0; i < 4; ++i) std::cout << " " << (x[i] - originalX[i]);
  std::cout << "\n";
}
//---------------------------------------------------------------------------
void
testLeastSqFit()
{
  f64 const k_c = 2.99792458e8;
  f64 dX[4] = {0,0,0,0};
  FourDPos pos(1e6,1e6,1e6,0);
  
  std::vector<f64> satX(6);
  std::vector<f64> satY(6);
  std::vector<f64> satZ(6);
  std::vector<f64> PRN(6);
  
  satX[0] = 20e6;  satY[0] = 0.0;   satZ[0] = 0.0;   
  satX[1] = 30e6;  satY[1] = 40e6;  satZ[1] = 0.0;   
  satX[2] = 0.0;   satY[2] = 0.0;   satZ[2] = 20e6;  
  satX[3] = -20e6; satY[3] = 0.0;   satZ[3] = 0.0;   
  satX[4] = 0.0;   satY[4] = -30e6; satZ[4] = 40e6;  
  satX[5] = 0.0;   satY[5] = 0.0;   satZ[5] = -20e6; 
  
  for(u32 i = 0; i < PRN.size(); ++i)
    PRN[i] = std::sqrt(satX[i] * satX[i] + satY[i] * satY[i] + satZ[i] * satZ[i]) / k_c;

  for(u32 j = 0; j < 10; ++j)
  {
    LeastSqFit(dX,pos,satX,satY,satZ,PRN);
  
    	pos.x += dX[0];
    	pos.y += dX[1];
    	pos.z += dX[2];
    	pos.t -= dX[3] / k_c;

    	for(u32 i = 0; i < PRN.size(); ++i)
	      PRN[i] -= dX[3] / k_c;

    f64 const posErr = 
      std::sqrt(dX[0] * dX[0] + dX[1] * dX[1] + dX[2] * dX[2] + dX[3]*dX[3]);

    std::cout << "Least Sq.: del "
      << dX[0] << " " << dX[1] << " " 
    	  << dX[2] << " " << dX[3] << " " << posErr << "\n";
    std::cout << "Least Sq.: pos "
      << pos.x << " " << pos.y << " " 
    	  << pos.z << " " << pos.t << "\n";
	         
    if(posErr < 0.001)  break;
  }
}
//---------------------------------------------------------------------------
void
testMsgDecodeFuncs()
{
  std::cerr << "s32 0xFF0000, 1, 8 = " << parseMsgDataS32(0xFF0000, 1, 8) << std::endl;
  std::cerr << "s32 0x00FF00, 9,16 = " << parseMsgDataS32(0x00FF00, 9,16) << std::endl;
  std::cerr << "s32 0x0000FF,17,24 = " << parseMsgDataS32(0x0000FF,17,24) << std::endl;
  std::cerr << "u32 0x0000FF,17,24 = " << parseMsgDataU32(0x0000FF,17,24) << std::endl;
  std::cerr << "s32 0xFF0000, 2, 8 = " << parseMsgDataS32(0xFF0000, 2, 8) << std::endl;
  std::cerr << "s32 0x00FF00, 7,16 = " << parseMsgDataS32(0x00FF00, 7,16) << std::endl;
  std::cerr << "s32 0x0000FF,24,24 = " << parseMsgDataS32(0x0000FF,24,24) << std::endl;
  std::cerr << "s32 0x0000FF,16,17 = " << parseMsgDataS32(0x0000FF,16,17) << std::endl;
  std::cerr << "u32 0x0000FF,17,24,0xFFFFFF,1,24 = " << parseMsgDataU32(0x0000FF,17,24,0xFFFFFF,1,24) << std::endl;
  std::cerr << "s32 0x0000FF,17,24,0xFFFFFF,1,24 = " << parseMsgDataS32(0x0000FF,17,24,0xFFFFFF,1,24) << std::endl;
  std::cerr << "f64(-10) s32 0x0000FF,16,17 = " << parseMsgDataSF64(-10,0x0000FF,16,17) << std::endl;
  std::cerr << "f64(-10) s32 0xFFFFFF,1,24 = " << parseMsgDataSF64(-10,0xFFFFFF,1,24) << std::endl;
  std::cerr << "f64(-10) u32 0xFFFFFF,1,24 = " << parseMsgDataUF64(-10,0xFFFFFF,1,24) << std::endl;
  std::cerr << "f64(-31) u32 0x0000FF,17,24,0xFFFFFF,1,24 = " << parseMsgDataUF64(-31,0x0000FF,17,24,0xFFFFFF,1,24) << std::endl;
  std::cerr << "f64(-31) s32 0x0000FF,17,24,0xFFFFFF,1,24 = " << parseMsgDataSF64(-31,0x0000FF,17,24,0xFFFFFF,1,24) << std::endl;
}
//---------------------------------------------------------------------------
//makes sure that the PRN code genrator works
void
testGenerateCACode()
{
  c8 ca[10];
  generateCACode(1,1,0,ca,ca+10);
  u32 v = 0;
  for(u32 i = 0; i < 10; ++i)
  {
    v <<= 1;
    v |= ca[i];
  }
  assert(v == 01440);
}
//---------------------------------------------------------------------------
//runs a timing test of the correlator does ~1 sec worth of correlation and
//reports the <actual process time in seconds> <iteration count>
void
timeProcessGPSCorrelator(
  u32 const Fs_u32)
{
  f64 const Fs = Fs_u32;
  u32 const chipFs_u32 = 2046000;
  f64 const chipFs = chipFs_u32;

  u32 const inputLength = static_cast<u32>(Fs_u32 * 1.1);
  std::vector<f32> inputData(inputLength);

  const f32* dataBegin = &*inputData.begin();
  const f32* const dataEnd = &*inputData.end();
  GPSCorrelatorChips prn1(1);
  GPSCorrelatorTrack gpscs(Fs,1.25e6,chipFs,1);
  gpscs.chipBase_ = prn1.promptEarly_;

  u32 phase = 0;//(carrierLength / 8) << 16;
  u32 chipPhase = 0;
  for(u32 i = 0; i < inputLength; ++i)
  {
    inputData[i] = 0;
    inputData[i] +=
      *cosData.cosine(phase) *
      prn1.promptEarly_[((chipPhase >> 16) + 0) % 2046 + 2];
    phase += gpscs.carrierDelta_;
    chipPhase += gpscs.chipDelta_;
    chipPhase %= 2046 << 16;
  }

  gpscs.carrierFreqSet(5e6);
  gpscs.carrierPhase_ = 0;


  u32 cnt = 0;

  clock_t const b = std::clock();
  while(dataBegin != dataEnd)
  {
    processGPSCorrelator(dataBegin,dataEnd,&gpscs);
    ++cnt;
  }
  clock_t const e = std::clock();

  f64 const d3 = 1.0 * (e - b) / CLOCKS_PER_SEC;

  std::cerr << "corr time test Fs = " << Fs_u32 << " : " << d3 << " " << cnt << std::endl;
}
//---------------------------------------------------------------------------
void
testParityCheck(
  c8 const* const inFileName)
{
  c8 CAMessage[1500];

  std::fstream f(inFileName,std::ios::in | std::ios::binary);
  if(!f.is_open()) throw KKException("Could Not Open C/A Message File");
  f.read(CAMessage,sizeof(CAMessage));
  if(!f.good()) throw KKException("Could Not Read C/A Message File");   
  for(u32 i = 0; i < sizeof(CAMessage); ++i)
    CAMessage[i] = ('1' == CAMessage[i])?1:-1;
    
  u32 msg = 0;
  std::cout << inFileName << " Parity Check: ";
  for(u32 i = 0; i < 1500; ++i)
  {
    msg <<= 1;
    	msg |= (CAMessage[i] > 0)?1:0;
    if(0 ==((i+1) % 30))
    {
      u32 data;
      std::cout << ((checkWordParity(msg,data))?1:0); 
    }
  }
  std::cout << std::endl;
}
//---------------------------------------------------------------------------
//load C/A message from file gotten from OpenGPSSim 
void
timeFullGPSTrack()
{  c8 CAMessage[1500];

  {
    std::fstream f("273673_22.txt",std::ios::in | std::ios::binary);
    if(!f.is_open()) throw KKException("Could Not Open C/A Message File");
    f.read(CAMessage,sizeof(CAMessage));
    if(!f.good()) throw KKException("Could Not Read C/A Message File");   
    for(u32 i = 0; i < sizeof(CAMessage); ++i)
      CAMessage[i] = ('1' == CAMessage[i])?1:-1;
  }
  
  
//GPS signal Simulation + processing
  u32 const Fs_u32 = 5000000;    //Sample Frequency
  f64 const Fs = Fs_u32;          //Sample Frequency Again
  u32 const chipFs_u32 = 2046000; //1/2 Chip Frequency
  f64 const chipFs = chipFs_u32;  //1/2 Chip Frequency Again

  f64 sourceFreq = 1.25e6 + 45;      //simulator carrier frequency

  
  std::vector<f32> inputData(Fs_u32); //simulator data buffer
  GPSCorrelatorChips prn22(22);          //simulator chip generator
  GPSCorrelatorState source(Fs,sourceFreq,chipFs); //used for simulator NCOs

  u32 phase = cosData.negSinOffset / 2; //simulator carrier phase offset
  u32 chipPhase = 2 << 15;              //simulator chip phase offset
  u64 messageCodeCount = 0;       //simulator spreading code counter

  GPSCorrelatorMsgTrack gpscs(Fs,1.25e6,chipFs,22); //tracking correlator channel

  //generate block of data
  u32 const inputLength = inputData.size();
  for(u32 i = 0; i < inputLength; ++i)
  {
    inputData[i] = 0;

    //carrier and spreading code
    inputData[i] +=
      *(cosData.cosine(phase)) *
      prn22.promptEarly_[((chipPhase >> 16) + 0) % 2046 + 2];

    //message bit
    inputData[i] *= CAMessage[(messageCodeCount / 20) % 1500];

    //uniform distribution noise
    inputData[i] += 2.0 * std::rand() / RAND_MAX - 1.0;

    //eventual gaussian distribution noise
//      inputData[i] += RandG(100.0,10);

    phase += source.carrierDelta_;
    chipPhase += source.chipDelta_;

    //handle updating message bit
    if(chipPhase >= (2046 << 16))
    {       
      ++messageCodeCount;
      messageCodeCount %= (1500 * 20);
    }
    chipPhase %= 2046 << 16;
  }

  const f32* dataBegin = &*inputData.begin();
  const f32* const dataEnd = &*inputData.end();

  gpscs.start();
  
  clock_t const b = std::clock();

  //process block of data
  while(dataBegin != dataEnd)
    gpscs.doTrack(dataBegin,dataEnd);

  clock_t const e = std::clock();

  f64 const d3 = (Fs / inputData.size()) * (e - b) / CLOCKS_PER_SEC;

  std::cerr << "full time test: " << d3 << std::endl;
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  try
  { 
    testMatrixInvert();
    
    testLeastSqFit();
 
    testMsgDecodeFuncs();

    testGenerateCACode();

    testParityCheck("273673_09.txt");
    testParityCheck("273673_21.txt");
    testParityCheck("273673_22.txt");
    testParityCheck("273673_23.txt");
    testParityCheck("273673_26.txt");
    testParityCheck("273673_29.txt");

    timeProcessGPSCorrelator(5000000);

    timeProcessGPSCorrelator(20000000);

    timeFullGPSTrack();

  }
  catch(std::exception const& E)
  {
    std::cerr << "\nERROR: " << E.what() << std::endl;
      return 1;
  }
  return 0;
}
//---------------------------------------------------------------------------
