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
#include <iostream>
#include <fstream>
#include "gps_receiver.h"

//---------------------------------------------------------------------------
std::string
printBits(
  u32 const&                 in_value)
{
  std::string result;
  
  for(s32 i = 31; i >= 0; --i)
    if(1 & (in_value >> i)) 
      result += '1';
    else
      result += '0';
  
  return result;
}
//---------------------------------------------------------------------------
void
loadCAMessage(
  c8 const* const            inFile,
  c8* const                  ioBuf,
  u32 const                  inSize)
{
  std::fstream f(inFile,std::ios::in | std::ios::binary);
  if(!f.is_open()) throw KKException("Could Not Open C/A Message File");
  f.read(ioBuf,inSize);
  if(!f.good()) throw KKException("Could Not Read C/A Message File");   
  for(u32 i = 0; i < inSize; ++i)
    ioBuf[i] = ('1' == ioBuf[i])?1:-1;
}
//---------------------------------------------------------------------------
//does tracking. continuosly reports
// <prompt I> <prompt Q> <track I> <track Q> <current carrier freq> <state>
void
testGPSTrack()
{
//load C/A message from file gotten from Async downalod from gps reciever
  c8 CAMessage09[1500];
  c8 CAMessage21[1500];
  c8 CAMessage22[1500];
  c8 CAMessage23[1500];
  c8 CAMessage26[1500];
  c8 CAMessage29[1500];
  loadCAMessage("273673_09.txt",CAMessage09,sizeof(CAMessage09));
  loadCAMessage("273673_21.txt",CAMessage21,sizeof(CAMessage21));
  loadCAMessage("273673_22.txt",CAMessage22,sizeof(CAMessage22));
  loadCAMessage("273673_23.txt",CAMessage23,sizeof(CAMessage23));
  loadCAMessage("273673_26.txt",CAMessage26,sizeof(CAMessage26));
  loadCAMessage("273673_29.txt",CAMessage29,sizeof(CAMessage29));

//GPS signal Simulation + processing
  u32 const Fs_u32 = 5000000;    //Sample Frequency
  f64 const Fs = Fs_u32;          //Sample Frequency Again
  u32 const chipFs_u32 = 2046000; //1/2 Chip Frequency
  f64 const chipFs = chipFs_u32;  //1/2 Chip Frequency Again

  f64 sourceFreq = 1.25e6 + 45;      //simulator carrier frequency

  std::vector<f32> inputData(16*1024); //simulator data buffer
  GPSCorrelatorChips prn09(9);          //simulator chip generator
  GPSCorrelatorChips prn21(21);          //simulator chip generator
  GPSCorrelatorChips prn22(22);          //simulator chip generator
  GPSCorrelatorChips prn23(23);          //simulator chip generator
  GPSCorrelatorChips prn26(26);          //simulator chip generator
  GPSCorrelatorChips prn29(29);          //simulator chip generator
  GPSCorrelatorState source(Fs,sourceFreq,chipFs); //used for simulator NCOs

  u32 phase = cosData.negSinOffset / 2; //simulator carrier phase offset
  u32 chipPhase = 2 << 15;              //simulator chip phase offset
  u32 messageCodeCount = 0;	      			//simulator spreading code counter
  u32 messageBitCount = 0;                 //simulator message bit count

  u32 const receiverChannels = 6;
  GPS_Receiver gpsRx(receiverChannels,ChannelSetup(Fs,1.25e6,chipFs,0));

  while(true)
  {
    //generate block of data
    u32 const inputLength = inputData.size();
    for(u32 i = 0; i < inputLength; ++i)
    {
      inputData[i] = 0;

      //carrier and spreading code
      f64 data09 =
        *(cosData.cosine(phase)) * 
        prn09.promptEarly_[((chipPhase >> 16) + 0) % 2046 + 2];
      //message bit
      data09 *= CAMessage09[messageBitCount];
      inputData[i] += data09;

      //carrier and spreading code
      f64 data21 =
        *(cosData.cosine(phase)) *
        prn21.promptEarly_[((chipPhase >> 16) + 0) % 2046 + 2];
      //message bit
      data21 *= CAMessage21[messageBitCount];
      inputData[i] += data21;

      //carrier and spreading code
      f64 data22 =
        *(cosData.cosine(phase)) *
        prn22.promptEarly_[((chipPhase >> 16) + 0) % 2046 + 2];
      //message bit
      data22 *= CAMessage22[messageBitCount];
      inputData[i] += data22;

      //carrier and spreading code
      f64 data23 =
        *(cosData.cosine(phase)) *
        prn23.promptEarly_[((chipPhase >> 16) + 0) % 2046 + 2];
      //message bit
      data23 *= CAMessage23[messageBitCount];
      inputData[i] += data23;
        
      //carrier and spreading code
      f64 data26 =
        *(cosData.cosine(phase)) *
        prn26.promptEarly_[((chipPhase >> 16) + 0) % 2046 + 2];
      //message bit
      data26 *= CAMessage26[messageBitCount];
      inputData[i] += data26;

      //carrier and spreading code
      f64 data29 =
        *(cosData.cosine(phase)) *
        prn29.promptEarly_[((chipPhase >> 16) + 0) % 2046 + 2];
      //message bit
      data29 *= CAMessage29[messageBitCount];
      inputData[i] += data29;

      //uniform distribution noise
//      inputData[i] += 2.0 * std::rand() / RAND_MAX - 1.0;

      //eventual gaussian distribution noise
//      inputData[i] += RandG(100.0,10);

      phase += source.carrierDelta_;
      chipPhase += source.chipDelta_;

      //handle updating message bit
      if(chipPhase >= (2046 << 16))
      {      	
        ++messageCodeCount;
        if(messageCodeCount >= 20)
        {
          messageCodeCount = 0;
          ++messageBitCount;
          if(messageBitCount >= 1500)
            messageBitCount = 0;
        }
      }
      chipPhase %= 2046 << 16;
    }

    f32 const* dataBegin = &*inputData.begin();
    f32 const* const dataEnd = &*inputData.end();

    gpsRx.processData(dataBegin,dataEnd);
  }
}
//---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  try
  {	
    testGPSTrack();
  }
  catch(std::exception const& E)
  {
    	std::cerr << "\nERROR: " << E.what() << std::endl;
    	return 1;
  }
  return 0;
}
//---------------------------------------------------------------------------
