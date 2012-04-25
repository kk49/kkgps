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
#include "gps_display.h"
#include <iostream>
#include <iomanip>
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
void 
DisplayMsgState(
  GPSCorrelatorMsgTrack::DoTrackResult const inResult,
  GPSCorrelatorMsgTrack const&               inData)
{
  if(GPSCorrelatorMsgTrack::trMsgWord <= inResult)
  {
    u32 msgData = 0;

    //display bit index 
    std::cout.setf(std::ios::dec,std::ios::basefield);
    std::cout.precision(12);
    std::cout << inData.spreadingCodeIndex_ / kSpreadingCodesPerBit << "\t"
              << inData.spreadingCodePhaseRemainder_ << "\t";
                 
    std::cout.setf(std::ios::hex,std::ios::basefield);

    //display spreading code history
    std::cout.width(5);
    std::cout.fill('0');
    std::cout << (0xFFFFF & inData.history_) << "\t";

    if(checkWordParity(inData.msgWord_,msgData))
      std::cout << 1;
    else
      std::cout << 0;
      
    if(isPossibleTLMData(msgData))
      std::cout << 1;
    else
      std::cout << 0;
 
    std::cout << "\t";
    std::cout.width(8);
    std::cout.fill('0');
    std::cout << inData.msgWord_ << "\t";
    std::cout.width(8);
    std::cout.fill('0');
    std::cout << msgData;
    
    std::cout  << std::endl;
    
    if(GPSCorrelatorMsgTrack::trMsgSubFrame == inResult)
    {
      s32 id;
      s32 page;
      SubFrame1 sf1;
      SubFrame2 sf2;
      SubFrame3 sf3;
      
      if(SubFrameIDGet(inData.subFrameRaw_,id,page))
      {
        std::cout << "(id,page): (" << id << "," << page << ")";
        switch(id)
        {
          case 1:                    
            sf1.parse(inData.subFrameRaw_);
            std::cout << std::endl << sf1.debugDump();
            break;
          case 2:
            sf2.parse(inData.subFrameRaw_);
            std::cout << std::endl << sf2.debugDump();
            break;
          case 3:
            sf3.parse(inData.subFrameRaw_);
            std::cout << std::endl << sf3.debugDump();
            break;
        }
      }
      else
      {
        std::cout << "Bad SubFrame" << std::endl;
      }
    }
    
    std::cout.flush();
  }
}
//---------------------------------------------------------------------------
void 
DisplayCorrelatorState(
  GPSCorrelatorMsgTrack::DoTrackResult const inResult,
  GPSCorrelatorMsgTrack const&               inData)
{
  if(GPSCorrelatorMsgTrack::trDump <= inResult)
  {
    f32 const ip = inData.dumpedSums_.ip_;
    f32 const qp = inData.dumpedSums_.qp_;
    f32 const it = inData.dumpedSums_.it_;
    f32 const qt = inData.dumpedSums_.qt_;
  
    std::cout
      << std::fixed << std::setprecision(1) << std::setw(8)
      << ip * 10000 << "\t" 
      << qp * 10000 << "\t" 
      << it * 10000 << "\t" 
      << qt * 10000
      << "\t" << inData.carrierFreqGet()
      << "\t" << inData.trackingState_ 
      << "\t" << inData.processedSamples_ 
      << "\t" << inData.chipPhase_
    ;
  
    std::cout << std::endl;
    std::cout.flush();
  }
}
//---------------------------------------------------------------------------
void 
DisplaySpreadingCodeHistogram(
  GPSCorrelatorMsgTrack::DoTrackResult const inResult,
  GPSCorrelatorMsgTrack const&               inData)
{
  if(GPSCorrelatorMsgTrack::trSpreadingCode <= inResult)
  {
    if(0 == (inData.spreadingCodeIndex_ % (30*20)))
    {
      std::cout << std::endl;
      for(u32 i = 0; i < inData.spreadingCodeHistogram_.size(); ++i)
      {
        if(i) std::cout << ' ';
        std::cout << inData.spreadingCodeHistogram_[i];
      }
      std::cout.flush();
    } 
  }            
}
//---------------------------------------------------------------------------
