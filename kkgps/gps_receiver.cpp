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
#include "gps_receiver.h"
#include <iostream>
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//GPS_Receiver::
//---------------------------------------------------------------------------
GPS_Receiver::GPS_Receiver(
  u32 const                inChannelCount,
  ChannelSetup const&      inChannelSetup)
: sampleIndex_(0)
, userPosition_()
, channelSetup_(inChannelSetup)
, chs_(inChannelCount,inChannelSetup)
, inactivePrns_()
{
  inactivePrns_.push_back(9);
  inactivePrns_.push_back(21);
  inactivePrns_.push_back(22);
  inactivePrns_.push_back(23);
  inactivePrns_.push_back(26);
  inactivePrns_.push_back(29);
//  for(u32 i = 1; i <= 33; ++i)
//    inactivePrns_.push_back(i);
  
}
//---------------------------------------------------------------------------
GPS_Receiver::~GPS_Receiver()
{
}
//---------------------------------------------------------------------------
void
GPS_Receiver::processData(
  f32 const*               inBegin,
  f32 const* const         inEnd)
{
  //update position at once per second
  u32 const kPositionProcessDelta = 
    static_cast<u32>(channelSetup_.sampleRate_ / 1);
  
  //samplesPerWeek used to wrap sampleIndex_
  u64 const samplesPerWeek = 
    static_cast<u64>(kSecondsPerWeek * channelSetup_.sampleRate_);
     
  while(inBegin != inEnd)
  {
    //if at desired user time then we sample 

    if(0 == (sampleIndex_ % kPositionProcessDelta))
    {
      f64 const receiverTime = sampleIndex_ / channelSetup_.sampleRate_;
      
      //manage channels        
      for(u32 i = 0; i < chs_.size(); ++i)
        switch(chs_[i].signalTrack_.trackingState_)
        {
          case GPSCorrelatorTrack::tsIdle: 
            {
              u32 const oldPrn = chs_[i].signalTrack_.chips_.prnNumber_;
              if(oldPrn) inactivePrns_.push_back(oldPrn);
             
              chs_[i].reset(inactivePrns_.front());
              chs_[i].start();
              inactivePrns_.pop_front();
            }
            break;
          default:
            //store T_transmit(SV) at T_receive(User)
            chs_[i].lastSatTtr_ = chs_[i].signalTrack_.satTimeOfWeek();
            break;
        }
        
      std::cerr << "-----------------------------------------" << std::endl;
      
      //process position
      findPosition(userPosition_,chs_);
      
      //display GPS State
      std::cerr.precision(10); 
      std::cerr 
        << "ReceiverTime: " << receiverTime 
        << std::endl
        << " uX: " << userPosition_.x 
        << " uY: " << userPosition_.y 
        << " uZ: " << userPosition_.z 
        << " uT: " << userPosition_.t 
        << std::endl
        << " uLat: " 
        << " uLon: "  
        << std::endl;
        
      std::cerr 
        << "channel\t""svid\t""freq.\t""t.state\t""f.state\t""TOW(sv)" << std::endl;
      for(u32 i = 0; i < chs_.size(); ++i)
      {
        std::cerr.precision(5); 
        std::cerr 
          << i << "\t"
          << chs_[i].signalTrack_.chips_.prnNumber_ << "\t"
          << chs_[i].signalTrack_.dopplerDeltaFreq() << "\t"
          << chs_[i].signalTrack_.trackingState_ << "\t"
          << chs_[i].signalTrack_.subFrameTrackState_ << "\t";
        std::cerr.precision(10); 
        std::cerr 
          << chs_[i].signalTrack_.satTimeOfWeek() << std::endl;
      }

      for(u32 i = 0; i < chs_.size(); ++i)
      {
        if(chs_[i].ephemeris_.ephemerisValid())
        {
          std::cerr.precision(10); 
          FourDPos p = chs_[i].ephemeris_.findPos(chs_[i].lastSatTtr_);
          std::cerr 
            << chs_[i].signalTrack_.chips_.prnNumber_ 
            << "\tx: " << p.x << "\ty: " << p.y 
            << "\tz: " << p.z << "\tt: " << p.t
            << std::endl;
             
        }
      }
    }

    //only process data until next user sample time
    u32 const sampleCount = std::min(
      static_cast<u32>((kPositionProcessDelta - sampleIndex_ % kPositionProcessDelta)),
      static_cast<u32>(inEnd - inBegin));

    for(u32 i = 0; i < chs_.size(); ++i)
    {
      chs_[i].processData(inBegin,inBegin + sampleCount,0);
      //DisplayMsgState
      //DisplayCorrelatorState
      //DisplaySpreadingCodeHistogram
    }
    
    inBegin += sampleCount;    
    
    sampleIndex_ += sampleCount;
    if(sampleIndex_ > samplesPerWeek) sampleIndex_ -= samplesPerWeek;
  }
}
//---------------------------------------------------------------------------
