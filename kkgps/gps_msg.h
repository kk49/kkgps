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
#ifndef gps_msgH
#define gps_msgH
//---------------------------------------------------------------------------
#include "gps_corr.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
u32 static const kSpreadingCodesPerBit = 20;
u32 static const kMsgBitsPerWord = 30;
u32 static const kMsgWordsPerSubFrame = 10;
u32 static const kMsgBitsPerSecond = 50;
u32 static const kSpreadingCodesPerSecond = kSpreadingCodesPerBit * kMsgBitsPerSecond;
u32 static const kSecondsPerWeek = 7 * 24 * 60 * 60; 
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
bool
isPossibleTLMWord(
  u32 const                  inWord);
//---------------------------------------------------------------------------
bool
isPossibleTLMData(
  u32 const                  inData);
//---------------------------------------------------------------------------
u32
parityEncodingEquations(
  u32 const                  inWord);
//---------------------------------------------------------------------------
bool
checkWordParity(
  u32 const                  inWord,
  u32&                       ouData);
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
struct
SubFrameRaw
{
  SubFrameRaw()
  {
    clear();
  }
  
  void
  clear()
  {
    std::memset(words_,0,sizeof(words_));
  }

/*
  bitOfWeek
  lastBitEndSampleNumber
  lastBitEndCodePhase
  lastBitEndCarrierPhase
*/
  u32                        words_[kMsgWordsPerSubFrame];
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
GPSCorrelatorMsgTrack
: public GPSCorrelatorTrack
{
public:
  GPSCorrelatorMsgTrack(
    f64 const                inSampleRate,
    f64 const                inCarrierFreq,
    f64 const                inChipRate,
    u32 const                inPrnNumber);

  void
  reset();

  enum 
  DoTrackResult
  { trNone
  , trDump
  , trSpreadingCode
  , trMsgBit
  , trMsgWord
  , trMsgSubFrame
  };

  typedef void (*DoTrackDumpCallback)(
    DoTrackResult const          inResult,
    GPSCorrelatorMsgTrack const& inData);

  DoTrackResult
  doTrack(
    f32 const*&               inBegin,
    f32 const* const          inEnd,
    DoTrackDumpCallback const inCallback = 0);
    
  f64
  satTimeOfWeek()
  const;
         

  u32                        spreadingCodeIndex_; //keeps track of number of spreadingCodes processed
  u32                        spreadingCodePhaseRemainder_; //keeps track of Spreading Code Phase Register at last dump

  //msg bit tracking i.e. every 20 spreadingCode cycles
  std::vector<u32>           spreadingCodeHistogram_; //array for tracking bit edges
  
  //word tracking
  u32                        msgWord_; //stores work bits of message word decoding

  //subframe tracking
  enum
  FrameTrackState
  { ftsNoMatch
  , ftsProbation 
  , ftsTracking 
  };

  FrameTrackState            subFrameTrackState_;
  u32                        subFrameEndBit_;
  SubFrameRaw                subFrameRaw_;
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
#endif // gps_msgH
