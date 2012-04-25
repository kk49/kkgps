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
#include "gps_msg.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
bool
isPossibleTLMWord(
  u32 const                  inWord)
{
  u32 static const kTLMPreamble     = 0x8B << 22;
  u32 static const kTLMPreambleMask = 0xFF << 22;
  return (kTLMPreamble == (kTLMPreambleMask & inWord));
}
//---------------------------------------------------------------------------
bool
isPossibleTLMData(
  u32 const                  inData)
{
  u32 static const kTLMPreamble     = 0x8B << 16;
  u32 static const kTLMPreambleMask = 0xFF << 16;
  return (kTLMPreamble == (kTLMPreambleMask & inData));
}
//---------------------------------------------------------------------------
//see Table 20-XIV "Parity Encoding Equation" in "ICD-GPS-200C 10 OCT 1993" spec
//input is u32 with bits MSB -> LSB (D29* D30*  d1 d2 d3 .... d24  ? ? ? ? ? ?)
//D29* and D30* are bottom two parity bits from previous word
//output is u32 with bits MSB -> LSB (D29* D30*  D1 D2 D3 .... D24  D25 D26 D27 D28 D29 D30)
u32
parityEncodingEquations(
  u32 const                  inWord)
{
#define B(I_) ((inWord >> ((24 - I_) + 6)) & 1)
#define B29S ((inWord >> 31) & 1)
#define B30S ((inWord >> 30) & 1)

  u32 result = inWord & (0xFFFFFF << 6); //result = 0 0 d1...d24 0 0 0 0 0 0

  if(B30S) //if D30* == 1
    result ^= (0xFFFFFF << 6); //xor d1..d24 in result


  //D25 compute
  result |= (B29S ^ B(1) ^ B(2) ^ B(3) ^ B(5) ^ B(6) ^ B(10) ^ B(11) ^ B(12)
                   ^ B(13) ^ B(14) ^ B(17) ^ B(18) ^ B(20) ^ B(23))  << 5;
  //D26 compute
  result |= (B30S ^ B(2) ^ B(3) ^ B(4) ^ B(6) ^ B(7) ^ B(11) ^ B(12) ^ B(13)
                   ^ B(14) ^ B(15) ^ B(18) ^ B(19) ^ B(21) ^ B(24))  << 4;
  //D27 compute
  result |= (B29S ^ B(1) ^ B(3) ^ B(4) ^ B(5) ^ B(7) ^ B(8) ^ B(12) ^ B(13)
                   ^ B(14) ^ B(15) ^ B(16) ^ B(19) ^ B(20) ^ B(22))  << 3;
  //D28 compute
  result |= (B30S ^ B(2) ^ B(4) ^ B(5) ^ B(6) ^ B(8) ^ B(9) ^ B(13) ^ B(14)
                   ^ B(15) ^ B(16) ^ B(17) ^ B(20) ^ B(21) ^ B(23))  << 2;
  //D29 compute
  result |= (B30S ^ B(1) ^ B(3) ^ B(5) ^ B(6) ^ B(7) ^ B(9) ^ B(10) ^ B(14)
                   ^ B(15) ^ B(16) ^ B(17) ^ B(18) ^ B(21) ^ B(22) ^ B(24))  << 1;
  //D30 compute
  result |= (B29S ^ B(3) ^ B(5) ^ B(6) ^ B(8) ^ B(9) ^ B(10) ^ B(11) ^ B(13)
                   ^ B(15) ^ B(19) ^ B(22) ^ B(23) ^ B(24))  << 0;

  return result | ((3 << 30) & inWord); //put D29* and D30* back in result

#undef B30S
#undef B29S
#undef B
}
//---------------------------------------------------------------------------
bool
checkWordParity(
  u32 const                  inWord,
  u32&                       ouData)
{
  u32 result = inWord;               //setup D29* D30* D1...D24 for parity check
  ouData = (inWord >> 6) & 0xFFFFFF; //setup D1...D24 for decodeing by caller

  if((inWord >> 30) & 1) //if D30* == 1
  {
    result ^= (0xFFFFFF << 6); //xor D1...D24 in result d1...d24
    ouData ^= 0xFFFFFF;        //xor D1...D24 in result d1...d24
  }

  result = parityEncodingEquations(result);

  return inWord == result;
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//GPSCorrelatorMsgTrack::
//---------------------------------------------------------------------------
GPSCorrelatorMsgTrack::GPSCorrelatorMsgTrack(
  f64 const                inSampleRate,
  f64 const                inCarrierFreq,
  f64 const                inChipRate,
  u32 const                inPrnNumber)
: GPSCorrelatorTrack(inSampleRate,inCarrierFreq,inChipRate,inPrnNumber)
{
  reset();
}
//---------------------------------------------------------------------------
void
GPSCorrelatorMsgTrack::reset()
{
  GPSCorrelatorTrack::reset();

  spreadingCodeIndex_ = 0;
  spreadingCodeHistogram_.clear();
  spreadingCodeHistogram_.resize(kSpreadingCodesPerBit);
  msgWord_ = 0;
  
  subFrameTrackState_ = ftsNoMatch;
  subFrameEndBit_ = 0;
  subFrameRaw_.clear();
}
//---------------------------------------------------------------------------
GPSCorrelatorMsgTrack::DoTrackResult
GPSCorrelatorMsgTrack::doTrack(
  f32 const*&               inBegin,
  f32 const* const          inEnd,
  DoTrackDumpCallback const inCallback)
{
  DoTrackResult result = trNone;
  
  //process block of data
  bool const dump = GPSCorrelatorTrack::doTrack(inBegin,inEnd);

  if(dump) result = trDump;

  if(dump && (trackingState_ == GPSCorrelatorTrack::tsTracking))
  {
    result = trSpreadingCode; 	

    //msg bit tracking
    if (0.0 > oldVector_.real() * curVector_.real())
      ++spreadingCodeHistogram_[spreadingCodeIndex_ % kSpreadingCodesPerBit];
    
    ++spreadingCodeIndex_;
    spreadingCodePhaseRemainder_ = chipPhase_;
        
    if(spreadingCodeIndex_ == 200)
    { //syncronise to message bit edges

      //find most common SpreadingCode transition     
      u32 index = 0;
      for(u32 i = 1; i < kSpreadingCodesPerBit; ++i)
        if(spreadingCodeHistogram_[index] < spreadingCodeHistogram_[i])
          index = i;
      
      //clear SpreadingCode transition tracking vector
      spreadingCodeHistogram_.clear();
  	  spreadingCodeHistogram_.resize(kSpreadingCodesPerBit);
	  
  	  //adjust spreadingCodeIndex offset so bit edge occurs at 
      //kSpreadingCodesPerBit - 1 == spreadingCodeIndex_ % kSpreadingCodesPerBit
      spreadingCodeIndex_ += kSpreadingCodesPerBit - 1 - index;
    }
    else if(spreadingCodeIndex_ > 200)
    {
      u32 const bitIndex = spreadingCodeIndex_ / kSpreadingCodesPerBit;
    	
      bool const bitMatch = 
        0 == ((spreadingCodeIndex_ + 1) % kSpreadingCodesPerBit);

      if(bitMatch)
      {
        result = trMsgBit;

        //construct 30-bit word + 2 previous word parity bits
        msgWord_ <<= 1;
        if(curVector_.real() > 0) 
   	    {
  	      //eventually keep history of previous 20 SpreadingCode dumps
	        //then average/vote
	        msgWord_ |= 1;
	      }
 
  	    //frame sync and construction
	
        u32 msgData;
        bool const parityMatch = checkWordParity(msgWord_,msgData);
        bool const tlmMatch = parityMatch && isPossibleTLMData(msgData);
       
        switch(subFrameTrackState_)
        {
          case ftsNoMatch:
            if(parityMatch && tlmMatch)
            {
              subFrameTrackState_ = ftsProbation;
              subFrameEndBit_ = bitIndex;
              subFrameRaw_.words_[0] = msgWord_;
            }
            break;
          case ftsProbation:
            if(0 == (bitIndex - subFrameEndBit_) % kMsgBitsPerWord)
            {
              bool failed = true;
              if(parityMatch)
              {
                u32 const wordIndex = 
                  (bitIndex - subFrameEndBit_) / kMsgBitsPerWord;
                  
                if(wordIndex == 1)
                {
                  failed = false; 
//this is another check but it does not work with the sample nav mesasge
//                  failed = ((msgWord_ & 3) == 0);
                  subFrameRaw_.words_[wordIndex] = msgWord_;
                }
                else if(wordIndex < kMsgWordsPerSubFrame)
                {
                  failed = false;
                  subFrameRaw_.words_[wordIndex] = msgWord_;
                }
                else if(wordIndex == kMsgWordsPerSubFrame)
                {
                  failed = !tlmMatch;
                  subFrameRaw_.words_[0] = msgWord_;
                }
                else if(wordIndex == (kMsgWordsPerSubFrame + 1))
                {
                  u32 TLM1st;
                  checkWordParity(subFrameRaw_.words_[1],TLM1st);
                  
                  u32 const truncatedZCount = (msgData >> 7) & 0x1FFFF;
                  
                  //this Truncated Z-Count must be one more
                  //then the Truncated Z-Count for the previous SubFrame
                  failed = truncatedZCount != (((TLM1st >> 7) + 1) & 0x1FFFF);

                  if(!failed)
                  {
                    subFrameRaw_.words_[1] = msgWord_;

                    // goto tracking mode but drop previous subframe
                    subFrameTrackState_ = ftsTracking;
                    spreadingCodeIndex_ = kSpreadingCodesPerBit * 
                      (truncatedZCount * 6 * 50 - kMsgBitsPerWord * 8) - 1;
                  }
                }
              }

              if(failed)
              { //expected word checks
                subFrameEndBit_ = 0;
                subFrameRaw_.clear();
                subFrameTrackState_ = ftsNoMatch;
              }
            }
            break;
          case ftsTracking:
            if(0 == (bitIndex + 1) % kMsgBitsPerWord)
            {
              u32 const wordIndex = 
                (bitIndex % (kMsgBitsPerWord * kMsgWordsPerSubFrame)) / kMsgBitsPerWord;
                
              subFrameRaw_.words_[wordIndex] = msgWord_;
              
              if(wordIndex == (kMsgWordsPerSubFrame - 1))
              {
                //reset spreading code index based on truncated Z count
                //in message.
                u32 TLM1st;
                checkWordParity(subFrameRaw_.words_[1],TLM1st);
                u32 const truncatedZCount = (TLM1st >> 7) & 0x1FFFF;
                spreadingCodeIndex_ = 
                  kSpreadingCodesPerBit * truncatedZCount * 6 * 50 - 1;
                result = trMsgSubFrame;
              }
              else
              {
                result = trMsgWord;
              }
            }
            break;
        }
      }
    }
  }
  
  if(inCallback) inCallback(result,*this);

  return result;
}
//---------------------------------------------------------------------------
f64
GPSCorrelatorMsgTrack::satTimeOfWeek()
const
{
  //calculate approximate fractional spreadingCode value
  f64 satTime = chipPhase_ - spreadingCodePhaseRemainder_;
  //remove chip offset used during normal tracking
  if(inProgress_) satTime -= chipResetOffset_;
  //convert from Chip Phase to Spreading Codes
  satTime /= (1 << 16) * (1 << 11) - chipResetOffset_;
  
  //add in spreading code of week value
  satTime += spreadingCodeIndex_;
  //convert from Spreading Codes to Seconds
  satTime /= kSpreadingCodesPerSecond;
  
  return satTime;
}
//---------------------------------------------------------------------------
  

