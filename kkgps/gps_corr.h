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
#ifndef gps_corrH
#define gps_corrH
//---------------------------------------------------------------------------
#include <vector>
#include <cmath>
#include "kkutils.h"
//---------------------------------------------------------------------------

/*
interleave prompt with each of the three other chip patterns
leave a 0 at the beggining of each chip pattern so we can advance one whole
chip during rough search, during normal mode, when we reach the last chip instead
of setting offset to zero chips set to one chip
*/

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//      0 p/2   p 3p/4  2p
//cos   1   0  -1    0   1
//sin   0   1   0   -1   0
//-sin  0  -1   0    1   0
template<typename VA_,u32 BITS_>
class
CosData
{
public:
  typedef VA_ Result;
  enum {bits   = BITS_};
  enum {length = 1 << bits};
  enum {mask   = length - 1};
  enum {negSinOffset = 1 << 30};
  enum {negSinIndexOffset = negSinOffset >> (32 - bits)};

  CosData()
  {
    for(u32 i = 0; i < length; ++i)
      data_[i] = std::cos(M_PI * 2.0 * i / length);
  }
  
  static
  inline
  u32
  cosineIndex(
    u32 const inPhase)
  {
  	return inPhase >> (32 - bits);
  }
  
  static
  inline
  u32
  negSineIndex(
    u32 const inPhase)
  {
  	return ((inPhase >> (32 - bits)) + negSinIndexOffset) & mask;
  }

  inline
  Result const* 
  cosine(
    u32 const inPhase)
  const
  {
  	return data_ + cosineIndex(inPhase);
  }
    
  VA_                        data_[length];
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
typedef CosData<f32,16> CarrierData;
static const CarrierData cosData;
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
static const u32 g2Shift1[37] =
{2,3,4,5,1,2,1,2,3,2,3,5,6,7,8,9,1,2,3,4,5,6,1,4,5,6,7,8,1,2,3,4,5,4,1,2,4};
static const u32 g2Shift2[37] =
{6,7,8,9,9,10,8,9,10,3,4,6,7,8,9,10,4,5,6,7,8,9,3,6,7,8,9,10,6,7,8,9,10,10,7,8,10};
//---------------------------------------------------------------------------
template<typename IT_,typename VA_>
void
generateCACode(
  u32 const        inPrnNumber,
  VA_ const        inTrueValue,
  VA_ const        inFalseValue,
  IT_              inBegin,
  IT_ const        inEnd)
{
  if((0 == inPrnNumber) || (37 <= inPrnNumber))
  { //if out of range for valid PRN numbers set all to 1
    for(u32 i = 0; (i < 1023) && (inBegin != inEnd); ++i)
    {
      *inBegin = inTrueValue;
      ++inBegin;
    }
  }
  else
  { //valid PRN Number 
    u32 g1Code = ~0;
    u32 g2Code = ~0;
  
    for(u32 i = 0; (i < 1023) && (inBegin != inEnd); ++i)
    {
      //generate g1 bit
      g1Code <<= 1;
      u32 const g1CodeNewBit =
        (g1Code >> 10) ^ (g1Code >> 3);
      g1Code |= 1 & g1CodeNewBit;
  
      //generate g2 bit
      g2Code <<= 1;
      u32 const g2CodeNewBit =
        (g2Code >> 10) ^ (g2Code >> 9) ^ (g2Code >> 8) ^
        (g2Code >> 6) ^ (g2Code >> 3) ^ (g2Code >> 2);
      g2Code |= 1 & g2CodeNewBit;
  
      u32 const caCodeNewBit =
        (g1Code >> 10) ^
        (g2Code >> g2Shift1[inPrnNumber-1]) ^
        (g2Code >> g2Shift2[inPrnNumber-1]);
  
      *inBegin = (1 & caCodeNewBit)?(inTrueValue):(inFalseValue);
      ++inBegin;
    }
  }
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
GPSCorrelatorChips
{
public:
  typedef f32 ValueType;
  enum {bits   = 11};
  enum {length = 1 << bits};
  enum {mask   = length - 1};

  GPSCorrelatorChips(
    u32 const                inPrnNumber)
  {
    generateChips(inPrnNumber);
  }

  void
  generateChips(
    u32 const                inPrnNumber)
  {
    prnNumber_ = inPrnNumber;
    std::vector<ValueType> ca(1023);
    generateCACode(inPrnNumber,1,-1,ca.begin(),ca.end());

    for(u32 i = 0; i < 2; ++i)
    {
      promptEarly_[i]                   = 0;
      promptLate_[i]                    = 0;
      promptEarlyMinusLate_[i]          = 0;
      promptEarly_[length + i]          = 0;
      promptLate_[length + i]           = 0;
      promptEarlyMinusLate_[length + i] = 0;
    }

    for(u32 i = 1; i < length / 2; ++i)
    {
      u32 const caIndex = i - 1;
      u32 const arrIndex = 2 * i;
      ValueType const prompt = ca[caIndex];
      ValueType const next = ca[(caIndex + 1) % 1023];
      ValueType const prev = ca[(caIndex + 1023 - 1) % 1023];

      promptEarly_[arrIndex]                     = prompt;
      promptEarly_[arrIndex+1]          		     = prompt;
      promptLate_[arrIndex]                      = prompt;
      promptLate_[arrIndex+1]                    = prompt;
      promptEarlyMinusLate_[arrIndex]            = prompt;
      promptEarlyMinusLate_[arrIndex+1]          = prompt;

      promptEarly_[length + arrIndex]            = prompt;
      promptEarly_[length + arrIndex+1]          = next;
      promptLate_[length + arrIndex]             = prev;
      promptLate_[length + arrIndex+1]           = prompt;
      promptEarlyMinusLate_[length + arrIndex]   = prompt - prev;
      promptEarlyMinusLate_[length + arrIndex+1] = next - prompt;
    }
  }

  static
  inline
  u32
  promptIndex(
    u32 const chipPhase)
  {
    return (chipPhase >> 16) & mask; 
  }

  static
  inline
  u32
  trackIndex(
    u32 const chipPhase)
  {
  	return ((chipPhase >> 16) & mask) + length;
  }

  u32                        prnNumber_;
  ValueType           			 promptEarly_[2 * length];
  ValueType            		   promptLate_[2 * length];
  ValueType						       promptEarlyMinusLate_[2 * length];
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
GPSCorrelatorState
{
public:
  GPSCorrelatorState(
    f64 const                inSampleRate,
    f64 const                inCarrierFreq,
    f64 const                inChipRate)
  : sampleRate_(inSampleRate)
  , carrierPhase_(0)
  , carrierDelta_(0)
  , chipPhase_(0)
  , chipDelta_(0)
  , chipResetOffset_(0)
  , processedSamples_(0)
  , chipBase_(0)
  , inProgress_(0)
  {
    carrierFreqSet(inCarrierFreq);
    chipRateSet(inChipRate);
  }

  void
  sampleRateSet(
    f64 const                inSampleRate)
  {
    //get sample rate independent carrier frequency
    f64 const carrierFreq = carrierFreqGet();
    //get sample rate independent chip rate
    f64 const chipRate = chipRateGet();
    //adjust sample rate
    sampleRate_ = inSampleRate;
    //set carrier frequency to value from before sample rate update
    carrierFreqSet(carrierFreq);
    //set chip rate to value from before sample rate update
    chipRateSet(chipRate);
  }

  f64
  carrierFreqGet()
  const
  {
    return sampleRate_ * carrierDelta_ / std::pow(2.0,32);
  }

  void
  carrierFreqSet(
    f64 const                inCarrierFreq)
  {
    carrierDelta_ = static_cast<u32>(inCarrierFreq * std::pow(2.0,32) / sampleRate_);
  }

  f64
  chipRateGet()
  const
  {
    return sampleRate_ * chipDelta_ / std::pow(2.0,16);
  }

  void
  chipRateSet(
    f64 const                inChipRate)
  {
    chipDelta_ = static_cast<u32>(inChipRate * std::pow(2.0,16) / sampleRate_);
  }

  f64                        sampleRate_;
  u32                        carrierPhase_;
  u32                        carrierDelta_;
  u32                        chipPhase_;
  u32                        chipDelta_;
  u32                        chipResetOffset_;
  u64                        processedSamples_;
  GPSCorrelatorChips::ValueType* chipBase_;

  bool                       inProgress_;

  struct
  Sums
  {
    Sums()
    : ip_(0.0)
    , qp_(0.0)
    , it_(0.0)
    , qt_(0.0)
    {
    };

    void
    set(
      f32 const              in_ip,
      f32 const              in_qp,
      f32 const              in_it,
      f32 const              in_qt)
    {
      ip_ = in_ip;
      qp_ = in_qp;
      it_ = in_it;
      qt_ = in_qt;
    }

    void
    get(
      f32&                   in_ip,
      f32&                   in_qp,
      f32&                   in_it,
      f32&                   in_qt)
    const
    {
      in_ip = ip_;
      in_qp = qp_;
      in_it = it_;
      in_qt = qt_;
    }

    f32 ip_,qp_,it_,qt_;
  };

  Sums                       runningSums_;
  Sums                       dumpedSums_;
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
GPSCorrelatorTrack
: public GPSCorrelatorState
{
public:
  GPSCorrelatorTrack(
    f64 const                inSampleRate,
    f64 const                inIntermediateFreq,
    f64 const                inChipRate,
    u32 const                inPrnNumber);

  void
  reset();
  
  void
  start();
  
  void
  prnNumberSet(
    u32 const                 inPrnNumber);
    
  f64
  dopplerDeltaFreq()
  const;

  bool
  doTrack(
    f32 const*&               inBegin,
    f32 const* const          inEnd);

  enum
  TrackingState
  { tsIdle
  , tsSearch
  , tsSearchConfirm
  , tsAdjustCarrierFrequency
  , tsAdjustCarrierPhase
  , tsAdjustConfirm
  , tsTracking
  };


  GPSCorrelatorChips         chips_;
  TrackingState              trackingState_;
  f64                        intermediateFreq_;
  f64                        deltaSearchFreq_;
  f64                        minSearchFreq_;
  f64                        maxSearchFreq_;
  f64                        carrierFreq_;
  s32                        codeSearchIndex_;
  f64                        searchFreqBase_;
  f64                        searchFreqUpper_;
  f64                        searchFreqLower_;
  
  f64                        lastFreqError_;

  u32                        history_;
  cf64                       curVector_;
  cf64                       oldVector_;
  s32                        confirmCount_;

};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
#endif
