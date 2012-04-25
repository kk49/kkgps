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
#include "gps_corr.h"
#include <xmmintrin.h>
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
bool
processGPSCorrelator(
  f32 const*&               inBegin,
  f32 const* const          inEnd,
  GPSCorrelatorState* const ioState)
{
  //ip f[0]
  //qp f[1]
  //it f[2]
  //qt f[3]
  union 
  {
    __m128 vec;
    f32 f[4];
  } convert;
  ioState->runningSums_.get(convert.f[0],convert.f[1],convert.f[2],convert.f[3]);
  register __m128 sum = convert.vec;

  f32 const* const orgBegin = inBegin;

  register f32 const* currPntr      = inBegin;
  register f32 const* const endPntr = inEnd;
  register u32 carrierPhase         = ioState->carrierPhase_;
  register u32 const carrierDelta 	= ioState->carrierDelta_;
  
  register u32 chipPhase            = ioState->chipPhase_;
  register u32 const chipDelta      = ioState->chipDelta_;
  register f32 const* carrierData   = cosData.data_;
  register f32 const* chipBase      = ioState->chipBase_;
  
  if(!ioState->inProgress_)
  {
    chipPhase += ioState->chipResetOffset_;
    ioState->processedSamples_ = 0;
  }

//  f32 register const* cosDataPntr = cosData.data_;
    
  while(currPntr != endPntr)
  {
    register __m128 data;
    register __m128 cosine;
    register __m128 cosnsin;
    register __m128 nsine;
    register __m128 promptCode;
    register __m128 trackCode;
    register __m128 despCode;
    
    //data <- {*inBegin,*inBegin,*inBegin,*inBegin}
    data = _mm_load_ps1(currPntr);
    
    //cosnsin <- {cos,-sin,cos,-sin}
    cosine = _mm_load_ss(carrierData + CarrierData::cosineIndex(carrierPhase));
    nsine  = _mm_load_ss(carrierData + CarrierData::negSineIndex(carrierPhase));
    cosnsin = _mm_shuffle_ps(cosine,nsine,_MM_SHUFFLE(0,0,0,0)); 
    cosnsin = _mm_shuffle_ps(cosnsin,cosnsin,_MM_SHUFFLE(2,0,2,0)); 
    
    //despCode <- {prompt,prompt,track,track}
    promptCode = _mm_load_ss(chipBase + GPSCorrelatorChips::promptIndex(chipPhase));
    trackCode = _mm_load_ss(chipBase + GPSCorrelatorChips::trackIndex(chipPhase));
    despCode = _mm_shuffle_ps(promptCode,trackCode,_MM_SHUFFLE(0,0,0,0)); 

    ++currPntr;
    carrierPhase += carrierDelta;
    chipPhase += chipDelta;

    data = _mm_mul_ps(data,cosnsin);
    data = _mm_mul_ps(data,despCode);
    sum  = _mm_add_ps(sum,data);

//    if((chipPhase >> 16) & (~0 << 11))
    //2^16 counts per half-chip, 
    //2^11-{0,2} half-chips per spreading code
    if(chipPhase >> 27) 
    { //dump
      convert.vec = sum;
      inBegin = currPntr;
    
      ioState->carrierPhase_ = carrierPhase;
      ioState->chipPhase_ = (chipPhase & 0xFFFF);
      ioState->processedSamples_ += (inBegin - orgBegin);
      ioState->runningSums_.set(0,0,0,0);
      ioState->dumpedSums_.set(
        convert.f[0] / ioState->processedSamples_,
        convert.f[1] / ioState->processedSamples_,
        convert.f[2] / ioState->processedSamples_,
        convert.f[3] / ioState->processedSamples_);
      ioState->inProgress_ = false;
      return true;
    }
  }

  convert.vec = sum;
  inBegin = currPntr;

  ioState->runningSums_.set(convert.f[0],convert.f[1],convert.f[2],convert.f[3]);
  ioState->carrierPhase_ = carrierPhase;
  ioState->chipPhase_ = chipPhase;
  ioState->processedSamples_ += (inBegin - orgBegin);

  ioState->inProgress_ = true;
  return false;
}
//---------------------------------------------------------------------------
