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
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
bool
processGPSCorrelator(
  f32 const*&               inBegin,
  f32 const* const          inEnd,
  GPSCorrelatorState* const ioState)
{
  register f32 ip;
  register f32 qp;
  register f32 it;
  register f32 qt;
  ioState->runningSums_.get(ip,qp,it,qt);

  f32 const* const orgBegin = inBegin;
  register u32 carrierPhase = ioState->carrierPhase_;
  const u32    carrierDelta = ioState->carrierDelta_;

  register u32 chipPhase    = ioState->chipPhase_;

  const u32    chipDelta    = ioState->chipDelta_;

  register f32 const* carrierData = cosData.data_;
  register GPSCorrelatorChips::ValueType const* chipBase = ioState->chipBase_;
  if(!ioState->inProgress_)
  {
    chipPhase += ioState->chipResetOffset_;
    ioState->processedSamples_ = 0;
  }

//  f32 register const* cosDataPntr = cosData.data_;
  
  register f32 const* currPntr = inBegin;
  while(currPntr != inEnd)
  {
    f32 register const d = *currPntr;

    f32 register const c = *(carrierData + CarrierData::cosineIndex(carrierPhase));
    f32 register const ns = *(carrierData + CarrierData::negSineIndex(carrierPhase));

    f32 register const i = d * c;
    f32 register const q = d * ns;

    f32 register const p = *(chipBase + GPSCorrelatorChips::promptIndex(chipPhase));
    f32 register const t = *(chipBase + GPSCorrelatorChips::trackIndex(chipPhase));

    ip += i * p;
    qp += q * p;
    it += i * t;
    qt += q * t;

    carrierPhase += carrierDelta;
    chipPhase += chipDelta;
    ++currPntr;

//    if((chipPhase >> 16) & (~0 << 11))
    //2^16 counts per half-chip, 
    //2^11-{0,2} half-chips per spreading code
    if(chipPhase >> 27) 
    { //dump
      inBegin = currPntr;
      ioState->carrierPhase_ = carrierPhase;
      ioState->chipPhase_ = (chipPhase & 0xFFFF);
      ioState->processedSamples_ += (inBegin - orgBegin);
      ioState->runningSums_.set(0,0,0,0);
      ioState->dumpedSums_.set(
        ip / ioState->processedSamples_,
        qp / ioState->processedSamples_,
        it / ioState->processedSamples_,
        qt / ioState->processedSamples_);
      ioState->inProgress_ = false;
      return true;
    }
  }

  inBegin = currPntr;
  ioState->runningSums_.set(ip,qp,it,qt);
  ioState->carrierPhase_ = carrierPhase;
  ioState->chipPhase_ = chipPhase;
  ioState->processedSamples_ += (inBegin - orgBegin);

  ioState->inProgress_ = true;
  return false;
}
//---------------------------------------------------------------------------
