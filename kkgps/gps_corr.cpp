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
#include "gps_corr_generic.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//GPSCorrelatorTrack::
//---------------------------------------------------------------------------
GPSCorrelatorTrack::GPSCorrelatorTrack(
  f64 const                inSampleRate,
  f64 const                inIntermediateFreq,
  f64 const                inChipRate,
  u32 const                inPrnNumber)
: GPSCorrelatorState(inSampleRate,inIntermediateFreq,inChipRate)
, chips_(inPrnNumber)
, trackingState_(tsIdle)
, intermediateFreq_(inIntermediateFreq)
, deltaSearchFreq_(200.0)
, minSearchFreq_(inIntermediateFreq - 20e3)
, maxSearchFreq_(inIntermediateFreq + 20e3)
, carrierFreq_(inIntermediateFreq)
{
  reset();
}
//---------------------------------------------------------------------------
void
GPSCorrelatorTrack::reset()
{
  trackingState_ = tsIdle;
  
  codeSearchIndex_ = 0;
  curVector_ = -1;
  oldVector_ = -1;
  confirmCount_ = 0;
  
  lastFreqError_ = 0;

  searchFreqBase_ = carrierFreq_;
  searchFreqUpper_ = searchFreqBase_;
  searchFreqLower_ = searchFreqBase_;
}
//---------------------------------------------------------------------------
void
GPSCorrelatorTrack::start()
{
  trackingState_ = tsSearch;
}
//---------------------------------------------------------------------------
void
GPSCorrelatorTrack::prnNumberSet(
  u32 const                 inPrnNumber)
{
  chips_.generateChips(inPrnNumber);
}
//---------------------------------------------------------------------------
f64
GPSCorrelatorTrack::dopplerDeltaFreq()
const
{
  return carrierFreqGet() - intermediateFreq_;
}
//---------------------------------------------------------------------------
bool
GPSCorrelatorTrack::doTrack(
  f32 const*&              inBegin,
  f32 const* const         inEnd)
{
  //process block of data
  switch(trackingState_)
  {
    case tsSearch:
      chipResetOffset_ = 0;
      chipBase_ = chips_.promptEarly_;
      break;
    default:
      chipResetOffset_ = (2 << 16);
      chipBase_ = chips_.promptEarlyMinusLate_;
      break;
  }

  bool const dump = processGPSCorrelator(inBegin,inEnd,this);

  if(dump)
  {
    u64 const currSamples = processedSamples_;
    f32 const ip = dumpedSums_.ip_;
    f32 const qp = dumpedSums_.qp_;
    f32 const it = dumpedSums_.it_;
    f32 const qt = dumpedSums_.qt_;

    oldVector_ = curVector_;
    curVector_ = cf64(ip,qp);
    history_ <<= 1;
    if(curVector_.real() > 0) history_ |= 1;
      
//    f64 const npv = norm(curVector_);
    f64 const npv = norm(cf64(ip+it,qp+qt));

    f64 const threshold = 40e6 / (20e3 * 20e3);

    if((tsIdle != trackingState_) && (npv < threshold))
    {
      trackingState_ = tsSearch;
    }

    switch(trackingState_)
    {
      case tsIdle:
        {
        }
        break;
      case tsSearch:
        if(npv >= threshold)
        {
          trackingState_ = tsSearchConfirm;
          confirmCount_ = 0;
        }
        else
        {
          ++codeSearchIndex_;
          if(codeSearchIndex_ >= 1024)
          { //completed code search at this frequency
            //select new frequency
            codeSearchIndex_ = 0;

            bool const maxReached = (maxSearchFreq_ == searchFreqUpper_);
            bool const minReached = (minSearchFreq_ == searchFreqLower_);
//            bool const baseIsAtUpper = (searchFreqBase_ == searchFreqUpper_);
            bool const baseIsAtLower = (searchFreqBase_ == searchFreqLower_);

            if(maxReached && minReached)
            {
              trackingState_ = tsIdle;
            }
            else if(!maxReached && baseIsAtLower)
            {
              searchFreqUpper_ += deltaSearchFreq_;
              searchFreqBase_ = searchFreqUpper_;
            }
            else
            {
              searchFreqLower_ -= deltaSearchFreq_;
              searchFreqBase_ = searchFreqLower_;
            }

            carrierFreqSet(searchFreqBase_);
          }
        }
        break;
      case tsSearchConfirm:
        {
          ++confirmCount_;
          if(confirmCount_ > 5) trackingState_ = tsAdjustCarrierFrequency;
        }
        break;
      case tsAdjustCarrierFrequency:
        {
          //adjust carrier frequency
          f64 freqTheta = std::arg(curVector_ / oldVector_);

          if(freqTheta > (M_PI * 0.5))
            freqTheta -= M_PI;
          if(freqTheta < (-M_PI * 0.5))
            freqTheta += M_PI;

          freqTheta *= (sampleRate_ / (2 * M_PI)) / currSamples;

          f64 const freqError = freqTheta * pow(2.0,32) / sampleRate_;
          f64 fet;
          if(freqError > 0)
            fet = floor(freqError + 0.5);
          else
            fet = ceil(freqError - 0.5);
          carrierDelta_ += static_cast<u32>(fet);

          trackingState_ = tsAdjustCarrierPhase;
        }
        break;
      case tsAdjustCarrierPhase:
        {
          //adjust carrier phase
          f64 phase_theta = arg(curVector_);

          if(phase_theta > (M_PI * 0.5))
            phase_theta -= M_PI;
          if(phase_theta < (-M_PI * 0.5))
            phase_theta += M_PI;

          phase_theta *= (1 / (2 * M_PI));
          f64 const phase_error = phase_theta * pow(2.0,32);
          f64 pet;
          if(phase_error > 0)
            pet = floor(phase_error + 0.5);
          else
            pet = ceil(phase_error - 0.5);
          carrierPhase_ += static_cast<u32>(pet);
 
          confirmCount_ = 0;
          trackingState_ = tsAdjustConfirm;
        }
        break;
      case tsAdjustConfirm:
        {
          ++confirmCount_;
          if(confirmCount_ > 5) trackingState_ = tsTracking;
        }
        break;
      case tsTracking:
        {
          //carrier frequency PLL
          f64 freqTheta = atan2(qp,ip);

          if(freqTheta > (M_PI * 0.5))
            freqTheta -= M_PI;
          if(freqTheta < (-M_PI * 0.5))
            freqTheta += M_PI;

          freqTheta *= (sampleRate_ / (2 * M_PI)) / currSamples;

          f64 const freqError = freqTheta * pow(2.0,32) / sampleRate_;


/*
//eventual proper carrier loop filter
            f64 const k_a = 0.6;
            f64 const k_k = 0.05;
            f64 static ynm1 = 0;
            ynm1 = k_a * ynm1 + (1.0 - k_a) * freqError;
            carrierDelta_ += k_k * ynm1;
*/

          
          carrierDelta_ += static_cast<u32>(
            0.1 * freqError + 0.2 * (freqError - lastFreqError_));

          lastFreqError_ = freqError;


          //code phase Lock Loop
          cf64 const tv(it,qt);
          cf64 const tDivP = tv / curVector_;
          f64 const s = tDivP.real() + tDivP.imag();

          chipPhase_ += static_cast<u32>((1<<16) * 0.5 * s);
        }
        break;
    }
  }
  
  return dump; //dump did not occur
}
//---------------------------------------------------------------------------
