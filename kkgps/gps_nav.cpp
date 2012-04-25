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
#include "gps_nav.h"
#include <fstream>
//debug#include <iostream>
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//GPS standard constants
//WGS 84 value of earth's universal gravitational parameter
f64 const k_mu = 3.986005e14; //meter^3 / second^2 
//speed of light
f64 const k_c = 2.99792458e8; //meter / second
//WGS 84 value of the earth's rotation rate
f64 const k_dot_omega_e = 7.2921151467e-5; //radians / second
//GPS standard Pi
f64 const k_pi = 3.1415926535898;
//GPS radians per semi circle
f64 const k_semi_circle_radians = k_pi;
//GPS inital guess on satellite distance
//satellite orbital radius - earth average radius
f64 const k_satellite_orbital_radius_m = 26560000 - 6368000;
//
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
void 
Invert4x4Matrix( f64 *mat, f64 *dst)
{
  f64 const a00 = mat[4*0 + 0];
  f64 const a01 = mat[4*0 + 1];
  f64 const a02 = mat[4*0 + 2];
  f64 const a03 = mat[4*0 + 3];
  f64 const a10 = mat[4*1 + 0];
  f64 const a11 = mat[4*1 + 1];
  f64 const a12 = mat[4*1 + 2];
  f64 const a13 = mat[4*1 + 3];
  f64 const a20 = mat[4*2 + 0];
  f64 const a21 = mat[4*2 + 1];
  f64 const a22 = mat[4*2 + 2];
  f64 const a23 = mat[4*2 + 3];
  f64 const a30 = mat[4*3 + 0];
  f64 const a31 = mat[4*3 + 1];
  f64 const a32 = mat[4*3 + 2];
  f64 const a33 = mat[4*3 + 3];

#define det2x2(A00,A01,A10,A11) \
  (A00 * A11 - A01 * A10)  
#define det3x3(A00,A01,A02,A10,A11,A12,A20,A21,A22) \
  (A00 * det2x2(A11,A12,A21,A22) - A01 * det2x2(A10,A12,A20,A22) + A02 * det2x2(A10,A11,A20,A21)) 
  
  f64 const r00 =  det3x3(a11,a12,a13,a21,a22,a23,a31,a32,a33);
  f64 const r01 = -det3x3(a10,a12,a13,a20,a22,a23,a30,a32,a33);
  f64 const r02 =  det3x3(a10,a11,a13,a20,a21,a23,a30,a31,a33);
  f64 const r03 = -det3x3(a10,a11,a12,a20,a21,a22,a30,a31,a32);
  f64 const r10 = -det3x3(a01,a02,a03,a21,a22,a23,a31,a32,a33);
  f64 const r11 =  det3x3(a00,a02,a03,a20,a22,a23,a30,a32,a33);
  f64 const r12 = -det3x3(a00,a01,a03,a20,a21,a23,a30,a31,a33);
  f64 const r13 =  det3x3(a00,a01,a02,a20,a21,a22,a30,a31,a32);
  f64 const r20 =  det3x3(a01,a02,a03,a11,a12,a13,a31,a32,a33);
  f64 const r21 = -det3x3(a00,a02,a03,a10,a12,a13,a30,a32,a33);
  f64 const r22 =  det3x3(a00,a01,a03,a10,a11,a13,a30,a31,a33);
  f64 const r23 = -det3x3(a00,a01,a02,a10,a11,a12,a30,a31,a32);
  f64 const r30 = -det3x3(a01,a02,a03,a11,a12,a13,a21,a22,a23);
  f64 const r31 =  det3x3(a00,a02,a03,a10,a12,a13,a20,a22,a23);
  f64 const r32 = -det3x3(a00,a01,a03,a10,a11,a13,a20,a21,a23);
  f64 const r33 =  det3x3(a00,a01,a02,a10,a11,a12,a20,a21,a22);
  
  f64 const det = a00 * r00 + a01 * r01 + a02 * r02 + a03 * r03;
  f64 const invDet = (0.0 == det)?(0.0):(1.0 / det);

  dst[4*0 + 0] = invDet * r00;
  dst[4*0 + 1] = invDet * r10;
  dst[4*0 + 2] = invDet * r20;
  dst[4*0 + 3] = invDet * r30;
  dst[4*1 + 0] = invDet * r01;
  dst[4*1 + 1] = invDet * r11; 
  dst[4*1 + 2] = invDet * r21;
  dst[4*1 + 3] = invDet * r31;
  dst[4*2 + 0] = invDet * r02;
  dst[4*2 + 1] = invDet * r12;
  dst[4*2 + 2] = invDet * r22;
  dst[4*2 + 3] = invDet * r32;
  dst[4*3 + 0] = invDet * r03;
  dst[4*3 + 1] = invDet * r13;
  dst[4*3 + 2] = invDet * r23;
  dst[4*3 + 3] = invDet * r33;
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
f64
limitTimeToValidRange(
  f64 const                  inTime)
{
  if(inTime < -302400) return inTime + 604800;
  if(inTime > 302400) return inTime - 604800;
  return inTime; 
}
//---------------------------------------------------------------------------
template<typename TYPE_>
TYPE_
parseMsgDataI32Helper(
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord,
  u32 const                  inLsbFirstBit,
  u32 const                  inLsbLastBit)
{
  TYPE_ value;

  u32 const msbFirstBit = 24 - inMsbFirstBit;
  u32 const msbLastBit = 24 - inMsbLastBit;

  //shift left to put FirstBit in Msb bit location so that if value is a 
  //signed integer the sign will be saved
  value = inMsbWord << (31 - msbFirstBit);

  //bring value back to lsb
  value >>= (31 - msbFirstBit) + msbLastBit; 

  if(inLsbFirstBit && inLsbLastBit)
  {
    u32 const lsbWidth = (inLsbLastBit - inLsbFirstBit + 1);
    u32 const lsbMask = ~(0xFFFFFFFF << lsbWidth);
    u32 const lsbOffset = 24 - inLsbLastBit;

    value <<= lsbWidth;
    value |= (inLsbWord >> lsbOffset) & lsbMask;
  }

  return value;
}
//---------------------------------------------------------------------------
template<typename TYPE_>
f64
parseMsgDataF64Helper(
  s32 const                  inScale,
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord,
  u32 const                  inLsbFirstBit,
  u32 const                  inLsbLastBit)
{
  return std::ldexp(
    static_cast<f64>(parseMsgDataI32Helper<TYPE_>(
      inMsbWord,inMsbFirstBit,inMsbLastBit,
      inLsbWord,inLsbFirstBit,inLsbLastBit)),
    inScale);
}
//---------------------------------------------------------------------------
s32
parseMsgDataS32(
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord,
  u32 const                  inLsbFirstBit,
  u32 const                  inLsbLastBit)
{
  return parseMsgDataI32Helper<s32>(
    inMsbWord,inMsbFirstBit,inMsbLastBit,inLsbWord,inLsbFirstBit,inLsbLastBit);
}
//---------------------------------------------------------------------------
u32
parseMsgDataU32(
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord,
  u32 const                  inLsbFirstBit,
  u32 const                  inLsbLastBit)
{
  return parseMsgDataI32Helper<u32>(
    inMsbWord,inMsbFirstBit,inMsbLastBit,inLsbWord,inLsbFirstBit,inLsbLastBit);
}
//---------------------------------------------------------------------------
f64
parseMsgDataSF64(
  s32 const                  inScale,
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord,
  u32 const                  inLsbFirstBit,
  u32 const                  inLsbLastBit)
{
  return parseMsgDataF64Helper<s32>(
    inScale,
    inMsbWord,inMsbFirstBit,inMsbLastBit,inLsbWord,inLsbFirstBit,inLsbLastBit);
}  
//---------------------------------------------------------------------------
f64
parseMsgDataUF64(
  s32 const                  inScale,
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord,
  u32 const                  inLsbFirstBit,
  u32 const                  inLsbLastBit)
{
  return parseMsgDataF64Helper<u32>(
    inScale,
    inMsbWord,inMsbFirstBit,inMsbLastBit,inLsbWord,inLsbFirstBit,inLsbLastBit);
}  
//---------------------------------------------------------------------------
bool
SubFrameIDGet(
  SubFrameRaw const&         inSubFrame,
  s32&                       ouSubFrameID,
  s32&                       ouPage)
{
  ouSubFrameID = 0;
  ouPage = 0;
  
  u32 dataWord;
  if(!checkWordParity(inSubFrame.words_[1],dataWord))
    return false; 
  
  ouSubFrameID = (dataWord >> 2) & 0x7;
  
  switch(ouSubFrameID)
  {
     case 1:
     case 2:
     case 3: 
       return true;
     case 4:
     case 5:
       //not implemented
       return false;
     default:
       return false;
  }
}
//---------------------------------------------------------------------------

#define M_DUMP_VALUE(V_) result << #V_ << ": " << V_ << std::endl

//---------------------------------------------------------------------------
//SubFrameBase::
//---------------------------------------------------------------------------
bool
SubFrameBase::parse(
  SubFrameRaw const&       inSubFrame)
{
  u32 dataWord1;
  if(!checkWordParity(inSubFrame.words_[0],dataWord1))
    return false; 
  u32 dataWord2;
  if(!checkWordParity(inSubFrame.words_[1],dataWord2))
    return false; 
    
//sub-frame data
  telemetryMessage_  = parseMsgDataU32(dataWord1,9,22);
  truncatedTowCount_ = parseMsgDataU32(dataWord2,1,17);
  alertFlag_         = parseMsgDataU32(dataWord2,18,18);
  antiSpoofFlag_     = parseMsgDataU32(dataWord2,19,19);
  subFrameId_        = parseMsgDataU32(dataWord2,20,22);
  
  return true;
}
//---------------------------------------------------------------------------
std::string
SubFrameBase::debugDump()
{
  std::stringstream result;
  
  M_DUMP_VALUE(telemetryMessage_);
  M_DUMP_VALUE(truncatedTowCount_);
  M_DUMP_VALUE(alertFlag_);
  M_DUMP_VALUE(antiSpoofFlag_);
  M_DUMP_VALUE(subFrameId_);
  
  return result.str();
}
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//SubFrame1::
//---------------------------------------------------------------------------
bool
SubFrame1::parse(
  SubFrameRaw const&       inSubFrame)
{
  if(!SubFrameBase::parse(inSubFrame))
    return false;
    
  if(1 != subFrameId_)
    return false;
    
  u32 dataWord3;
  if(!checkWordParity(inSubFrame.words_[2],dataWord3))
    return false; 
  u32 dataWord4;
  if(!checkWordParity(inSubFrame.words_[3],dataWord4))
    return false; 
  u32 dataWord7;
  if(!checkWordParity(inSubFrame.words_[6],dataWord7))
    return false; 
  u32 dataWord8;
  if(!checkWordParity(inSubFrame.words_[7],dataWord8))
    return false; 
  u32 dataWord9;
  if(!checkWordParity(inSubFrame.words_[8],dataWord9))
    return false; 
  u32 dataWord10;
  if(!checkWordParity(inSubFrame.words_[9],dataWord10))
    return false; 

//sub-frame data
  gpsWeekNumber_     = parseMsgDataU32(dataWord3,1,10);
  codesOnL2_         = parseMsgDataU32(dataWord3,11,12);
  svAccuracy_        = parseMsgDataU32(dataWord3,13,16);
  svHealthSummary_   = parseMsgDataU32(dataWord3,17,17);
  svHealth_          = parseMsgDataU32(dataWord3,18,22);
  IODC_              = parseMsgDataU32(dataWord3,23,24,dataWord8,1,8);
  pCodeOffOnL2_      = parseMsgDataU32(dataWord4,1,1);
  T_GD_              = parseMsgDataSF64(-31,dataWord7,17,24);
  t_oc_              = parseMsgDataU32(dataWord8,9,24) << 4;
  a_f2_              = parseMsgDataSF64(-55,dataWord9,1,8);
  a_f1_              = parseMsgDataSF64(-43,dataWord9,9,24); 
  a_f0_              = parseMsgDataSF64(-31,dataWord10,1,22);
                  
  return true;
}
//---------------------------------------------------------------------------
std::string
SubFrame1::debugDump()
{
  std::stringstream result;
  result << SubFrameBase::debugDump();
  
  M_DUMP_VALUE(gpsWeekNumber_);
  M_DUMP_VALUE(codesOnL2_);
  M_DUMP_VALUE(svAccuracy_);
  M_DUMP_VALUE(svHealthSummary_);
  M_DUMP_VALUE(svHealth_);
  M_DUMP_VALUE(IODC_);
  M_DUMP_VALUE(pCodeOffOnL2_);
  M_DUMP_VALUE(T_GD_);
  M_DUMP_VALUE(t_oc_);
  M_DUMP_VALUE(a_f2_);
  M_DUMP_VALUE(a_f1_);
  M_DUMP_VALUE(a_f0_);

  return result.str();
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//SubFrame2::
//---------------------------------------------------------------------------
bool
SubFrame2::parse(
  SubFrameRaw const&       inSubFrame)
{
  if(!SubFrameBase::parse(inSubFrame))
    return false;
    
  if(2 != subFrameId_)
    return false;
    
  u32 dataWord3;
  if(!checkWordParity(inSubFrame.words_[2],dataWord3))
    return false; 
  u32 dataWord4;
  if(!checkWordParity(inSubFrame.words_[3],dataWord4))
    return false; 
  u32 dataWord5;
  if(!checkWordParity(inSubFrame.words_[4],dataWord5))
    return false; 
  u32 dataWord6;
  if(!checkWordParity(inSubFrame.words_[5],dataWord6))
    return false; 
  u32 dataWord7;
  if(!checkWordParity(inSubFrame.words_[6],dataWord7))
    return false; 
  u32 dataWord8;
  if(!checkWordParity(inSubFrame.words_[7],dataWord8))
    return false; 
  u32 dataWord9;
  if(!checkWordParity(inSubFrame.words_[8],dataWord9))
    return false; 
  u32 dataWord10;
  if(!checkWordParity(inSubFrame.words_[9],dataWord10))
    return false; 

//sub-frame data
  sf2IODE_           = parseMsgDataU32(dataWord3,1,8);
  C_rs_              = parseMsgDataSF64( -5,dataWord3,9,24);
  deltan_            = parseMsgDataSF64(-43,dataWord4,1,16);
  M_0_               = parseMsgDataSF64(-31,dataWord4,17,24,dataWord5,1,24);
  C_uc_              = parseMsgDataSF64(-29,dataWord6,1,16);
  e_                 = parseMsgDataUF64(-33,dataWord6,17,24,dataWord7,1,24);
  C_us_              = parseMsgDataSF64(-29,dataWord8,1,16);
  sqrtA_             = parseMsgDataUF64(-19,dataWord8,17,24,dataWord9,1,24);
  t_oe_              = parseMsgDataU32(dataWord10,1,16) << 4;
  fitIntervalFlag_   = parseMsgDataU32(dataWord10,17,17);
  AODO_              = parseMsgDataU32(dataWord10,18,22);
  
  return true;
}
//---------------------------------------------------------------------------
std::string
SubFrame2::debugDump()
{
  std::stringstream result;
  result << SubFrameBase::debugDump();
  
  M_DUMP_VALUE(sf2IODE_);
  M_DUMP_VALUE(C_rs_);
  M_DUMP_VALUE(deltan_);
  M_DUMP_VALUE(M_0_);
  M_DUMP_VALUE(C_uc_);
  M_DUMP_VALUE(e_);
  M_DUMP_VALUE(C_us_);
  M_DUMP_VALUE(sqrtA_);
  M_DUMP_VALUE(t_oe_);
  M_DUMP_VALUE(fitIntervalFlag_);
  M_DUMP_VALUE(AODO_);

  return result.str();
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//SubFrame3::
//---------------------------------------------------------------------------
bool
SubFrame3::parse(
  SubFrameRaw const&       inSubFrame)
{
  if(!SubFrameBase::parse(inSubFrame))
    return false;
    
  if(3 != subFrameId_)
    return false;
    
  u32 dataWord3;
  if(!checkWordParity(inSubFrame.words_[2],dataWord3))
    return false; 
  u32 dataWord4;
  if(!checkWordParity(inSubFrame.words_[3],dataWord4))
    return false; 
  u32 dataWord5;
  if(!checkWordParity(inSubFrame.words_[4],dataWord5))
    return false; 
  u32 dataWord6;
  if(!checkWordParity(inSubFrame.words_[5],dataWord6))
    return false; 
  u32 dataWord7;
  if(!checkWordParity(inSubFrame.words_[6],dataWord7))
    return false; 
  u32 dataWord8;
  if(!checkWordParity(inSubFrame.words_[7],dataWord8))
    return false; 
  u32 dataWord9;
  if(!checkWordParity(inSubFrame.words_[8],dataWord9))
    return false; 
  u32 dataWord10;
  if(!checkWordParity(inSubFrame.words_[9],dataWord10))
    return false; 

//sub-frame data
  C_ic_            = parseMsgDataSF64(-29,dataWord3,1,16);
  omega_0_         = parseMsgDataSF64(-31,dataWord3,17,24,dataWord4,1,24);
  C_is_            = parseMsgDataSF64(-29,dataWord5,1,16);
  i_0_             = parseMsgDataSF64(-31,dataWord5,17,24,dataWord6,1,24);
  C_rc_            = parseMsgDataSF64( -5,dataWord7,1,16);
  w_               = parseMsgDataSF64(-31,dataWord7,17,24,dataWord8,1,24);
  dot_omega_       = parseMsgDataSF64(-43,dataWord9,1,24);
  sf3IODE_         = parseMsgDataU32(dataWord10,1,8);
  IDOT_            = parseMsgDataSF64(-43,dataWord10,9,22);

  return true;
}
//---------------------------------------------------------------------------
std::string
SubFrame3::debugDump()
{
  std::stringstream result;
  result << SubFrameBase::debugDump();
  
  M_DUMP_VALUE(C_ic_);
  M_DUMP_VALUE(omega_0_);
  M_DUMP_VALUE(C_is_);
  M_DUMP_VALUE(i_0_);
  M_DUMP_VALUE(C_rc_);
  M_DUMP_VALUE(w_);
  M_DUMP_VALUE(dot_omega_);
  M_DUMP_VALUE(sf3IODE_);
  M_DUMP_VALUE(IDOT_);

  return result.str();
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//Ephemeris::
//---------------------------------------------------------------------------
Ephemeris::Ephemeris()
: ephemerisValid_(false)
{
}
//---------------------------------------------------------------------------
void
Ephemeris::reset()
{
  ephemerisValid_ = false;
}
//---------------------------------------------------------------------------
bool
Ephemeris::ephemerisValid()
const
{
  return ephemerisValid_;
}
//---------------------------------------------------------------------------
void
Ephemeris::processSubFrameRaw(
  SubFrameRaw const&       inSubFrameRaw)
{
  s32 id;
  s32 page;
  if(SubFrameIDGet(inSubFrameRaw,id,page))
  {
    switch(id)
    {
      case 1:                    
        preliminarySF1_.parse(inSubFrameRaw);
        preliminarySubFramesProcess();
        break;
      case 2:
        preliminarySF2_.parse(inSubFrameRaw);
        preliminarySubFramesProcess();
        break;
      case 3:
        preliminarySF3_.parse(inSubFrameRaw);
        preliminarySubFramesProcess();
        break;
    }
  }
}  
//---------------------------------------------------------------------------
void
Ephemeris::debugLoadCAMsg(
  c8 const* const          inFileName)
{
  std::fstream f(inFileName,std::ios::in);
  
  M_KK_ASSERT(f.is_open());
  
  SubFrameRaw sfraw;
  u32 word = 0;
  u32 bitIndex = 0;
  for(;;)
  {
    c8 c;
    f.get(c);
    if(!f.good()) break;

    switch(c)
    {
      case '0':
      case 0:
        ++bitIndex;
        word <<= 1;
        break;
      case '1':
      case 1:
        ++bitIndex;
        word <<= 1;
        word |= 1;
        break;
      default:
        break;
    }
    
    if(0 == (bitIndex % kMsgBitsPerWord))
    { //add word to buffer
      sfraw.words_[bitIndex / kMsgBitsPerWord - 1] = word;      
    }
    
    if(0 == (bitIndex % (kMsgBitsPerWord * kMsgWordsPerSubFrame)))
    { //process subframe
      processSubFrameRaw(sfraw);
      bitIndex = 0;
    }
  }
}
//---------------------------------------------------------------------------
void
Ephemeris::calculateDeltaSVTimeAndEkFromGPSTime(
  f64 const                inT_GPS,
  f64&                     ouDeltaT_sv,
  f64&                     ouE_k)
const
{
  f64 const t = inT_GPS;
 
  f64 const F = -4.442807633e-10; //-2 * sqrt(k_mu) / (k_c * k_c);
  
  f64 const A = sqrtA_ * sqrtA_;
  
  f64 const n_0 = std::sqrt(k_mu / (A * A * A));
  
  f64 const n = n_0 + deltan_ * k_semi_circle_radians;
  
  f64 const t_k = limitTimeToValidRange(t - t_oe_);
  
  f64 const M_k = M_0_ * k_semi_circle_radians + n * t_k;
  
  //E_k is calculated iteratively
  //GPS standard says that 10 iteration is enough 
  f64 E_k = M_k;
  for(;;) 
  {
    f64 const E_k_last = E_k;
    E_k = M_k + e_ * std::sin(E_k);
    if(std::fabs(static_cast<f64>(E_k_last) - static_cast<f64>(E_k)) < 1e-14) break;
//debug    std::cerr 
//debug      << "E_k Iter: " << static_cast<f64>(E_k_last) - static_cast<f64>(E_k)
//debug      << " " << E_k
//debug      << "\n";
  }
//debug  std::cerr << "E_k Done\n";
  
  f64 const delta_t_r = F * e_ * sqrtA_ * std::sin(E_k);
  
  f64 const dt = limitTimeToValidRange(t - t_oc_); 

  f64 const delta_t_sv = a_f0_ + a_f1_ * dt + a_f2_ * dt * dt + delta_t_r;

  f64 const delta_t_sv_l1 = delta_t_sv - T_GD_;

  ouDeltaT_sv = delta_t_sv_l1; 
  ouE_k = E_k;
}
//---------------------------------------------------------------------------
FourDPos
Ephemeris::findPos(
  f64 const                inTtr_SV)
const
{
  FourDPos pos;
    
  //find Ttr_GPS
  //long double is used because of x86 FP registers having 80bits of precision
  //while in memory FP are stored with 64bits
  f64 lastDt = 0.0;
  f64 Ttr_GPS = inTtr_SV;
  f64 E_k = 0;
  for(;;)
  {
    f64 dt;
    calculateDeltaSVTimeAndEkFromGPSTime(Ttr_GPS,dt,E_k);

    Ttr_GPS = inTtr_SV - dt;

//debug    std::cerr 
//debug      << "Ttr_GPS Iter:"
//debug      << " " << lastDt - dt
//debug      << " " << static_cast<f64>(lastDt) - static_cast<f64>(dt)
//debug      << " " << static_cast<f64 volatile>(lastDt) - static_cast<f64 volatile>(dt)
//debug      << " " << Ttr_GPS
//debug      << "\n";
    if(std::fabs(lastDt - dt) == 0.0) break;
    lastDt = dt;
  }
  pos.t = Ttr_GPS;
//debug  std::cerr << "Ttr_GPS Done\n";
  
  //calculate Satelite position @ time Ttr_GPS
  f64 const t_k = limitTimeToValidRange(Ttr_GPS - t_oe_);

  //true anomaly
  f64 const cos_E_k = std::cos(E_k);
  f64 const sin_E_k = std::sin(E_k);
  f64 const v_k = std::atan2(
    std::sqrt(1.0 - e_ * e_) * sin_E_k,
    cos_E_k - e_);
  
  //argument of latitude
  f64 const phi_k = v_k + w_ * k_semi_circle_radians;

  //second harmonic perturbations
  f64 const sin_2phi_k = std::sin(2 * phi_k);
  f64 const cos_2phi_k = std::cos(2 * phi_k);
  f64 const delta_u_k = C_us_ * sin_2phi_k + C_uc_ * cos_2phi_k; //argument of latitude correction
  f64 const delta_r_k = C_rs_ * sin_2phi_k + C_rc_ * cos_2phi_k; //radius correction
  f64 const delta_i_k = C_is_ * sin_2phi_k + C_ic_ * cos_2phi_k; //inclination correction

  //corrected argument of latitude
  f64 const u_k = phi_k + delta_u_k;
  
  //corrected radius
  f64 const r_k = sqrtA_ * sqrtA_ * (1 - e_ * cos_E_k) + delta_r_k;
  
  //corrected inclination
  f64 const i_k = 
    i_0_ * k_semi_circle_radians + 
    delta_i_k + 
    IDOT_ * k_semi_circle_radians * t_k;

  //position in orbital plane
  f64 const xp = r_k * std::cos(u_k);
  f64 const yp = r_k * std::sin(u_k);

  //corrected longiture of ascending node
  f64 const omega_k = 
    omega_0_ * k_semi_circle_radians + 
    (dot_omega_ * k_semi_circle_radians - k_dot_omega_e) * t_k - 
    k_dot_omega_e * t_oe_;
  
  //convert to earth fixed coordinates
  f64 const cos_omega_k = std::cos(omega_k);
  f64 const sin_omega_k = std::sin(omega_k);
  f64 const yp_cos_i_k = yp * std::cos(i_k);
  
  pos.x = xp * cos_omega_k - yp_cos_i_k * sin_omega_k;
  pos.y = xp * sin_omega_k + yp_cos_i_k * cos_omega_k;
  pos.z = yp * std::sin(i_k);

  return pos;
}
//---------------------------------------------------------------------------
void
Ephemeris::preliminarySubFramesProcess()
{
  SubFrame1 const& psf1 = preliminarySF1_;
  SubFrame2 const& psf2 = preliminarySF2_;
  SubFrame3 const& psf3 = preliminarySF3_;
  
  //check if subframes 1 & 2 & 3 match
  if(((psf1.IODC_ & 0xff) == (psf2.sf2IODE_)) &&
     ((psf1.IODC_ & 0xff) == (psf3.sf3IODE_)))
  {
    static_cast<SubFrame1Data&>(*this) = psf1;
    static_cast<SubFrame2Data&>(*this) = psf2;
    static_cast<SubFrame3Data&>(*this) = psf3;
    ephemerisValid_ = true;
  }
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//ChannelSetup::
//---------------------------------------------------------------------------
ChannelSetup::ChannelSetup(
  f64 const                inSampleRate,
  f64 const                inCarrierFreq,
  f64 const                inChipRate,
  u32 const                inPrnNumber)
: sampleRate_(inSampleRate)
, carrierFreq_(inCarrierFreq)
, chipRate_(inChipRate)
, prnNumber_(inPrnNumber)
{
}
//---------------------------------------------------------------------------
ChannelSetup::ChannelSetup(
  ChannelSetup const&      inOther)
{
  *this = inOther;
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//Channel::
//---------------------------------------------------------------------------
Channel::Channel(
  f64 const                inSampleRate,
  f64 const                inCarrierFreq,
  f64 const                inChipRate,
  u32 const                inPrnNumber)
: signalTrack_(inSampleRate,inCarrierFreq,inChipRate,inPrnNumber)
, ephemeris_()
, lastSatTtr_(0.0)
{
}
//---------------------------------------------------------------------------
Channel::Channel(
  ChannelSetup const&      inSetup)
: signalTrack_(
   inSetup.sampleRate_,inSetup.carrierFreq_,
   inSetup.chipRate_,inSetup.prnNumber_)
, ephemeris_()
, lastSatTtr_()
{
}
//---------------------------------------------------------------------------
void
Channel::processData(
  f32 const*                inBegin,
  f32 const* const          inEnd,
  GPSCorrelatorMsgTrack::DoTrackDumpCallback const inCallback)
{
  //process block of data
  while(inBegin != inEnd)
  {
    GPSCorrelatorMsgTrack::DoTrackResult const result =
      signalTrack_.doTrack(inBegin,inEnd,inCallback);

    if(GPSCorrelatorMsgTrack::trMsgSubFrame == result)
      ephemeris_.processSubFrameRaw(signalTrack_.subFrameRaw_);
  }
}
//---------------------------------------------------------------------------
void
Channel::reset()
{  
  signalTrack_.reset();
  ephemeris_.reset();
  lastSatTtr_ = 0.0;
}
//---------------------------------------------------------------------------
void
Channel::reset(
  u32 const                inPrnNumber)
{
  signalTrack_.reset();
  ephemeris_.reset();
  lastSatTtr_ = 0.0;

  signalTrack_.prnNumberSet(inPrnNumber);
}
//---------------------------------------------------------------------------
void
Channel::start()
{
  signalTrack_.start();
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
void LeastSqFit(
  f64*                       ouDX,
  FourDPos&                  inPosition,
  std::vector<f64>&	         inSatX,
  std::vector<f64>&	         inSatY,
  std::vector<f64>&	         inSatZ,
  std::vector<f64>&	         inPRN)
{
  //solve interative least squares equations
  //dX = (H^T * H)^-1 * H^T * dP
  f64 HtH[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  f64 HtdP[4] = {0,0,0,0};
  for(u32 i = 0; i < inSatX.size(); ++i)
  {
    f64 const dx = inPosition.x - inSatX[i];
    f64 const dy = inPosition.y - inSatY[i];
    f64 const dz = inPosition.z - inSatZ[i];
    f64 const r  = std::sqrt(dx*dx + dy*dy + dz*dz);
    f64 const dp = inPRN[i] * k_c - r;
    
    HtH[ 0] += dx * dx / (r * r);
    HtH[ 1] += dx * dy / (r * r);
    HtH[ 2] += dx * dz / (r * r);
    HtH[ 3] += dx / r;
    HtH[ 4] += dy * dx / (r * r);
    HtH[ 5] += dy * dy / (r * r);
    HtH[ 6] += dy * dz / (r * r);
    HtH[ 7] += dy / r;
    HtH[ 8] += dz * dx / (r * r);
    HtH[ 9] += dz * dy / (r * r);
    HtH[10] += dz * dz / (r * r);
    HtH[11] += dz / r;
    HtH[12] += dx / r;
    HtH[13] += dy / r;
    HtH[14] += dz / r;
    HtH[15] += 1;
    
    HtdP[0] += dp * dx / r;
    HtdP[1] += dp * dy / r;
    HtdP[2] += dp * dz / r;
    HtdP[3] += dp;
  }
  
  //HtHi = HtH^-1;
  f64 HtHi[16];
  
  Invert4x4Matrix(HtH,HtHi);
  
  for(u32 i = 0; i < 4; ++i) ouDX[i] = 0;
  
  for(u32 i = 0; i < 4; ++i)
  {
    ouDX[0] += HtHi[0*4 + i] * HtdP[i];
    ouDX[1] += HtHi[1*4 + i] * HtdP[i];
    ouDX[2] += HtHi[2*4 + i] * HtdP[i];
    ouDX[3] += HtHi[3*4 + i] * HtdP[i];
  }
}           
//---------------------------------------------------------------------------
bool findPosition(
  FourDPos&                  ioPosition,
  ChannelVector const&       inChannels)
{
  //find average Ttr(GPS)
  f64 satMaxTtx = 0.0;
  std::vector<u32> validSat;
  std::vector<FourDPos> satPos;

  //compute all valid satellite positions/times and average Ttx(GPS) time
  for(u32 i = 0; i < inChannels.size(); ++i)
    if(inChannels[i].ephemeris_.ephemerisValid())
    {
      validSat.push_back(i);
      satPos.push_back(
        inChannels[i].ephemeris_.findPos(inChannels[i].lastSatTtr_));
      satMaxTtx = std::max(satMaxTtx,satPos.back().t);
    }


  //some valid satelites  
  if(satPos.size() >= 4)
  {
    //add in average expected transit time
    satMaxTtx += k_satellite_orbital_radius_m / k_c; 
 
    //if user time is too far away from the estimated receive time
    //then reset user time to max satellite transmit time + estimated 
    //signal travel time
    if(std::fabs(satMaxTtx - ioPosition.t) > 1.0) ioPosition.t = satMaxTtx;
       
    std::vector<f64> satX(satPos.size());
    std::vector<f64> satY(satPos.size());
    std::vector<f64> satZ(satPos.size());
    std::vector<f64> PRN(satPos.size());
    for(;;)
    {
      for(u32 i = 0; i < satPos.size(); ++i)
      {
        f64 const x = satPos[i].x;
        f64 const y = satPos[i].y;
        f64 const z = satPos[i].z;
        f64 const t = satPos[i].t;

        //compute PRNs based on user and satellite times
        PRN[i] = ioPosition.t - t;
      
        //update PRNs for atmosphere 
        //Channel const& ch = inChannels[validSat[i]];
        //xxx

        //compute all ECI positions to userTime reference
        //section 20.3.3.4.3.3.2 GPS standard
        f64 const theta = k_dot_omega_e * (t - ioPosition.t);
        f64 const ct = std::cos(theta);
        f64 const st = std::sin(theta);
        satX[i] = x * ct - y * st;
        satY[i] = x * st + y * ct;
        satZ[i] = z;
      }
      
      f64 dX[4] = {0,0,0,0};
      LeastSqFit(dX,ioPosition,satX,satY,satZ,PRN);

      //update user position and time
      ioPosition.x += dX[0];
      ioPosition.y += dX[1];
      ioPosition.z += dX[2];
      ioPosition.t -= dX[3] / k_c;
      
      //if deltas small enough break out of loop
      f64 const posErr = std::sqrt(dX[0] * dX[0] + dX[1] * dX[1] + dX[2] * dX[2]);
      f64 const timeErr = dX[3] / k_c;  
      if((posErr < 1e-6) && (std::fabs(timeErr) < 1e-9)) break;
    }
    return true;    
  }
  else
  {
  	return false;
  }
}
//---------------------------------------------------------------------------
