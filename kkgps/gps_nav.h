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
#ifndef gps_navH
#define gps_navH
//---------------------------------------------------------------------------
#include "gps_msg.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
void 
Invert4x4Matrix( f64 *mat, f64 *dst);
//---------------------------------------------------------------------------

//all reference to sections is to the GPS standard ICD-GPS-200C 10/OCT/1993
/*
 * msg word bits are label as follows
 * MSb                          LSb
 * XX000000000111111111122222222223
 * XX123456789012345678901234567890
 * XXDataDataDataDataDataDataParity
 *
 * msg data bits are label as follows
 * MSb                          LSb
 * XXXXXXXX000000000111111111122222
 * XXXXXXXX123456789012345678901234
 * XXXXXXXXDataDataDataDataDataData
 */
//---------------------------------------------------------------------------
//desc: Will parse a GPS nav message and extract a value from 1 or 2
//      words. Bit addresses are in GPS Notation GPS bit 1 is normal 
//      bit 23 (base 0) and GPS bit 24 is normal bit 0 (base 0)
//---------------------------------------------------------------------------
s32
parseMsgDataS32(
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord = 0,
  u32 const                  inLsbFirstBit = 0,
  u32 const                  inLsbLastBit = 0);
//---------------------------------------------------------------------------
u32
parseMsgDataU32(
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord = 0,
  u32 const                  inLsbFirstBit = 0,
  u32 const                  inLsbLastBit = 0);
//---------------------------------------------------------------------------
f64
parseMsgDataSF64(
  s32 const                  inScale,
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord = 0,
  u32 const                  inLsbFirstBit = 0,
  u32 const                  inLsbLastBit = 0);
//---------------------------------------------------------------------------
f64
parseMsgDataUF64(
  s32 const                  inScale,
  u32 const                  inMsbWord,
  u32 const                  inMsbFirstBit,
  u32 const                  inMsbLastBit,
  u32 const                  inLsbWord = 0,
  u32 const                  inLsbFirstBit = 0,
  u32 const                  inLsbLastBit = 0);
//---------------------------------------------------------------------------
bool
SubFrameIDGet(
  SubFrameRaw const&         inSubFrame,
  s32&                       ouSubFrameID,
  s32&                       ouPage);
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
SubFrameBase
{
public:
  virtual
  bool
  parse(
    SubFrameRaw const&       inSubFrame);
    
  virtual
  std::string
  debugDump();

//sub-frame data
//preamble                   //word 1 : bit  1 - 8
  u32    telemetryMessage_;  //word 1 : bit  9 - 22
//reserved                   //word 1 : bit 23 - 24  
  u32    truncatedTowCount_; //word 2 : bit  1 - 17
  bool   alertFlag_;         //word 2 : bit 18
  bool   antiSpoofFlag_;     //word 2 : bit 19
  u32    subFrameId_;        //word 2 : bit 20 - 22
//parity adjustment          //word 2 : bit 23 - 24  

};
//---------------------------------------------------------------------------

struct
SubFrame1Data
{
//sub-frame data
  u32    gpsWeekNumber_;      //word  3 : bit  1 - 10
  u32    codesOnL2_;          //word  3 : bit 11 - 12
  u32    svAccuracy_;         //word  3 : bit 13 - 16 //section 20.3.3.3.1.3
  bool   svHealthSummary_;    //word  3 : bit 17
  u32    svHealth_;           //word  3 : bit 18 - 22
                             //IODC  //section 20.3.3.3.1.5
  u32    IODC_;               //word  3 : bit 23 - 24 MSbs
                             //word  8 : bit  1 -  8 LSbs
  bool   pCodeOffOnL2_;       //word  4 : bit  1
//reserved                   //word  4 : bit  2 - 24  
//reserved                   //word  5 : bit  1 - 24  
//reserved                   //word  6 : bit  1 - 24  
//reserved                   //word  7 : bit  1 - 16
  f64    T_GD_;               //word  7 : bit 17 - 24 //scale 2^-31
//IODC LSb                   //word  8 : bit  1 -  8
  u32    t_oc_;               //word  8 : bit  9 - 24 //scale 2^4
  f64    a_f2_;               //word  9 : bit  1 -  8 //scale 2^-55
  f64    a_f1_;               //word  9 : bit  9 - 24 //scale 2^-43
  f64    a_f0_;               //word 10 : bit  1 - 22 //scale 2^-31
//parity adjustment          //word 10 : bit 23 - 24  
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
struct
SubFrame2Data
{
//sub-frame data
  u32    sf2IODE_;            //word  3 : bit  1 -  8        
  f64    C_rs_;               //word  3 : bit  9 - 24        //scale 2^-5
  f64    deltan_;             //word  4 : bit  1 - 16        //scale 2^-43
  f64    M_0_;                //word  4 : bit 17 - 24 //MSbs //scale 2^-31
                             //word  5 : bit  1 - 24 //LSbs
  f64    C_uc_;               //word  6 : bit  1 - 16        //scale 2^-29 
  f64    e_;                  //word  6 : bit 17 - 24 //MSbs //scale 2^-33
                             //word  7 : bit  1 - 24 //LSbs 
  f64    C_us_;               //word  8 : bit  1 - 16        //scale 2^-29
  f64    sqrtA_;              //word  8 : bit 17 - 24 //MSbs //scale 2^-19
                             //word  9 : bit  1 - 24 //LSbs
  f64    t_oe_;               //word 10 : bit  1 - 16        //scale 2^4
  bool   fitIntervalFlag_;    //word 10 : bit 17
  u32    AODO_;               //word 10 : bit 18 - 22
//parity adjustment          //word 10 : bit 23 - 24  
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
struct
SubFrame3Data
{
//sub-frame data
  f64    C_ic_;               //word  3 : bit  1 - 16        //scale 2^-29
  f64    omega_0_;            //word  3 : bit 17 - 24 //MSbs //scale 2^-31
                              //word  4 : bit  1 - 24 //LSbs
  f64    C_is_;               //word  5 : bit  1 - 16        //scale 2^-29
  f64    i_0_;                //word  5 : bit 17 - 24 //MSbs //scale 2^-31
                              //word  6 : bit  1 - 24 //LSbs
  f64    C_rc_;               //word  7 : bit  1 - 16        //scale 2^-5
  f64    w_;                  //word  7 : bit 17 - 24 //MSbs //scale 2^-31
                              //word  8 : bit  1 - 24 //LSbs
  f64    dot_omega_;          //word  9 : bit  1 - 24        //scale 2^-43
  u32    sf3IODE_;            //word 10 : bit  1 -  8
  f64    IDOT_;               //word 10 : bit  9 - 22        //scale 2^-43
//parity adjustment           //word 10 : bit 23 - 24  
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
SubFrame1
: public SubFrameBase
, public SubFrame1Data
{
public:

  virtual
  bool
  parse(
    SubFrameRaw const&       inSubFrame);
    
  virtual
  std::string
  debugDump();
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
SubFrame2
: public SubFrameBase
, public SubFrame2Data
{
public:

  virtual
  bool
  parse(
    SubFrameRaw const&       inSubFrame);
    
  virtual
  std::string
  debugDump();
    
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
SubFrame3
: public SubFrameBase
, public SubFrame3Data
{
public:

  virtual
  bool
  parse(
    SubFrameRaw const&       inSubFrame);
    
  virtual
  std::string
  debugDump();
  
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class FourDPos 
{ 
public:
  FourDPos()
  : x(0.0)
  , y(0.0)
  , z(0.0)
  , t(0.0)
  {
  }
  
  FourDPos(
    f64 const                inX,
    f64 const                inY,
    f64 const                inZ,
    f64 const                inT)    
  : x(inX)
  , y(inY)
  , z(inZ)
  , t(inT)
  {
  }

  
  f64 x;
  f64 y;
  f64 z;
  f64 t;
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
Ephemeris
: public SubFrame1Data
, public SubFrame2Data
, public SubFrame3Data
{
public:
  Ephemeris();

  void
  reset();

  bool
  ephemerisValid()
  const;
  
  void
  processSubFrameRaw(
    SubFrameRaw const&       inSubFrameRaw);  
    
  void
  debugLoadCAMsg(
    c8 const* const          inFileName);
    
  void 
  calculateDeltaSVTimeAndEkFromGPSTime(
    f64 const                inT_GPS,
    f64&                     ouDeltaT_sv,
    f64&                     ouE_k)
  const;
    
  FourDPos
  findPos(
    f64 const                inTtr_SV)
  const;
  
protected:
  bool                       ephemerisValid_;
  SubFrame1                  preliminarySF1_;
  SubFrame2                  preliminarySF2_;
  SubFrame3                  preliminarySF3_;
  
  void
  preliminarySubFramesProcess();
};
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
struct
ChannelSetup
{
  ChannelSetup(
    f64 const                inSampleRate,
    f64 const                inCarrierFreq,
    f64 const                inChipRate,
    u32 const                inPrnNumber);
    
  ChannelSetup(
    ChannelSetup const&      inOther);

  f64                        sampleRate_;
  f64                        carrierFreq_;
  f64                        chipRate_;
  u32                        prnNumber_;
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
Channel
{
public:
  Channel(
    f64 const                inSampleRate,
    f64 const                inCarrierFreq,
    f64 const                inChipRate,
    u32 const                inPrnNumber);
    
  Channel(
    ChannelSetup const&      inSetup);
    
  void
  processData(
    f32 const*               inBegin,
    f32 const* const         inEnd,
    GPSCorrelatorMsgTrack::DoTrackDumpCallback const inCallback = 0);
        
  void
  reset();

  void
  reset(
    u32 const                inPrnNumber);
    
  void
  start();
    

  GPSCorrelatorMsgTrack      signalTrack_;
  Ephemeris                  ephemeris_;
  f64                        lastSatTtr_;
};
//---------------------------------------------------------------------------
typedef std::vector<Channel> ChannelVector;
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
void LeastSqFit(
  f64*                       ouDX,
  FourDPos&                  inPosition,
  std::vector<f64>&	         inSatX,
  std::vector<f64>&	         inSatY,
  std::vector<f64>&	         inSatZ,
  std::vector<f64>&	         inPRN);
//---------------------------------------------------------------------------
bool findPosition(
  FourDPos&                  ioPosition,
  ChannelVector const&       inChannels);
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
#endif
