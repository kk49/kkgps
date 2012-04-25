//---------------------------------------------------------------------------
//Copyright (C) 2004 Krzysztof Kamieniecki (krys@kamieniecki.com)
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
#include "kkutils.h"
#include <deque>
#include "gps_nav.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class GPS_Receiver
{ 
public:
  typedef std::deque<u32> u32Deque;

  GPS_Receiver(
    u32 const                inChannelCount,
    ChannelSetup const&      inChannelSetup);
  
  virtual
  ~GPS_Receiver();
  
  void
  processData(
    f32 const*               inBegin,
    f32 const* const         inEnd);
    
protected:
  u64                        sampleIndex_;
  FourDPos                   userPosition_;
  ChannelSetup               channelSetup_;  
  ChannelVector              chs_;
  u32Deque                   inactivePrns_;
  
};
//---------------------------------------------------------------------------
