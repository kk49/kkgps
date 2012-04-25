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
#ifndef kkutilsH
#define kkutilsH
//---------------------------------------------------------------------------
#include <complex>
#include <exception>
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
typedef char c8;
typedef float f32;
typedef double f64;
typedef signed char s8;
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;
typedef long s32;
typedef unsigned long u32;
typedef signed long long s64;
typedef unsigned long long u64;
typedef std::complex<f64> cf64;
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class
KKException
: public std::exception
{
public:
  KKException(
    std::string const&       inStr)
  : whatString_(inStr)
  {
  }
  
  virtual
  ~KKException()
  throw()
  {
  }
  
  c8 const*
  what()
  const
  throw()
  {
    return whatString_.c_str();
  }
  
protected:
  std::string                whatString_;
};
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
#define M_KK_ASSERT(COND_) do {if(!(COND_)) throw KKException("ASSERT FAILED: "#COND_);} while(false)
//---------------------------------------------------------------------------

#endif
