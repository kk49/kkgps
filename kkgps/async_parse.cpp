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
#include <cmath>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>

#include "kkutils.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  try
  {
    if(argc > 2)
    {
      std::fstream f(argv[1],std::ios::in | std::ios::binary);
      if(!f.is_open()) throw KKException("Could Not Open File");
      
      u32 satId;
      std::stringstream ss(argv[2]);
      ss >> satId;
  
      u32 lastSeq = 0xffffffff;
      for(;;)
      {
        u8 id = 0;      
        u8 length = 0;
        u8 buf[256];
        
        f.read(reinterpret_cast<char*>(&id),1);
        f.read(reinterpret_cast<char*>(&length),1);
        if(length)
        {
          f.read(reinterpret_cast<char*>(buf),length);
          buf[static_cast<int>(length)] = '\0';
        }
        
        if(f.eof()) break;
        
        if(0x36 == id)
        {
          u32 const seq = 
            buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0];
          u32 const msg = 
            buf[7] << 24 | buf[6] << 16 | buf[5] << 8 | buf[4];
          u32 const svid = 
            static_cast<u32>(buf[8]) + 1;


          if(svid == satId)
          {
            
            if((lastSeq != 0xffffffff) && ((seq - lastSeq) > 30))
            {
              for(u32 i = 0; i < (seq - lastSeq - 30); ++i)
                std::cout << 'X';
            }
            
            if((lastSeq == 0xffffffff) && (0 != ((seq + 1500 - 30) % 1500))) 
            {
              for(u32 i = 0; i < ((seq + 1500 - 30) % 1500); ++i)
                std::cout << 'x';
            }
           
            for(s32 i = 29; i >= 0; --i)
              std::cout << (1 & (msg >> i))?(c8)(1):(c8)(0);

            lastSeq = seq;
            
            //if we just precess the last word in a frame then start new line
            if(0 == ((seq) % 1500)) std::cout << std::endl;
          }
        }
      }
    }
    else
    {
      std::cerr << "Usage: async_parse <input file> <satelite id>";
    }
  }
  catch(std::exception const& E)
  {
    std::cerr << E.what();
  }
  return 0;
}
//---------------------------------------------------------------------------
