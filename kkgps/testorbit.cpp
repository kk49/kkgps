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
#include <cassert>
#include <ctime>
#include <fstream>

#include "gps_corr.h"
#include "gps_msg.h"
#include "gps_nav.h"
//---------------------------------------------------------------------------
s32 const kSecondsPerDay = 24 * 60 * 60;

//---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  try
  { 
    Ephemeris ephemeris;

    ephemeris.debugLoadCAMsg("273673_09.txt");
//    ephemeris.debugLoadCAMsg("273673_21.txt");
//    ephemeris.debugLoadCAMsg("273673_22.txt");
//    ephemeris.debugLoadCAMsg("273673_23.txt");
//    ephemeris.debugLoadCAMsg("273673_26.txt");
//    ephemeris.debugLoadCAMsg("273673_29.txt");
/*
    static const f64 k_pi = 3.1415926535898;

//based on orbit from tebinuma@yahoo.com
    ephemeris.a_f0_           = 2.356632612646E-04;
    ephemeris.a_f1_           = 1.932676241267E-12;
    ephemeris.a_f2_           = 0.000000000000E+00;
    ephemeris.sf3IODE_        = ephemeris.sf2IODE_ = 212;
    ephemeris.C_rs_           = -9.409375000000E+01;
    ephemeris.deltan_         = 4.070169538899E-09 / k_pi;
    ephemeris.M_0_            = 2.056614701229E+00 / k_pi;
    ephemeris.C_uc_           = -4.839152097702E-06 / k_pi;
    ephemeris.e_              = 5.168720614165E-03;
    ephemeris.C_us_           = 1.081451773643E-05 / k_pi;
    ephemeris.sqrtA_          = 5.153628406525E+03;
    ephemeris.t_oe_           = 8.640000000000E+04;
    ephemeris.C_ic_           = 1.005828380585E-07 / k_pi; 
    ephemeris.omega_0_        = 2.979421253197E+00 / k_pi;
    ephemeris.C_is_           = -1.080334186554E-07 / k_pi;    
    ephemeris.i_0_            = 9.698482115566E-01 / k_pi;
    ephemeris.C_rc_           = 1.797187500000E+02;
    ephemeris.w_              = -1.694442176021E+00 / k_pi;
    ephemeris.dot_omega_      = -7.571029649489E-09 / k_pi;   
    ephemeris.IDOT_           = 3.053698627377E-10 / k_pi;
    ephemeris.T_GD_           = -3.259629011154E-09;
    ephemeris.IODC_           = 468;
//    ephemeris.CODES on L2 = 0.000000000000E+00;
//    ephemeris.GPSWeek = 1.172000000000E+03;
//    ephemeris.L2Pdata = 0.000000000000E+00;
//    ephemeris.SVAcc = 4.000000000000E+00;
//    ephemeris.SVHealth = 0.000000000000E+00;
   
//    Transmissitib time 8.637000000000D+04
//    spare 4.000000000000D+00
*/    
    //dump satellite orbit
//    if(ephemeris.ephemerisValid())
    {
      std::cout.precision(10); 
      for(s32 j = (-kSecondsPerDay/2); j < (kSecondsPerDay/2); j += 60)
      {
        FourDPos p = ephemeris.findPos(ephemeris.t_oe_ + j);
        std::cout 
          << p.t << "\t" << p.x << "\t" << p.y << "\t" << p.z << std::endl;
      }
    }
  }
  catch(std::exception const& E)
  {
    std::cerr << "\nERROR: " << E.what() << std::endl;
      return 1;
  }
  return 0;
}
