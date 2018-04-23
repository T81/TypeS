// Type B Thermocouple library per ITS-90

// *** BSD License ***
// ------------------------------------------------------------------------------------------
//
// Author: T81
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this list of 
// conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice, this list 
// of conditions and the following disclaimer in the documentation and/or other materials 
// provided with the distribution.
//
// Neither the name of the MLG Properties, LLC nor the names of its contributors may be 
// used to endorse or promote products derived from this software without specific prior 
// written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// Original Author of Type K thermocouples library: Jim Gallt
// ------------------------------------------------------------------------------------------

#include "TypeS.h"
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif


// -------------------------------------
const int TypeS::nranges_inv = 4;  // number of mV ranges for inverse lookup
const int TypeS::ncoeff_inv = 10;  // number of coefficients for inverse lookup
const float TypeS::mv_min = -0.235;
const float TypeS::mv_max = 18.693;

// coefficients for inverse lookup (given mV, find C)
const double TypeS::coeff_inv[10][4] = {
	{  0.00000000E+00,  1.291507177E+01, -8.087801117E+01,  5.333875126E+04 },
	{  1.84949460E+02,  1.466298863E+02,  1.621573104E+02, -1.235892298E+04 },
	{ -8.00504062E+01, -1.534713402E+01, -8.536869453E+00,  1.092657613E+03 },
	{  1.02237430E+02,  3.145945973E+00,  4.719686976E-01, -4.265693686E+01 },
	{ -1.52248592E+02, -4.163257839E-01, -1.441693666E-02,  6.247205420E-01 },
	{  1.88821343E+02,  3.187963771E-02,  2.081618890E-04,  0.000000000E+00 },
	{ -1.59085941E+02, -1.291637500E-03,  0.000000000E+00,  0.000000000E+00 },
	{  8.23027880E+01,  2.183475087E-05,  0.000000000E+00,  0.000000000E+00 },
	{ -2.34181944E+01, -1.447379511E-07,  0.000000000E+00,  0.000000000E+00 },
	{  2.79786260E+00,  8.211272125E-09,  0.000000000E+00,  0.000000000E+00 }
};

// mV ranges for inverse lookup coefficients
const float TypeS::range_inv[2][4] = {
  { -0.235,  1.874, 10.332, 17.536 },
  {  1.874, 11.950, 17.536, 18.693 }
};

// coefficients for direct lookup (given C, find mV)
const double TypeS::coeff_dir[9][3] = {
	{  0.000000000000E+00,  0.132900444085E+01,  0.146628232636E+03 },
	{  0.540313308631E-02,  0.334509311344E-02, -0.258430516752E+00 },
	{  0.125934289740E-04,  0.654805192818E-05,  0.163693574641E-03 },
	{ -0.232477968689E-07, -0.164856259209E-08, -0.330439046987E-07 },
	{  0.322028823036E-10,  0.129989605174E-13, -0.943223690612E-14 },
	{ -0.331465196389E-13,  0.000000000000E+00,  0.000000000000E+00 },
	{  0.255744251786E-16,  0.000000000000E+00,  0.000000000000E+00 },
	{ -0.125068871393E-19,  0.000000000000E+00,  0.000000000000E+00 },
	{  0.271443176145E-23,  0.000000000000E+00,  0.000000000000E+00 }
};

// ranges for direct lookup
const double TypeS::range_dir[3][2] = {
  { -50.000 , 1064.180 },
  { 1064.180, 1664.500 },
  { 1664.500, 1768.100 }
};

const float TypeS::C_max = 1768.100;
const float TypeS::C_min = -50.000;

// -------------------------- constructor
TypeS::TypeS() {
  F_max = C_TO_F( C_max );
  F_min = C_TO_F( C_min );
}

// ------------------- given mv reading, returns absolute temp C
double TypeS::Temp_C( float mv ) {
  double x = 1.0;
  double sum = 0.0;
  int i,j,ind;
  ind = 0;
  if ( ! inrange_mV( mv ) ) return TC_RANGE_ERR;
  // first figure out which range of values
  for( j = 0; j < nranges_inv; j++ ) {
    if((mv >= range_inv[0][j]) && (mv <= range_inv[1][j]))
      ind = j;
  };
//  Serial.println(ind);
  for( i = 0; i < ncoeff_inv; i++ ) {
    sum += x * coeff_inv[i][ind];
    x *= mv;
  }
  return sum;  
}

// --------- given mv reading and ambient temp, returns compensated (true)
//           temperature at tip of sensor
double TypeS::Temp_C( float mv, float amb ) {
  float mv_amb;
  mv_amb = mV_C( amb );
  return Temp_C( mv + mv_amb );
};

// --------------------- returns compensated temperature in F units
double TypeS::Temp_F( float mv, float amb ) {
  return C_TO_F( Temp_C( mv, F_TO_C( amb ) ) );
};

// --------------------- returns absolute temperature in F units
double TypeS::Temp_F( float mv ) {
  float temp = Temp_C( mv );
  if( temp == TC_RANGE_ERR ) return TC_RANGE_ERR;
  return C_TO_F( temp );  
}

// --------------------- checks to make sure mv signal in range
boolean TypeS::inrange_mV( float mv ) {
  return ( mv >= mv_min ) & ( mv <= mv_max );
};

// ---------------------- checks to make sure temperature in range
boolean TypeS::inrange_C( float ambC ) {
  return ( ambC >= C_min ) & ( ambC <= C_max );
};

// ----------------------- checks to make sure temperature in range
boolean TypeS::inrange_F( float ambF ) {
  return ( ambF >= F_min ) & ( ambF <= F_max );
};

// ---------------- returns mV corresponding to temp reading
//                  used for cold junction compensation
double TypeS::mV_C( float ambC ) {
  double sum = 0.0;
  double x = 1.0;
  int i;
  if( !inrange_C( ambC ) ) return TC_RANGE_ERR;

  if ( (ambC >= range_dir[0][0]) && ( ambC <= range_dir[1][0] ) )  {
    for( i = 0; i < 9; i++ ) {
      sum += x * coeff_dir[i][0];
      x *= ambC;
    } 
  }
  else if ( (ambC >= range_dir[1][0]) && ( ambC <= range_dir[2][0] ) )  {
    for( i = 0; i < 5; i++ ) {
      sum += x * coeff_dir[i][1];
      x *= ambC;
    } 
  }
  else {
    for( i = 0; i < 5; i++ ) {
      sum += x * coeff_dir[i][2];
      x *= ambC;    
    };
//    Serial.print( sum );
  };  
  return sum;
};

// -------------------- cold junction compensation in F units
double TypeS::mV_F( float ambF ) {
  if( inrange_F( ambF ) )
    return mV_C( F_TO_C( ambF ) );
  else
    return TC_RANGE_ERR;
};
