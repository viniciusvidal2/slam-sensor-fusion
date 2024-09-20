#ifndef _UTM_H
#define _UTM_H
#include <cmath>
#include <stdio.h>
#include <stdlib.h>


namespace UTM
{
    // Grid granularity for rounding UTM coordinates to generate MapXY.
    const double grid_size = 100000.0;    ///< 100 km grid

    // WGS84 Parameters
    #define WGS84_A		6378137.0		///< major axis
    #define WGS84_B		6356752.31424518	///< minor axis
    #define WGS84_F		0.0033528107		///< ellipsoid flattening
    #define WGS84_E		0.0818191908		///< first eccentricity
    #define WGS84_EP	0.0820944379		///< second eccentricity

    // UTM Parameters
    #define UTM_K0		0.9996			///< scale factor
    #define UTM_FE		500000.0		///< false easting
    #define UTM_FN_N	0.0           ///< false northing, northern hemisphere
    #define UTM_FN_S	10000000.0    ///< false northing, southern hemisphere
    #define UTM_E2		(WGS84_E*WGS84_E)	///< e^2
    #define UTM_E4		(UTM_E2*UTM_E2)		///< e^4
    #define UTM_E6		(UTM_E4*UTM_E2)		///< e^6
    #define UTM_EP2		(UTM_E2/(1-UTM_E2))	///< e'^2

    // Converts degrees to radians
    #define DEG_TO_RAD	0.017453292519943295769236907684886

    static inline void LLtoUTM(const double Lat, const double Long,
                               double &UTMNorthing, double &UTMEasting)
    {
        double a = WGS84_A;
        double eccSquared = UTM_E2;
        double k0 = UTM_K0;

        double LongOrigin;
        double eccPrimeSquared;
        double N, T, C, A, M;

        //Make sure the longitude is between -180.00 .. 179.9
        double LongTemp = (Long+180)-int((Long+180)/360)*360-180;

        double LatRad = Lat*DEG_TO_RAD;
        double LongRad = LongTemp*DEG_TO_RAD;
        double LongOriginRad;
        int    ZoneNumber;

        ZoneNumber = int((LongTemp + 180)/6) + 1;

        if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
            ZoneNumber = 32;

        // Special zones for Svalbard
        if( Lat >= 72.0 && Lat < 84.0 )
        {
            if(      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
            else if( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
            else if( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
            else if( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
        }
        // +3 puts origin in middle of zone
        LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
        LongOriginRad = LongOrigin * DEG_TO_RAD;

        eccPrimeSquared = (eccSquared)/(1-eccSquared);

        N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
        T = tan(LatRad)*tan(LatRad);
        C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
        A = cos(LatRad)*(LongRad-LongOriginRad);

        M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
                - 5*eccSquared*eccSquared*eccSquared/256) * LatRad
               - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
               + (15*eccSquared*eccSquared/256
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
               - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

        UTMEasting = (double)
        (k0*N*(A+(1-T+C)*A*A*A/6
               + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
         + 500000.0);

        UTMNorthing = (double)
        (k0*(M+N*tan(LatRad)
             *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
               + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

        if(Lat < 0)
        {
            //10000000 meter offset for southern hemisphere
            UTMNorthing += 10000000.0;
        }
    }
} // end namespace UTM

#endif // _UTM_H