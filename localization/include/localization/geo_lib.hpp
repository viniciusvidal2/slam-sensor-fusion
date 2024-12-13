#ifndef _UTM_H
#define _UTM_H
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

namespace UTM
{
    /// @brief Grid granularity for rounding UTM coordinates to generate MapXY.
    const double grid_size = 100000.0; ///< 100 km grid

/// @brief WGS84 Parameters
#define WGS84_A 6378137.0        ///< major axis
#define WGS84_B 6356752.31424518 ///< minor axis
#define WGS84_F 0.0033528107     ///< ellipsoid flattening
#define WGS84_E 0.0818191908     ///< first eccentricity
#define WGS84_EP 0.0820944379    ///< second eccentricity

/// @brief UTM Parameters
#define UTM_K0 0.9996                   ///< scale factor
#define UTM_FE 500000.0                 ///< false easting
#define UTM_FN_N 0.0                    ///< false northing, northern hemisphere
#define UTM_FN_S 10000000.0             ///< false northing, southern hemisphere
#define UTM_E2 (WGS84_E * WGS84_E)      ///< e^2
#define UTM_E4 (UTM_E2 * UTM_E2)        ///< e^4
#define UTM_E6 (UTM_E4 * UTM_E2)        ///< e^6
#define UTM_EP2 (UTM_E2 / (1 - UTM_E2)) ///< e'^2

/// @brief Converts degrees to radians
#define DEG_TO_RAD 0.017453292519943295769236907684886

    /// @brief Converts LatLon to UTM coordinates.
    /// @param lat Latitude in decimal degrees.
    /// @param lon Longitude in decimal degrees.
    /// @param utm_northing UTM Northing coordinate.
    /// @param utm_easting UTM Easting coordinate.
    static inline void LLtoUTM(const double lat, const double lon,
                               double &utm_northing, double &utm_easting)
    {
        const double a = WGS84_A;
        const double ecc_squared = UTM_E2;
        const double k0 = UTM_K0;

        // Make sure the longitude is between -180.00 .. 179.9
        const double lon_temp = (lon + 180.0) - int((lon + 180.0) / 360.0) * 360.0 - 180.0;

        double lat_rad = lat * DEG_TO_RAD;
        double lon_rad = lon_temp * DEG_TO_RAD;
        int zone_number = static_cast<int>((lon_temp + 180.0) / 6.0) + 1;

        if (lat >= 56.0 && lat < 64.0 && lon_temp >= 3.0 && lon_temp < 12.0)
        {
            zone_number = 32;
        }

        // +3 puts origin in middle of zone
        const double lon_origin_rad = ((static_cast<double>(zone_number) - 1.0) * 6.0 - 180.0 + 3.0) * DEG_TO_RAD;

        const double ecc_prime_squared = (ecc_squared) / (1.0 - ecc_squared);

        const double N = a / sqrt(1.0 - ecc_squared * sin(lat_rad) * sin(lat_rad));
        const double T = tan(lat_rad) * tan(lat_rad);
        const double C = ecc_prime_squared * cos(lat_rad) * cos(lat_rad);
        const double A = cos(lat_rad) * (lon_rad - lon_origin_rad);

        const double M = a * ((1 - ecc_squared / 4.0 - 3.0 * ecc_squared * ecc_squared / 64.0 - 5.0 * ecc_squared * ecc_squared * ecc_squared / 256.0) * lat_rad - (3.0 * ecc_squared / 8.0 + 3.0 * ecc_squared * ecc_squared / 32.0 + 45.0 * ecc_squared * ecc_squared * ecc_squared / 1024.0) * sin(2.0 * lat_rad) + (15.0 * ecc_squared * ecc_squared / 256.0 + 45.0 * ecc_squared * ecc_squared * ecc_squared / 1024.0) * sin(4.0 * lat_rad) - (35.0 * ecc_squared * ecc_squared * ecc_squared / 3072.0) * sin(6.0 * lat_rad));

        utm_easting = (k0 * N * (A + (1 - T + C) * A * A * A / 6.0 + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ecc_prime_squared) * A * A * A * A * A / 120.0) + 500000.0);

        utm_northing = k0 * (M + N * tan(lat_rad) * (A * A / 2 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0 + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ecc_prime_squared) * A * A * A * A * A * A / 720.0)) + 10000000.0; // 10000000 meter offset for southern hemisphere
    }
} // end namespace UTM

#endif // _UTM_H