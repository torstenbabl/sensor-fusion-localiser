// Adapted from https://github.com/ethz-asl/geodetic_utils/blob/master/geodetic_utils/include/geodetic_utils/geodetic_conv.hpp
#ifndef GEODETIC_CONVERTER_H_
#define GEODETIC_CONVERTER_H_

#include "sfl/types/GpsReading.h"
#include "math.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

// Geodetic system parameters
static double kSemimajorAxis = 6378137;
static double kSemiminorAxis = 6356752.3142;
static double kFirstEccentricitySquared = 6.69437999014 * 0.001;
static double kSecondEccentricitySquared = 6.73949674228 * 0.001;
static double kFlattening = 1 / 298.257223563;

class GeodeticConverter{
public:

  GeodeticConverter(const sfl::types::GpsReading &gnss){
    Eigen::Vector3d origin;
    origin(0) = gnss.latitude;
    origin(1) = gnss.longitude;
    origin(2) = gnss.heightAboveSpheriod;
    // Save NED origin
    initial_llh_(0) = deg2Rad(origin.x());
    initial_llh_(1) = deg2Rad(origin.y());
    initial_llh_(2) = origin.z();

    // Compute ECEF of NED origin
    initial_ecef_ = geodetic2Ecef(origin);

    // Compute ECEF to NED and NED to ECEF matrices
    double phiP = atan2(initial_ecef_.z(), sqrt(pow(initial_ecef_.x(), 2) + pow(initial_ecef_.y(), 2)));

    ecef_to_ned_matrix_ = nRe(phiP, initial_llh_.y());
    ned_to_ecef_matrix_ = nRe(initial_llh_.x(), initial_llh_.y()).transpose();
  }

  GeodeticConverter() = default;

  ~GeodeticConverter(){}
  // Default copy constructor and assignment operator are OK.

  Eigen::Vector3d
  geodetic2Ecef(const Eigen::Vector3d &geodetic){
    // Convert geodetic coordinates to ECEF.
    // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
    Eigen::Vector3d ret;
    double lat_rad = deg2Rad(geodetic.x());
    double lon_rad = deg2Rad(geodetic.y());
    double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
    ret(0) = (kSemimajorAxis / xi + geodetic.z()) * cos(lat_rad) * cos(lon_rad);
    ret(1) = (kSemimajorAxis / xi + geodetic.z()) * cos(lat_rad) * sin(lon_rad);
    ret(2) = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + geodetic.z()) * sin(lat_rad);

    return ret;
  }

  Eigen::Vector3d
  ecef2Geodetic(const Eigen::Vector3d ecef){
    // Convert ECEF coordinates to geodetic coordinates.
    // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
    // to geodetic coordinates," IEEE Transactions on Aerospace and
    // Electronic Systems, vol. 30, pp. 957-961, 1994.

    double r = sqrt(ecef.x() * ecef.x() + ecef.y() * ecef.y());
    double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
    double F = 54 * kSemiminorAxis * kSemiminorAxis * ecef.z() * ecef.z();
    double G = r * r + (1 - kFirstEccentricitySquared) * ecef.z() * ecef.z() - kFirstEccentricitySquared * Esq;
    double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
    double S = cbrt(1 + C + sqrt(C * C + 2 * C));
    double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
    double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
    double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q)
        + sqrt(
            0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q)
                - P * (1 - kFirstEccentricitySquared) * ecef.z() * ecef.z() / (Q * (1 + Q)) - 0.5 * P * r * r);
    double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + ecef.z() * ecef.z());
    double V = sqrt(
        pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * ecef.z() * ecef.z());
    double Z_0 = kSemiminorAxis * kSemiminorAxis * ecef.z() / (kSemimajorAxis * V);
    Eigen::Vector3d ret;
    ret(2) = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
    ret(0) = rad2Deg(atan((ecef.z() + kSecondEccentricitySquared * Z_0) / r));
    ret(1) = rad2Deg(atan2(ecef.z(), ecef.z()));
    return ret;
  }

  Eigen::Vector3d
  ecef2Ned(const Eigen::Vector3d ecef){
    // Converts ECEF coordinate position into local-tangent-plane NED.
    // Coordinates relative to given ECEF coordinate frame.

    Eigen::Vector3d vect, ret;
    vect = ecef - initial_ecef_;
    ret = ecef_to_ned_matrix_ * vect;
    ret(2) = -ret(2);
    return ret;
  }

  Eigen::Vector3d
  ned2Ecef(const double north, const double east, const double down){
    // NED (north/east/down) to ECEF coordinates
    Eigen::Vector3d ned, ret;
    ned(0) = north;
    ned(1) = east;
    ned(2) = -down;
    ret = ned_to_ecef_matrix_ * ned;
    return ret + initial_ecef_;
  }

  Eigen::Vector3d
  relativeNEDto(const sfl::types::GpsReading &gnss){
    // Geodetic position to local NED frame
    Eigen::Vector3d point;
    point(0) = gnss.latitude;
    point(1) = gnss.longitude;
    point(2) = gnss.heightAboveSpheriod;
    auto ecef = geodetic2Ecef(point);
    return ecef2Ned(ecef);
  }

  Eigen::Vector3d
  relativeENUto(const sfl::types::GpsReading &gnss){
    auto ned = relativeNEDto(gnss);
    return {ned.y(), ned.x(), -ned.z()};
  }

  Eigen::Vector3d
  ned2Geodetic(const double north, const double east, const double down){
    // Local NED position to geodetic coordinates
    auto ecef = ned2Ecef(north, east, down);
    return ecef2Geodetic(ecef);
  }

  Eigen::Vector3d
  enu2Geodetic(const double east, const double north, const double up){
    return ned2Geodetic(north, east, -up);
  }

 private:

  inline Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians)
  {
    const double sLat = sin(lat_radians);
    const double sLon = sin(lon_radians);
    const double cLat = cos(lat_radians);
    const double cLon = cos(lon_radians);

    Eigen::Matrix3d ret;
    ret(0, 0) = -sLat * cLon;
    ret(0, 1) = -sLat * sLon;
    ret(0, 2) = cLat;
    ret(1, 0) = -sLon;
    ret(1, 1) = cLon;
    ret(1, 2) = 0.0;
    ret(2, 0) = cLat * cLon;
    ret(2, 1) = cLat * sLon;
    ret(2, 2) = sLat;

    return ret;
  }

  inline
  double rad2Deg(const double radians)
  {
    return (radians / M_PI) * 180.0;
  }

  inline
  double deg2Rad(const double degrees)
  {
    return (degrees / 180.0) * M_PI;
  }

  Eigen::Vector3d initial_llh_;
  Eigen::Vector3d initial_ecef_;

  Eigen::Matrix3d ecef_to_ned_matrix_;
  Eigen::Matrix3d ned_to_ecef_matrix_;

}; // class GeodeticConverter

#endif // GEODETIC_CONVERTER_H_
