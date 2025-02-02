#include "sfl/lookups/CitrusDatasetZedCalibration.h"

namespace sfl::lookups {

static std::array<double, 9> ROTATION_MATRIX = { 0.9999966,  0.0000034,  0.0026000, \
                                                 0.0000034,  0.9999966, -0.0026000, \
                                                 0.0026000,  0.0026000,  0.9999933 };
static std::array<double, 3> TRANSLATION = { 0.1198, -0.0003, -0.0046 }; // From https://ucr-robotics.github.io/Citrus-Farm-Dataset/calibration.html

class Left: public MonoCameraCalibrationIf
{
  static constexpr std::array<double, 4> const INTRINSICS = {527.5591059906969, 528.5624579927512, 647.1975009993375, 357.2476935284654};
  std::array<double, 4> const& getIntrinsics() final
  {
    return INTRINSICS;
  }

  static constexpr std::array<double, 4> const DISTORTION_COEFFICIENTS = {0.004262406434905663, -0.030631455483041737, 5.567440162484537e-05, -0.00079751451332914};
  std::array<double, 4> const& getDistortionCoefficients() final
  {
    return DISTORTION_COEFFICIENTS;
  }

  /// @brief Get image size, width x height
  std::pair<unsigned int, unsigned int> getImageSize() final
  {
    return std::pair(1280, 720);
  }
};

Left left;

class Right: public MonoCameraCalibrationIf
{
  static constexpr std::array<double, 4> const INTRINSICS = {528.9103358910911, 530.2388864649299, 648.5912694971759, 356.37488150521466 };
  std::array<double, 4> const& getIntrinsics() final
  {
    return INTRINSICS;
  }

  static constexpr std::array<double, 4> const DISTORTION_COEFFICIENTS = {0.005616854427747498, -0.03723298516570143, 0.00017837381105988243, 2.8200219447953252e-05 };
  std::array<double, 4> const& getDistortionCoefficients() final
  {
    return DISTORTION_COEFFICIENTS;
  }

  /// @brief Get image size, width x height
  std::pair<unsigned int, unsigned int> getImageSize() final
  {
    return std::pair(1280, 720);
  }
};

Right right;

//-----------------------------------------------------------------------------
std::array<double, 9> const& CitrusDatasetZedCalibration::getRotation()
{
  return ROTATION_MATRIX;
}

//-----------------------------------------------------------------------------
std::array<double, 3> const& CitrusDatasetZedCalibration::getTranslation()
{
  return TRANSLATION;
}

//-----------------------------------------------------------------------------
std::pair<MonoCameraCalibrationIf&, MonoCameraCalibrationIf&> CitrusDatasetZedCalibration::getCameraCalibrations()
{
  return std::pair<MonoCameraCalibrationIf&, MonoCameraCalibrationIf&>(left, right);
}

}