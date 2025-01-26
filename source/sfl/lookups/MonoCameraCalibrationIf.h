#include <array>

/// @brief Interface to access calibration parameters of a pinhole camera
class MonoCameraCalibrationIf{
public:
  /// @brief Get the intrinsics of a pinhole camera
  virtual std::array<double, 4> const& getIntrinsics() = 0;

  /// @brief Get the distortion coeficients
  virtual std::array<double, 4> const& getDistortionCoefficients() = 0;

  /// @brief Get image size, width x height
  virtual std::pair<unsigned int, unsigned int> getImageSize() = 0;

};