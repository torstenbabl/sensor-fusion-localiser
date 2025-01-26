#include <array>
#include "sfl/lookups/MonoCameraCalibrationIf.h"

/// @brief Interface to access calibration parameters of a stereo camera pair
/// @details Uses a std::pair of MonoCameraCalibrationIf referencs for camera specific parameters
class StereoCameraCalibrationIf {
public:
  /// @brief Rotation matrix, flattened, rowwise
  virtual std::array<double, 9> const& getRotation() = 0;
  /// @brief Translation vector
  virtual std::array<double, 3> const& getTranslation() = 0;
  /// @brief Get a pair of mono camera calibrations
  virtual std::pair<MonoCameraCalibrationIf&, MonoCameraCalibrationIf&> getCameraCalibrations() = 0;
};