#include "sfl/lookups/StereoCameraCalibrationIf.h"

namespace sfl::lookups{

/// @brief   File static storage for the calibration parameters for the ZEDi2 used by the Citrus dataset
/// @details https://ucr-robotics.github.io/Citrus-Farm-Dataset/calibration.html
class CitrusDatasetZedCalibration : public StereoCameraCalibrationIf 
{
public:
  CitrusDatasetZedCalibration() = default;
  virtual ~CitrusDatasetZedCalibration() = default;

  // From StereoCameraCalibrationIf
  std::array<double, 9> const& getRotation() final;
  std::array<double, 3> const& getTranslation() final;
  std::pair<MonoCameraCalibrationIf&, MonoCameraCalibrationIf&> getCameraCalibrations() final;

};
}