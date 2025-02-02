#ifndef _UTILITIES_ROTATIONMATRIXTOQUARTERNION_H_
#define _UTILITIES_ROTATIONMATRIXTOQUARTERNION_H_

#include <cmath>
#include "sfl/types/Quarternion.h"

namespace sfl::utilities {

// Function to convert a rotation matrix to a quaternion
sfl::types::Quarternion rotationMatrixToQuaternion(const double matrix[9]);

}
#endif // _UTILITIES_ROTATIONMATRIXTOQUARTERNION_H_