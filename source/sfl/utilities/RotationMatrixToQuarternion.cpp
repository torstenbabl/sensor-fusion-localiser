#include "sfl/utilities/RotationMatrixToQuarternion.h"
#include "sfl/types/Quarternion.h"

namespace sfl::utilities {
using sfl::types::Quarternion;

Quarternion rotationMatrixToQuaternion(const double matrix[9]) {
    Quarternion outputQuarternion;
    double trace = matrix[0] + matrix[4] + matrix[8];

    if (trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        outputQuarternion.w = 0.25 / s;
        outputQuarternion.i = (matrix[7] - matrix[5]) * s;
        outputQuarternion.j = (matrix[2] - matrix[6]) * s;
        outputQuarternion.k = (matrix[3] - matrix[1]) * s;
    } else {
        if (matrix[0] > matrix[4] && matrix[0] > matrix[8]) {
            double s = 2.0 * sqrt(1.0 + matrix[0] - matrix[4] - matrix[8]);
            outputQuarternion.w = (matrix[7] - matrix[5]) / s;
            outputQuarternion.i = 0.25 * s;
            outputQuarternion.j = (matrix[1] + matrix[3]) / s;
            outputQuarternion.k = (matrix[2] + matrix[6]) / s;
        } else if (matrix[4] > matrix[8]) {
            double s = 2.0 * sqrt(1.0 + matrix[4] - matrix[0] - matrix[8]);
            outputQuarternion.w = (matrix[2] - matrix[6]) / s;
            outputQuarternion.i = (matrix[1] + matrix[3]) / s;
            outputQuarternion.j = 0.25 * s;
            outputQuarternion.k = (matrix[5] + matrix[7]) / s;
        } else {
            double s = 2.0 * sqrt(1.0 + matrix[8] - matrix[0] - matrix[4]);
            outputQuarternion.w = (matrix[3] - matrix[1]) / s;
            outputQuarternion.i = (matrix[2] + matrix[6]) / s;
            outputQuarternion.j = (matrix[5] + matrix[7]) / s;
            outputQuarternion.k = 0.25 * s;
        }
    }

    return outputQuarternion;
}
}