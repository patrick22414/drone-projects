#ifndef PROJECT_CHASE_CAMERA_PROFILE_H
#define PROJECT_CHASE_CAMERA_PROFILE_H

#include <Eigen/Dense>

namespace eg = Eigen;

class CameraProfile {
public:
    CameraProfile(int index, float focal_length_mm, const eg::Vector2f& sensor_size_mm)
    {
        // clang-format off
        intrinsics <<
            focal_length_mm * 1e-3,                      0, 0.5f * sensor_size_mm[0] * 1e-3,
                                 0, focal_length_mm * 1e-3, 0.5f * sensor_size_mm[1] * 1e-3,
                                 0,                      0,                               1;
        // clang-format on

        this->index = index;
    }

    int          index;
    eg::Matrix3f intrinsics;
};

#endif // PROJECT_CHASE_CAMERA_PROFILE_H
