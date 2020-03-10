#ifndef PROJECT_CHASE_CAMERAPROFILE_H
#define PROJECT_CHASE_CAMERAPROFILE_H

#include <Eigen/Dense>

namespace eg = Eigen;

class CameraProfile {
public:
    CameraProfile(
        int                 index,
        float               focal_length_mm,
        const eg::Vector2f& sensor_size_mm,
        const eg::Vector2i& resolution = {640, 480})
    {
        // clang-format off
        intrinsics <<
            focal_length_mm * 1e-3,                      0, 0.5f * sensor_size_mm[0] * 1e-3,
                                 0, focal_length_mm * 1e-3, 0.5f * sensor_size_mm[1] * 1e-3,
                                 0,                      0,                               1;
        // clang-format on

        this->index      = index;
        this->resolution = resolution;
    }

    int          index;
    eg::Vector2i resolution;
    eg::Matrix3f intrinsics;
};

#endif // PROJECT_CHASE_CAMERAPROFILE_H
