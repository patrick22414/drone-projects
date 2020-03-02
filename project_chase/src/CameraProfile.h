#ifndef PROJECT_CHASE_CAMERAPROFILE_H
#define PROJECT_CHASE_CAMERAPROFILE_H

#include <Eigen/Dense>

namespace eg = Eigen;

class CameraProfile {
public:
    CameraProfile(int index, float focal_length_m, float pixel_size_m, eg::Vector2i resolution)
    {
        this->index = index;

        // clang-format off
        intrinsics <<
            focal_length_m,              0, 0.5f * (resolution[0] - 1) * pixel_size_m,
            0,              focal_length_m, 0.5f * (resolution[1] - 1) * pixel_size_m,
            0,                           0,                                           1;
        // clang-format on
    }

    int index;

    eg::Matrix3f intrinsics;
};

#endif // PROJECT_CHASE_CAMERAPROFILE_H
