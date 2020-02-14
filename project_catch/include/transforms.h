#ifndef CATCH_TRANSFORMS_H
#define CATCH_TRANSFORMS_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/opencv.hpp>

#include <chrono>

using namespace cv;
using namespace mavsdk;
using namespace std;
using namespace std::chrono;

Vec3d invert_camera_transform(double ball_x_px, double ball_y_px, double ball_radius_px, const Vec2i& resolution);

Vec3d invert_world_transform(const Vec3d& p_c, const Telemetry::PositionNED& t, const Telemetry::EulerAngle& r);

Vec6d calculate_parabola_2(const vector<Vec3d>& points);

Vec6d calculate_parabola(const vector<Vec3d>& points, const vector<nanoseconds>& timestamps);

Offboard::PositionNEDYaw calculate_destination_2(const Vec6d& parabola, double catch_alt);

Offboard::PositionNEDYaw calculate_destination(const Vec6d& parabola, double catch_alt);

#endif // CATCH_TRANSFORMS_H
