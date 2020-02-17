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

Vec3f invert_camera_transform(float ball_x_px, float ball_y_px, float ball_radius_px, const Vec2i& resolution);

Vec3f invert_world_transform(const Vec3f& p_c, const Telemetry::PositionNED& t, const Telemetry::EulerAngle& r);

Offboard::PositionNEDYaw calculate_destination_2(const vector<Vec3f>& points_w, float catch_alt);

#endif // CATCH_TRANSFORMS_H
