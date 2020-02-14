#ifndef CATCH_COMMON_H
#define CATCH_COMMON_H

#define G_HALF (0.5 * 9.81)

#define CLI_COLOR_RED "\033[31m"    // Turn text on console red
#define CLI_COLOR_GREEN "\033[32m"  // Turn text on console red
#define CLI_COLOR_YELLOW "\033[33m" // Turn text on console red
#define CLI_COLOR_BLUE "\033[34m"   // Turn text on console blue
#define CLI_COLOR_NORMAL "\033[0m"  // Restore normal console colour

#include <opencv2/opencv.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <thread>

using namespace cv;
using namespace mavsdk;

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

enum TrackingStatus {
    BeforeTracking,
    DuringTracking,
    AfterTracking,
};

struct TrackingRecord {
    double xi;
    double yi;
    double ri;

    nanoseconds timestamp;

    Telemetry::PositionNED drone_position;
    Telemetry::EulerAngle drone_rotation;

    Vec3d position_c;
    Vec3d position_w;
};

void check_action_result(Action::Result result, const string& fail_message);

void check_offboard_result(Offboard::Result result, const string& fail_message);

void check_connection_result(ConnectionResult result, const string& fail_message);

void exit_and_land(const Action& action, const Telemetry& telemetry);

Matx33d euler_angle_to_rotation_matrix(const Telemetry::EulerAngle& ea);

pair<optional<double>, optional<double>> solve_quadratic(double a, double b, double c);

#endif // CATCH_COMMON_H
