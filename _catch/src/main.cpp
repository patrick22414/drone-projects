#include <opencv2/opencv.hpp>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "internal.h"

using namespace cv;
using namespace mavsdk;
using namespace std;
using namespace this_thread;
using namespace chrono;

void fitParabola(const vector<Vec4d>& points)
{
    // TODO: find the parameters of a parabola from a series of points in World frame
}

void worldTransform(const Vec3d& p, Telemetry::PositionNED t, Telemetry::EulerAngle r)
{
    // TODO: given a point in the Camera frame, transform the position into the World frame
}

Vec3d cameraTransform(const Vec3d& ball_position_radius_px)
{
    // TODO: given a point in the Image frame in pixels, and the distance from the camera to the
    // ball, find the position of the ball in the Camera frame
    return {0, 0, 0};
}

void calculateDestination() {
    // TODO
}

pair<Vec3d, steady_clock::time_point> getBallInImage(VideoCapture v)
{
    /* Get ball position, radius, and timestamp in Image frame
     * Parameters:
     *     v : VideoCapture
     * Returns:
     *     Vec3d     : {xi_px, yi_px, r_px}
     *     timestamp : steady_clock right before the image was captured
     */
    Mat im;

    auto timestamp = steady_clock::now();
    if (!v.read(im)) {
        cerr << CLI_COLOR_RED << "Cannot read frame" << CLI_COLOR_NORMAL << endl;
        exit(-1);
    }

    Mat im_grey;
    cvtColor(im, im_grey, COLOR_BGR2GRAY);

    threshold(im_grey, im_grey, 127, 255, THRESH_BINARY_INV);

    vector<vector<Point2i>> contours;
    vector<Vec4i> hierarchy;
    findContours(im_grey, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        sortContours(contours);
        auto largestContour = contours[0];

        auto m = moments(largestContour);

        double area = m.m00;
        double xi = 0;
        double yi = 0;
        double ri = 0;
        if (area > 20) {
            xi = m.m10 / area;
            yi = m.m01 / area;
            ri = sqrt(area / CV_PI);

            return {{xi, yi, ri}, timestamp};
        } else {
            return {NULL, timestamp};
        }
    }

    return {NULL, timestamp};
}

int main(int argc, char* argv[])
{
    const string keys =
        "{h help ?       |    | Print this message}"
        "{n n-frames     | 10 | Maximum number of frames used to calculate the trajectory of the ball}"
        "{r resolution   | s  | Camera resolution. Valid values are s: 640x480; m: 1296x972; l: 2592x1944}"
        "{a altitude     | -1 | Altitude used by the drone to catch the ball. Default -1 will remain at current altitude}"
        "{ta takeoff-alt | 3  | Takeoff altitude}"
        "{@connection    | serial:///dev/serial0:921600 | MAVLink connection url}";

    CommandLineParser parser(argc, argv, keys);
    parser.about("The main program that will catch the ball");

    if (argc == 1) {
        parser.printMessage();
        return 0;
    }

    // Parse and check arguments
    auto n_frames = parser.get<int>("n-frames");
    auto altitude = parser.get<int>("altitude");
    auto takeoff_alt = parser.get<float>("takeoff-alt");
    auto res_char = parser.get<string>("resolution")[0];
    auto connection = parser.get<string>("@connection");

    if (!parser.check()) {
        parser.printErrors();
        exit(-1);
    }

    cout << "Get command line arg `n-frames`: " << n_frames << endl;
    cout << "Get command line arg `altitude`: " << altitude << endl;
    cout << "Get command line arg `takeoff-alt`: " << takeoff_alt << endl;
    cout << "Get command line arg `resolution`: " << res_char << endl;
    cout << "Get command line arg `connection`: " << connection << endl;

    if (n_frames < 8) {
        cerr << CLI_COLOR_RED << "Invalid n-frames `" << n_frames << "`. Must be at least 8"
             << CLI_COLOR_NORMAL << endl;
        exit(-1);
    }

    if (altitude < 3 && altitude != -1) {
        cerr << CLI_COLOR_RED << "Invalid altitude `" << altitude
             << "`. Must be at least 3 meters or -1 (use current altitude)" << CLI_COLOR_NORMAL
             << endl;
        exit(-1);
    }

    Vec2i resolution;
    if (res_char == 's')
        resolution = {640, 480};
    else if (res_char == 'm')
        resolution = {1296, 972};
    else if (res_char == 'l')
        resolution = {2592, 1944};
    else {
        cerr << CLI_COLOR_RED << "Invalid resolution `" << res_char << "`. Must be one of s/m/l"
             << CLI_COLOR_NORMAL << endl;
        exit(-1);
    }

    // Prepare VideoCapture
    VideoCapture v(0, CAP_V4L2);

    v.set(CAP_PROP_FRAME_WIDTH, resolution[0]);
    v.set(CAP_PROP_FRAME_HEIGHT, resolution[1]);

    if (!v.isOpened()) {
        cerr << CLI_COLOR_RED << "Cannot open video capture" << CLI_COLOR_NORMAL << endl;
        exit(-1);
    }

    // Test VideoCapture
    Mat test_image;
    if (!v.read(test_image)) {
        cerr << CLI_COLOR_RED << "Cannot read frame" << CLI_COLOR_NORMAL << endl;
        exit(-1);
    }

    getBallInImage(v);

    // Prepare Drone
    Mavsdk mav;
    auto connection_result = mav.add_any_connection(connection);
    check_connection_result(connection_result, "Connection failed: ");

    // Wait for system to connect via heartbeat
    while (!mav.is_connected()) {
        cout << CLI_COLOR_YELLOW << "Waiting for system to connect via heartbeat"
             << CLI_COLOR_NORMAL << endl;
        sleep_for(seconds(1));
    }

    // Make system objects
    System& system = mav.system();
    auto action = make_shared<Action>(system);
    auto offboard = make_shared<Offboard>(system);
    auto telemetry = make_shared<Telemetry>(system);

    while (!telemetry->health_all_ok()) {
        cout << CLI_COLOR_YELLOW << "Waiting for system to be ready" << CLI_COLOR_NORMAL << endl;
        sleep_for(seconds(1));
    }

    cout << CLI_COLOR_GREEN << "System is ready" << CLI_COLOR_NORMAL << endl;

    // Arm system
    Action::Result arm_result = action->arm();
    check_action_result(arm_result, "Arm failed: ");
    cout << "Armed" << endl;

    // System Takeoff!!!
    action->set_takeoff_altitude(takeoff_alt);
    Action::Result takeoff_result = action->takeoff();
    check_action_result(takeoff_result, "Takeoff failed: ");
    sleep_for(seconds(5));

    // TODO: Catch a ball!

    v.release();

    // Return the vehicle
    action->set_return_to_launch_return_altitude(10.0f);
    const Action::Result rtl_result = action->return_to_launch();
    check_action_result(rtl_result, "Return to launch failed: ");

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        cout << CLI_COLOR_YELLOW << "Vehicle is returning..." << CLI_COLOR_NORMAL << endl;
        sleep_for(seconds(2));
    }

    cout << CLI_COLOR_GREEN << "System landed!" << CLI_COLOR_NORMAL << endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(2));
    cout << "Finished." << endl;

    return 0;
}
