#include <opencv2/opencv.hpp>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <optional>

#include "internal.h"

using namespace cv;
using namespace mavsdk;
using namespace std;
using namespace this_thread;
using namespace chrono;


optional<tuple<Vec2d, double, steady_clock::time_point>> getBallInImage(VideoCapture &v)
{
    /* Get ball position, radius, and timestamp in Image frame
     * Parameters:
     *     v : VideoCapture
     * Returns:
     *     Vec2d     : {xi_px, yi_px}
     *     double    : ri_px
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

    threshold(im_grey, im_grey, 90, 255, THRESH_BINARY_INV);

#if 0
    imshow("im", im);
    imshow("im_grey", im_grey);
    waitKey(-1);
    destroyAllWindows();
#endif

    vector<vector<Point2i>> contours;
    vector<Vec4i> hierarchy;
    findContours(im_grey, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    im.release();
    im_grey.release();

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

            return tuple<Vec2d, double, steady_clock::time_point>({xi, yi}, ri, timestamp);
        } else {
            return {};
        }
    }

    return {};
}


Vec3d cameraTransform(const Vec2d &ball_position_px, double r_px, const Vec2d &resolution)
{
    const double tennis_size = 6.6e-2; // tennis ball size 6.6cm

    // Pi Camera V1 parameters
    const double sensor_w_px = 2592; // sensor width in pixels
    const double sensor_h_px = 1944; // sensor height in pixels
    const double focal_length = 3.6e-3; // focal length 3.6mm
    const double real_pixel_size = 1.4e-6; // real pixel size 1.4um

    double u_px = ball_position_px[0];
    double v_px = ball_position_px[1];

    double res_w = resolution[0];
    double res_h = resolution[1];

    double pix_size = real_pixel_size * (sensor_w_px / res_w); // relative pixel size with binning

    // camera intrinsic matrix (partial)
    double intrinsics[3][3] = {
            {focal_length, 0,            0.5 * res_w * pix_size},
            {0,            focal_length, 0.5 * res_h * pix_size},
            {0,            0,            1},
    };

    // object homogeneous coordinates (Image frame) (z = 1)
    double object_i_homo[3] = {u_px * pix_size, v_px * pix_size, 1};

    Mat A(3, 3, CV_64F, intrinsics);
    Mat b(3, 1, CV_64F, object_i_homo);
    Mat x(3, 1, CV_64F);
    solve(A, b, x, DECOMP_SVD);

    // ray direction (xc1, yc1 , 1)
    double xc1 = x.at<double>(0);
    double yc1 = x.at<double>(1);

    // distance from camera to ball
    double distance = tennis_size * focal_length / (r_px * pix_size);
    double distance1 = sqrt(xc1 * xc1 + yc1 * yc1 + 1);

    // object coordinates (Camera frame)
    double xc = xc1 * distance / distance1;
    double yc = yc1 * distance / distance1;
    double zc = sqrt(distance * distance - xc * xc - yc * yc);

    return {xc, yc, zc};
}


Vec3d worldTransform(const Vec3d &p, const Telemetry::PositionNED &t, const Telemetry::EulerAngle &r)
{
    // TODO: given a point in the Camera frame, transform the position into the World frame

    Matx33d drone_attitude_rotation = eulerAngleToRotationMatrix(r);
    Matx33d camera_mounting_rotation = eulerAngleToRotationMatrix({0, 0, -65});

    // Total extrinsic rotation
    Matx33d A = drone_attitude_rotation * camera_mounting_rotation;

    //
    Matx31d b(
            p[0] - t.north_m, p[1] - t.east_m, p[2] - t.down_m
    );

    Matx31d x = A.solve(b, DECOMP_SVD);

    return Vec3d(x.val);
}


Vec6d calculateParabola(const vector<Vec3d> &points, const vector<nanoseconds> &timestamps)
{
    // Find the parameters of a parabola from a series of points in World frame
    assert(points.size() == timestamps.size());

    int n = points.size();

    vector<double> A_data(n * 3 * 6, 0.0);
    vector<double> b_data(n * 3);
    for (int i = 0; i < n; ++i) {
        double s = 1e-9 * timestamps[i].count(); // time in second

        int A_start = i * 3 * 6;
        A_data[A_start + 3] = A_data[A_start + 13] = A_data[A_start + 23] = s;
        A_data[A_start + 6] = A_data[A_start + 16] = A_data[A_start + 26] = 1;

        int b_start = i * 3;
        b_data[b_start + 0] = points[i][0];
        b_data[b_start + 1] = points[i][1];
        b_data[b_start + 2] = points[i][2] - G_HALF * s * s;
    }

    Mat A = Mat(A_data).reshape(1, n * 3);
    Mat b = Mat(b_data);
    Mat x = Mat(6, 1, CV_64F);

    solve(A, b, x, DECOMP_SVD);

    Vec6d parabola((double *) x.data);

    return parabola;
}


Offboard::PositionNEDYaw calculateDestination(const Vec6d &parabola, double catch_alt)
{
    // TODO
}


int main(int argc, char *argv[])
{
    const string keys =
            "{h help ?       |    | Print this message}"
            "{v video        |    | Use a video file instead of camera}"
            "{n n-frames     | 10 | Maximum number of frames used to calculate the trajectory of the ball}"
            "{r resolution   | s  | Camera resolution. Valid values are s: 640x480; m: 1296x972; l: 2592x1944}"
            "{a altitude     | -1 | Altitude used by the drone to catch the ball. Default -1 will remain at current altitude}"
            "{ta takeoff-alt | 3  | Takeoff altitude}"
            "{@connection    | serial:///dev/serial0:921600 | MAVLink connection url}";

    CommandLineParser parser(argc, argv, keys);
    parser.about("The main program that will catch the ball");

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    // Parse and check arguments
    auto video_path = parser.get<string>("video");
    auto n_frames = parser.get<int>("n-frames");
    auto altitude = parser.get<int>("altitude");
    auto takeoff_alt = parser.get<float>("takeoff-alt");
    auto res_char = parser.get<string>("resolution")[0];
    auto connection = parser.get<string>("@connection");

    if (!parser.check()) {
        parser.printErrors();
        exit(-1);
    }

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

    Vec2d resolution;
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
    VideoCapture v;
    if (parser.has("video")) {
        cout << "Using video file " << video_path << endl;
        v = VideoCapture(video_path);

        if (parser.has("resolution")) {
            cout << "Ignoring resolution setting" << endl;
        }
    } else {
        cout << "Using camera with resolution " << res_char << ": " << resolution << endl;
        v = VideoCapture(0, CAP_V4L2);
        v.set(CAP_PROP_FRAME_WIDTH, resolution[0]);
        v.set(CAP_PROP_FRAME_HEIGHT, resolution[1]);
    }

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

    test_image.release();

#ifdef USE_DRONE_CONTROL
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

    System& system = mav.system();
    cout << CLI_COLOR_GREEN << "System connected: " << system.get_uuid() << CLI_COLOR_NORMAL
         << endl;

    // Make system objects
    auto action = Action(system);
    auto offboard = Offboard(system);
    auto telemetry = Telemetry(system);

    while (!telemetry.health_all_ok()) {
        cout << CLI_COLOR_YELLOW << "Waiting for system to be ready" << CLI_COLOR_NORMAL << endl;
        sleep_for(seconds(1));
    }

    cout << CLI_COLOR_GREEN << "System is ready" << CLI_COLOR_NORMAL << endl;

    // Arm system
    Action::Result arm_result = action.arm();
    check_action_result(arm_result, "Arm failed: ");
    cout << "Armed" << endl;

    // System Takeoff!!!
    cout << "Using takeoff altitude: " << takeoff_alt << "m" << endl;
    action.set_takeoff_altitude(takeoff_alt);
    Action::Result takeoff_result = action.takeoff();
    check_action_result(takeoff_result, "Takeoff failed: ");
    sleep_for(seconds(5));
#endif // USE_DRONE_CONTROL

    steady_clock::time_point t0 = steady_clock::now();

    while (true) {
        auto result = getBallInImage(v);

        if (result.has_value()) {
            cout << "\nFound ball in image" << endl;
            break;
        } else {
            cout << CLI_COLOR_YELLOW << "\rWaiting for ball to appear in image" << CLI_COLOR_NORMAL;
        }
    }

    // Get everything we need to calculate the parabola
    cout << "Tracking ball position within " << n_frames << " frames" << endl;
    vector<Vec2d> positions_i;
    vector<double> radii_i;
    vector<nanoseconds> timestamps;
    vector<Telemetry::PositionNED> drone_positions;
    vector<Telemetry::EulerAngle> drone_rotations;
    for (int i = 0; i < n_frames; ++i) {
        auto result = getBallInImage(v);

        if (result.has_value()) {
#ifdef USE_DRONE_CONTROL
            drone_positions.push_back(telemetry.position_velocity_ned().position);
            drone_rotations.push_back(telemetry.attitude_euler_angle());
#endif // USE_DRONE_CONTROL
            auto values = result.value();
            positions_i.push_back(get<0>(values));
            radii_i.push_back(get<1>(values));
            timestamps.push_back(duration_cast<nanoseconds>(get<2>(values) - t0));
        }
    }

    cout << "Positions acquired in " << positions_i.size() << " frames" << endl;
    cout << "Total interval: "
         << duration_cast<milliseconds>(timestamps[timestamps.size() - 1] - timestamps[0]).count()
         << "ms" << endl;

    vector<Vec3d> positions_c;
    for (int i = 0; i < positions_i.size(); ++i) {
        positions_c.push_back(cameraTransform(positions_i[i], radii_i[i], resolution));
    }

    cout << "Get following positions in Camera frame:" << endl;
    for (const auto &p: positions_c) {
        cout << "\t" << p << "," << endl;
    }

    vector<Vec3d> positions_w;
    for (int i = 0; i < positions_c.size(); ++i) {
        positions_w.push_back(worldTransform(positions_c[i], drone_positions[i], drone_rotations[i]));
    }

    cout << "Get following positions in World frame:" << endl;
    for (const auto &p: positions_w) {
        cout << "\t" << p << "," << endl;
    }

    auto parabola = calculateParabola(positions_w, timestamps);
    cout << "Get parabola parameters: " << parabola << endl;

    v.release();

#ifdef USE_DRONE_CONTROL
    // Return the vehicle to home location
    const float rtl_altitude = 10.0f;
    cout << "Using RTL altitude " << rtl_altitude << "m" << endl;
    action.set_return_to_launch_return_altitude(rtl_altitude);

    const Action::Result rtl_result = action.return_to_launch();
    check_action_result(rtl_result, "Return to launch failed: ");

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        cout << CLI_COLOR_YELLOW << "Vehicle is returning..." << CLI_COLOR_NORMAL << endl;
        sleep_for(seconds(2));
    }

    cout << CLI_COLOR_GREEN << "System landed!" << CLI_COLOR_NORMAL << endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(2));
    cout << "Finished." << endl;
#endif // USE_DRONE_CONTROL

    return 0;
}
