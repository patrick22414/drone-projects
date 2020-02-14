#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <optional>
#include <thread>

#include "common.h"
#include "cxxopts.h"
#include "transforms.h"

using namespace cv;
using namespace mavsdk;

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

int main(int argc, char* argv[])
{
    // clang-format off
    cxxopts::Options options("catch", "The main program that will catch the ball");
    options.add_options()
        ("h,help", "Print help message")
        ("v,video", "Use a video file instead of the camera for testing", cxxopts::value<string>())
        ("n,n-records","Maximum number of records used to calculate the trajectory of the ball", cxxopts::value<int>()->default_value("30"))
        ("r,resolution","Camera resolution. One of 's': 640x480; 'm': 1296x972; 'l': 2592x1944",cxxopts::value<char>()->default_value("s"))
        ("c,catch-alt", "Altitude used to catch the ball",cxxopts::value<double>()->default_value("3"))
        ("t,takeoff-alt", "Altitude used for takeoff", cxxopts::value<double>()->default_value("3"))
        ("g,time-avg", "Enabling time average for tracking", cxxopts::value<bool>()->default_value("false"))
        ("connection", "MAVSDK connection url", cxxopts::value<string>()->default_value("serial:///dev/serial0:921600"));
    options.parse_positional({"connection"});
    // clang-format on

    auto args = options.parse(argc, argv);

    auto video_file   = args.count("video") ? args["video"].as<string>() : "";
    auto n_records    = args["n-records"].as<int>();
    auto res_char     = args["resolution"].as<char>();
    auto catch_alt    = args["catch-alt"].as<double>();
    auto takeoff_alt  = args["takeoff-alt"].as<double>();
    auto use_time_avg = args["time-avg"].as<bool>();
    auto connection   = args["connection"].as<string>();

    if (args.count("help")) {
        cout << options.help() << endl;
        return 0;
    }

    if (n_records < 15) {
        cerr << CLI_COLOR_RED << "Invalid n-records `" << n_records << "`. Must be at least 15" << CLI_COLOR_NORMAL
             << endl;
        exit(-1);
    }

    if (catch_alt < 3 && catch_alt != -1) {
        cerr << CLI_COLOR_RED << "Invalid altitude `" << catch_alt
             << "`. Must be at least 3 meters or -1 (use current altitude)" << CLI_COLOR_NORMAL << endl;
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
        cerr << CLI_COLOR_RED << "Invalid resolution `" << res_char << "`. Must be one of s/m/l" << CLI_COLOR_NORMAL
             << endl;
        exit(-1);
    }

    // Prepare VideoCapture
    VideoCapture v;
    if (args.count("video")) {
        cout << "Using video file " << video_file << endl;
        v = VideoCapture(video_file);
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

    Mat im;
    if (!v.read(im)) {
        cerr << "Cannot read frame" << endl;
        exit(-1);
    }
    if (!video_file.empty()) {
        cout << "Ignoring camera resolution setting when using a video" << endl;
        resolution = {im.cols, im.rows};
    }

#ifdef USE_DRONE
    // Prepare Drone
    Mavsdk mav;
    auto connection_result = mav.add_any_connection(connection);
    check_connection_result(connection_result, "Connection failed: ");

    // Wait for system to connect via heartbeat
    while (!mav.is_connected()) {
        cout << CLI_COLOR_YELLOW << "Waiting for system to connect via heartbeat" << CLI_COLOR_NORMAL << endl;
        sleep_for(seconds(1));
    }

    System& system = mav.system();
    cout << CLI_COLOR_GREEN << "System connected: " << system.get_uuid() << CLI_COLOR_NORMAL << endl;

    // Make system objects
    auto action    = Action(system);
    auto offboard  = Offboard(system);
    auto telemetry = Telemetry(system);

#if 0
    telemetry.set_rate_attitude(1.0);
    telemetry.attitude_euler_angle_async([](Telemetry::EulerAngle ea) {
        cout << CLI_COLOR_BLUE << "Attitude (euler angles): [Yew: " << ea.yaw_deg << ", Pitch: " << ea.pitch_deg
             << ", Roll: " << ea.roll_deg << "]" << CLI_COLOR_NORMAL << endl;
    });
#endif

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
#endif // USE_DRONE

    // Tracking
    steady_clock::time_point t0 = steady_clock::now();

    // Begin tracking when the ball appeared in 3 continuous frames
    // End tracking when the ball disappeared in 3 continuous frames, or after timeout, or when reached n-records
    TrackingStatus tracking_status = BeforeTracking;

    // Negative means number of continuous frames without a ball
    // Positive means number of continuous frames with a ball
    int tracking_counter = 0;

    // Record ball position, radius, timestamp, drone position, rotation here
    list<TrackingRecord> tracking_records;

    // Color range for the orange ball
    const Scalar lower_orange_1 = {0, 28, 0};
    const Scalar upper_orange_1 = {60, 255, 255};

    const Scalar lower_orange_2 = {108, 28, 0};
    const Scalar upper_orange_2 = {180, 255, 255};

    Mat im_hsv;
    Mat im_bin_1;
    Mat im_bin_2;

    // Ball position, radius in image
    double xi = -1;
    double yi = -1;
    double ri = -1;

    // Assume 30 FPS, tracking timeout in 15 seconds
    for (int i = 0; i < 30 * 15; ++i) {
        if (tracking_status == AfterTracking)
            break;

        TrackingRecord new_record{
            // Use real time only when not using a video. Otherwise use timeline of a 30 FPS video
            .timestamp = duration_cast<nanoseconds>(
                video_file.empty() ? steady_clock::now() - t0 : nanoseconds((int)(1e9 / 30 * (i + 1)))),
#ifdef USE_DRONE
            .drone_position = telemetry.position_velocity_ned().position,
            .drone_rotation = telemetry.attitude_euler_angle(),
#else
            .drone_position = {0, 0, (float)takeoff_alt},
            .drone_rotation = {0, 0, 0},
#endif // USE_DRONE
        };

        if (!v.read(im)) {
            cerr << "Cannot read frame or end of video" << endl;
            break;
        }

        // Convert to HSV colo space
        cvtColor(im, im_hsv, COLOR_BGR2HSV);

        // Filter color within range
        inRange(im_hsv, lower_orange_1, upper_orange_1, im_bin_1);
        inRange(im_hsv, lower_orange_2, upper_orange_2, im_bin_2);

        // Combine two color ranges
        bitwise_or(im_bin_1, im_bin_2, im_bin_1);

        // Find contours
        vector<vector<Point2i>> contours;
        vector<Vec4i> hierarchy;
        findContours(im_bin_1, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find largest contour by area
            sort(contours.begin(), contours.end(), [](const vector<Point2i>& c1, const vector<Point2i>& c2) {
                return contourArea(c1, false) > contourArea(c2, false);
            });

            auto largestContour = contours[0];

            auto m = moments(largestContour);

            double area = m.m00;

            // Use time average or not
            if (use_time_avg) {
                xi = xi < 0 ? (m.m10 / area) : (0.8 * xi + 0.2 * m.m10 / area);
                yi = yi < 0 ? (m.m01 / area) : (0.8 * yi + 0.2 * m.m01 / area);
                ri = ri < 0 ? sqrt(area / CV_PI) : (0.8 * ri + 0.2 * sqrt(area / CV_PI));
            } else {
                xi = m.m10 / area;
                yi = m.m01 / area;
                ri = sqrt(area / CV_PI);
            }

            if (ri > 6) {
                // Count as a ball if the blob has a large enough radius
                tracking_counter = tracking_counter <= 0 ? 1 : tracking_counter + 1;

                new_record.xi = xi;
                new_record.yi = yi;
                new_record.ri = ri;
                tracking_records.push_back(new_record);
            } else {
                // Otherwise the blob is just noise
                tracking_counter = tracking_counter >= 0 ? -1 : tracking_counter - 1;
            }
        } else {
            // No ball in this frame
            tracking_counter = tracking_counter >= 0 ? -1 : tracking_counter - 1;
        }

        // Examine tracking status
        if (tracking_status == BeforeTracking && tracking_counter < 0)
            tracking_records.clear();

        if (tracking_status == BeforeTracking && tracking_counter >= 3)
            tracking_status = DuringTracking;

        if (tracking_status == DuringTracking && tracking_counter <= -3)
            tracking_status = AfterTracking;

        if (tracking_status == DuringTracking && tracking_records.size() >= n_records)
            tracking_status = AfterTracking;
    }

    // End of tracking
    v.release();
    im.release();
    im_hsv.release();
    im_bin_1.release();
    im_bin_2.release();

    // Drop the first and last records as they are in accurate
    tracking_records.pop_front();
    tracking_records.pop_back();

    cout << "Positions acquired in " << tracking_records.size() << " frames" << endl;
    cout << "Total interval: "
         << duration_cast<milliseconds>(tracking_records.back().timestamp - tracking_records.front().timestamp).count()
         << "ms" << endl;

    // Transform Image frame -> Camera frame
    for (auto& record : tracking_records) {
        record.position_c = invert_camera_transform(record.xi, record.yi, record.ri, resolution);
    }

    cout << "Got following positions in Camera frame:" << endl;
    for (const auto& record : tracking_records) {
        cout << "\t" << record.position_c << "," << endl;
    }

    // Transform Camera frame -> World frame
    for (auto& record : tracking_records) {
        record.position_w = invert_world_transform(record.position_c, record.drone_position, record.drone_rotation);
    }

    cout << "Got following positions in World frame:" << endl;
    for (const auto& record : tracking_records) {
        cout << "\t" << record.position_w << "," << endl;
    }

    // From World coordinates to parabola
    vector<Vec3d> positions_w;
    vector<nanoseconds> timestamps;
    for (const auto& record : tracking_records) {
        positions_w.push_back(record.position_w);
        timestamps.push_back(record.timestamp);
    }

    auto parabola = calculate_parabola(positions_w, timestamps);
    cout << "Got parabola parameters: " << parabola << endl;

#ifdef USE_DRONE
    if (catch_alt < 3) {
        catch_alt = telemetry.position().relative_altitude_m;
    }
#endif

    Offboard::PositionNEDYaw destination{};
    try {
        destination = calculate_destination(parabola, catch_alt);
    } catch (const runtime_error& e) {
        cout << CLI_COLOR_RED << e.what() << CLI_COLOR_NORMAL << endl;
        exit_and_land(action, telemetry);
    }

    cout << CLI_COLOR_GREEN << "Destination is at " << destination << CLI_COLOR_NORMAL << endl;

    if (abs(destination.north_m) > 10 || abs(destination.east_m) > 10) {
        cout << CLI_COLOR_RED << "Destination too far. Aborting for safety" << CLI_COLOR_NORMAL << endl;
        exit_and_land(action, telemetry);
    }

#ifdef USE_DRONE
    // Fly to destination in offboard mode
    offboard.set_position_ned(destination);

    Offboard::Result offboard_result = offboard.start();
    check_offboard_result(offboard_result, "Offboard start failed: ");
    cout << "Offboard started" << endl;

    sleep_for(seconds(5));

    offboard_result = offboard.stop();
    check_offboard_result(offboard_result, "Offboard stop failed: ");
    cout << "Offboard stopped" << endl;

    // Land the vehicle
    const Action::Result land_result = action.land();
    check_action_result(land_result, "Landing failed: ");

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        cout << CLI_COLOR_YELLOW << "Vehicle is Landing..." << CLI_COLOR_NORMAL << endl;
        sleep_for(seconds(2));
    }

    cout << CLI_COLOR_GREEN << "System landed!" << CLI_COLOR_NORMAL << endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(2));
    cout << "Finished." << endl;
#endif // USE_DRONE

    return 0;
}
