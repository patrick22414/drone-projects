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
        ("n,n-records","Maximum number of records used to calculate the trajectory of the ball", cxxopts::value<int>()->default_value("10"))
        ("r,resolution","Camera resolution. One of 's': 640x480; 'm': 1296x972; 'l': 2592x1944",cxxopts::value<char>()->default_value("s"))
        ("c,catch-alt", "Altitude used to catch the ball",cxxopts::value<float>()->default_value("-1"))
        ("t,takeoff-alt", "Altitude used for takeoff", cxxopts::value<float>()->default_value("2"))
        ("o,output", "Output to video ~/Videos/v*-catch.mp4", cxxopts::value<bool>()->default_value("false"))
        ("connection", "MAVSDK connection url", cxxopts::value<string>()->default_value("serial:///dev/serial0:921600"));
    options.parse_positional({"connection"});
    // clang-format on

    auto args = options.parse(argc, argv);

    auto video_file  = args.count("video") ? args["video"].as<string>() : "";
    auto n_records   = args["n-records"].as<int>();
    auto res_char    = args["resolution"].as<char>();
    auto catch_alt   = args["catch-alt"].as<float>();
    auto takeoff_alt = args["takeoff-alt"].as<float>();
    auto use_output  = args["output"].as<bool>();
    auto connection  = args["connection"].as<string>();

    if (args.count("help")) {
        cout << options.help() << endl;
        return 0;
    }

    if (catch_alt < 1 && catch_alt != -1) {
        cerr << CLI_COLOR_RED << "Invalid catch altitude `" << catch_alt
             << "`. Must be at least 1 meters or -1 (use current altitude)" << CLI_COLOR_NORMAL << endl;
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

    // Prepare VideoWriter
    VideoWriter writer;

    if (use_output) {
        auto codec    = VideoWriter::fourcc('m', 'p', '4', 'v');
        auto fps      = 30.0;
        auto filename = generate_video_filename();

        writer.open(filename, codec, fps, im.size(), true);

        // check if we succeeded
        if (!writer.isOpened()) {
            cerr << "Could not open the output video file for write\n";
            return -1;
        }
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

    // Subscribe to NED positions
    telemetry.set_rate_position_velocity_ned(0.2);
    telemetry.position_velocity_ned_async([](Telemetry::PositionVelocityNED position_velocity_ned) {
        cout << CLI_COLOR_BLUE << position_velocity_ned.position << CLI_COLOR_NORMAL << endl;
    });

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
    cout << CLI_COLOR_GREEN << "Ready for tracking" << CLI_COLOR_NORMAL << endl;

    // Begin tracking when the ball appeared in 2 continuous frames
    // End tracking when the ball disappeared in 2 continuous frames, or after timeout, or when reached n-records
    TrackingStatus tracking_status = BeforeTracking;

    // Negative means number of continuous frames without a ball
    // Positive means number of continuous frames with a ball
    int tracking_counter = 0;

    // Record ball position, radius, timestamp, drone position, rotation here
    list<TrackingRecord> tracking_records;

    // Color range for the orange ball
    const Scalar lower_orange_1 = {0, 28, 0};
    const Scalar upper_orange_1 = {60, 255, 100};

    const Scalar lower_orange_2 = {108, 28, 0};
    const Scalar upper_orange_2 = {180, 255, 100};

    Mat im_hsv;
    Mat im_bin_1;
    Mat im_bin_2;

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

    // Assume 30 FPS, tracking timeout in 15 seconds
    for (int i = 0; i < 30 * 15; ++i) {
        if (tracking_status == AfterTracking)
            break;

        TrackingRecord new_record{
#ifdef USE_DRONE
            .drone_position = telemetry.position_velocity_ned().position,
            .drone_rotation = telemetry.attitude_euler_angle(),
#else
            .drone_position = {0, 0, -takeoff_alt},
            .drone_rotation = {0, 0, 0},
#endif // USE_DRONE
        };

        if (!v.read(im)) {
            cerr << "Cannot read frame or end of video" << endl;
            break;
        }

        if (use_output)
            writer.write(im);

        // Convert to HSV colo space
        cvtColor(im, im_hsv, COLOR_BGR2HSV);

        // Filter color within range
        inRange(im_hsv, lower_orange_1, upper_orange_1, im_bin_1);
        inRange(im_hsv, lower_orange_2, upper_orange_2, im_bin_2);

        // Combine two color ranges
        bitwise_or(im_bin_1, im_bin_2, im_bin_1);

        morphologyEx(im_bin_1, im_bin_1, MORPH_OPEN, element);
        morphologyEx(im_bin_1, im_bin_1, MORPH_DILATE, element);

        // Find contours
        vector<vector<Point2i>> contours;
        vector<Vec4i> hierarchy;
        findContours(im_bin_1, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find largest contour by area
            sort(contours.begin(), contours.end(), [](const vector<Point2i>& c1, const vector<Point2i>& c2) {
                return contourArea(c1, false) > contourArea(c2, false);
            });

            auto largest_contour = contours[0];

            Point2f center;
            float radius;
            minEnclosingCircle(largest_contour, center, radius);

            if (radius > 5) {
                // Count as a ball if the blob has a large enough radius
                tracking_counter = tracking_counter <= 0 ? 1 : tracking_counter + 1;

                new_record.xi = center.x;
                new_record.yi = center.y;
                new_record.ri = radius;
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

        if (tracking_status == BeforeTracking && tracking_counter >= 2)
            tracking_status = DuringTracking;

        if (tracking_status == DuringTracking && tracking_counter <= -2)
            tracking_status = AfterTracking;

        if (tracking_status == DuringTracking && tracking_records.size() >= n_records + 2)
            tracking_status = AfterTracking;
    }

    // End of tracking
    //    writer.release();
    //    v.release();
    //    im.release();
    //    im_hsv.release();
    //    im_bin_1.release();
    //    im_bin_2.release();
    cout << CLI_COLOR_GREEN << "End of tracking" << CLI_COLOR_NORMAL << endl;

    // Drop the first and last records as they are inaccurate
    tracking_records.pop_front();
    tracking_records.pop_back();

    cout << "Positions acquired in " << tracking_records.size() << " frames" << endl;

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
    vector<Vec3f> positions_w;
    for (const auto& record : tracking_records) {
        positions_w.push_back(record.position_w);
    }

    if (catch_alt == -1) {
#ifdef USE_DRONE
        catch_alt = telemetry.position().relative_altitude_m;
#else
        catch_alt = 3;
#endif // USE_DRONE
    }

    Offboard::PositionNEDYaw destination{};
    try {
        destination = calculate_destination_2(positions_w, catch_alt);
    } catch (const runtime_error& e) {
        cout << CLI_COLOR_RED << e.what() << CLI_COLOR_NORMAL << endl;
#ifdef USE_DRONE
        exit_and_land(action, telemetry);
#else
        exit(-1);
#endif // USE_DRONE
    }

    cout << CLI_COLOR_GREEN << "Destination is at " << destination << CLI_COLOR_NORMAL << endl;

    if (abs(destination.north_m) > 10 || abs(destination.east_m) > 10) {
        cout << CLI_COLOR_RED << "Destination too far. Aborting for safety" << CLI_COLOR_NORMAL << endl;
#ifdef USE_DRONE
        exit_and_land(action, telemetry);
#else
        exit(-1);
#endif // USE_DRONE
    }

#ifdef USE_DRONE
    // Fly to destination in offboard mode
    offboard.set_position_ned(destination);

    Offboard::Result offboard_result = offboard.start();
    check_offboard_result(offboard_result, "Offboard start failed: ");
    cout << "Offboard started" << endl;

    offboard.set_position_ned(destination);
    sleep_for(seconds(10));

    offboard_result = offboard.stop();
    check_offboard_result(offboard_result, "Offboard stop failed: ");
    cout << "Offboard stopped" << endl;

    exit_and_land(action, telemetry);
#endif // USE_DRONE

    return 0;
}
