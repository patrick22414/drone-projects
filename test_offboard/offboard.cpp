#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <filesystem>
#include <thread>

#define CONSOLE_TEXT_ERROR "\033[31m"     // Turn text on console red
#define CONSOLE_TEXT_TELEMETRY "\033[34m" // Turn text on console blue
#define CONSOLE_TEXT_NORMAL "\033[0m"     // Restore normal console colour

using namespace mavsdk;

using std::this_thread::sleep_for;
using std::chrono::milliseconds;

namespace fs = std::filesystem;

// Handles Action's result
inline void check_action_result(Action::Result result, const std::string& message)
{
    if (result != Action::Result::SUCCESS) {
        std::cerr << CONSOLE_TEXT_ERROR << message << Action::result_str(result) << CONSOLE_TEXT_NORMAL << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void check_offboard_result(Offboard::Result result, const std::string& message)
{
    if (result != Offboard::Result::SUCCESS) {
        std::cerr << CONSOLE_TEXT_ERROR << message << Offboard::result_str(result) << CONSOLE_TEXT_NORMAL << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void check_connection_result(ConnectionResult result, const std::string& message)
{
    if (result != ConnectionResult::SUCCESS) {
        std::cerr << CONSOLE_TEXT_ERROR << message << connection_result_str(result) << CONSOLE_TEXT_NORMAL << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

static bool is_recording = true;

std::string generate_video_filename(const std::string& prefix = "test")
{
    fs::path full_filename = fs::path(std::getenv("HOME")) / "Videos";
    std::stringstream filename;
    for (int i = 1;; ++i) {
        filename.str("");
        filename << prefix << "-v" << i << ".mp4";

        if (!fs::exists(fs::path(full_filename) / filename.str())) {
            break;
        }
    }

    std::cout << "Using video filename " << fs::path(full_filename) / filename.str() << std::endl;

    return fs::path(full_filename) / filename.str();
}

void start_recording()
{
    cv::VideoCapture capture(0, cv::CAP_V4L2);

    capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    if (!capture.isOpened()) {
        std::cerr << CONSOLE_TEXT_ERROR << "Cannot open camera capture" << CONSOLE_TEXT_NORMAL << std::endl;
        return;
    }

    cv::Mat frame;
    if (!capture.read(frame)) {
        std::cerr << CONSOLE_TEXT_ERROR << "Cannot read frame or end of video" << CONSOLE_TEXT_NORMAL << std::endl;
        return;
    }

    auto filename = generate_video_filename();
    auto fps      = 30.0;
    auto codec    = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    cv::VideoWriter writer(filename, codec, fps, frame.size());

    std::cout << "Start recording!!!" << std::endl;
    while (is_recording) {
        if (!capture.read(frame)) {
            std::cerr << CONSOLE_TEXT_ERROR << "Cannot read frame or end of video" << CONSOLE_TEXT_NORMAL << std::endl;
            return;
        }

        writer.write(frame);
    }

    std::cout << "Stop recording!!!" << std::endl;
}

int main(int argc, char** argv)
{
    std::thread recording_thread(start_recording);

    Mavsdk dc;
    std::string connection_url;
    ConnectionResult connection_result;

    if (argc == 2) {
        connection_url    = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    } else {
        std::cout << CONSOLE_TEXT_NORMAL << "Usage : " << argv[0] << " <connection_url>" << std::endl
                  << "Connection URL format should be :" << std::endl
                  << "  For TCP : tcp://[server_host][:server_port]" << std::endl
                  << "  For UDP : udp://[bind_host][:bind_port]" << std::endl
                  << "  For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
                  << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
        return 1;
    }

    check_connection_result(connection_result, "Connection failed: ");

    // Wait for the system to connect via heartbeat
    while (!dc.is_connected()) {
        std::cout << "Wait for system to connect via heartbeat" << std::endl;
        sleep_for(milliseconds(1000));
    }

    // System got discovered.
    System& system = dc.system();

    Action action(system);
    Offboard offboard(system);
    Telemetry telemetry(system);

    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready" << std::endl;
        sleep_for(milliseconds(1000));
    }

    std::cout << "System is ready" << std::endl;

    Action::Result arm_result = action.arm();
    check_action_result(arm_result, "Arming failed");
    std::cout << "Armed" << std::endl;

    action.set_takeoff_altitude(5.0f);
    Action::Result takeoff_result = action.takeoff();
    check_action_result(takeoff_result, "Takeoff failed");
    std::cout << "In Air..." << std::endl;
    sleep_for(milliseconds(5000));

    // Start offboard test
    offboard.set_position_ned({0, 0, -5, 0});

    const Offboard::Result offboard_start_result = offboard.start();
    check_offboard_result(offboard_start_result, "Offboard start failed:");
    sleep_for(milliseconds(2000));

    std::cout << "PITCH DOWN to the north" << std::endl;
#ifdef USE_VELOCITY_CONTROL
    offboard.set_velocity_ned({5, 0, 0, 0});
    sleep_for(milliseconds(2000));
    offboard.set_velocity_ned({0, 0, 0, 0});
    sleep_for(milliseconds(2000));
#else
    offboard.set_position_ned({10, 0, -5, 0});
    sleep_for(milliseconds(8000));
#endif

    std::cout << "PITCH UP to the south" << std::endl;
#ifdef USE_VELOCITY_CONTROL
    offboard.set_velocity_ned({-5, 0, 0, 0});
    sleep_for(milliseconds(2000));
    offboard.set_velocity_ned({0, 0, 0, 0});
    sleep_for(milliseconds(2000));
#else
    offboard.set_position_ned({0, 0, -5, 0});
    sleep_for(milliseconds(8000));
#endif

    std::cout << "ROLL RIGHT to the east" << std::endl;
#ifdef USE_VELOCITY_CONTROL
    offboard.set_velocity_ned({0, 5, 0, 0});
    sleep_for(milliseconds(2000));
    offboard.set_velocity_ned({0, 0, 0, 0});
    sleep_for(milliseconds(2000));
#else
    offboard.set_position_ned({0, 10, -5, 0});
    sleep_for(milliseconds(8000));
#endif

    std::cout << "ROLL LEFT to the west" << std::endl;
#ifdef USE_VELOCITY_CONTROL
    offboard.set_velocity_ned({0, -5, 0, 0});
    sleep_for(milliseconds(2000));
    offboard.set_velocity_ned({0, 0, 0, 0});
    sleep_for(milliseconds(2000));
#else
    offboard.set_position_ned({0, 0, -5, 0});
    sleep_for(milliseconds(8000));
#endif

    std::cout << "YAW CLOCKWISE" << std::endl;
    offboard.set_velocity_ned({0, 0, 0, 90});
    sleep_for(milliseconds(100));
    offboard.set_velocity_ned({0, 0, 0, 180});
    sleep_for(milliseconds(5000));

    std::cout << "YAW COUNTER-CLOCKWISE" << std::endl;
    offboard.set_velocity_ned({0, 0, 0, 90});
    sleep_for(milliseconds(100));
    offboard.set_velocity_ned({0, 0, 0, 0});
    sleep_for(milliseconds(5000));

    const Offboard::Result offboard_stop_result = offboard.stop();
    check_offboard_result(offboard_stop_result, "Offboard stop failed:");

    action.set_return_to_launch_return_altitude(telemetry.position().relative_altitude_m);
    sleep_for(milliseconds(2000));

    const Action::Result land_result = action.return_to_launch();
    check_action_result(land_result, "Landing failed");

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(milliseconds(1000));
    }

    std::cout << "Landed!" << std::endl;

    // Stop recording
    is_recording = false;
    recording_thread.join();

    return 0;
}
