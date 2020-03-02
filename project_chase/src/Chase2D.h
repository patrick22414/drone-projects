#ifndef PROJECT_CHASE_CHASE2D_H
#define PROJECT_CHASE_CHASE2D_H

#define CLI_COLOR_RED "\033[31m"    // Turn text on console red
#define CLI_COLOR_GREEN "\033[32m"  // Turn text on console red
#define CLI_COLOR_YELLOW "\033[33m" // Turn text on console red
#define CLI_COLOR_NORMAL "\033[0m"  // Restore normal console colour

#include <Eigen/Dense>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

namespace mav = mavsdk;
namespace eg  = Eigen;

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

class Chase2D {
    // TODO
public:
    typedef std::chrono::steady_clock             Clock;
    typedef std::chrono::steady_clock::time_point Timestamp;

    Chase2D(
        const std::string&  connection,
        int                 camera,
        const eg::Vector2i& resolution,
        const std::string&  video_output = "");

    Chase2D(const std::string& connection, const std::string& video_input);

    ~Chase2D() { stop(); };

    void start(float v_speed, float h_speed, bool show_live = false);

    void stop();

    void generate_flight_log(const std::string& log_file);

private:
    explicit Chase2D(const std::string& connection);

    bool is_recording = false;
    bool is_tracking  = false;
    bool is_chasing   = false;

    std::thread recording_thread;
    std::thread tracking_thread;
    std::thread chasing_thread;

    Timestamp old_timestamp;
    Timestamp new_timestamp;

    std::list<eg::Vector2f> positions_i; // target positions in Image frame
    std::list<eg::Vector3f> positions_c; // target positions in Camera frame, `z` is always 1 since depth is unknown
    std::list<eg::Vector3f> positions_w; // target positions in World frame, using NED coordinates

    cv::VideoCapture capture;
    cv::VideoWriter  writer;

    eg::Vector2i resolution;

    std::shared_ptr<mav::Action>    action;
    std::shared_ptr<mav::Offboard>  offboard;
    std::shared_ptr<mav::Telemetry> telemetry;

    cv::Mat im;

    void recording_routine(bool show_live);
    void tracking_routine(bool show_live);
    void chasing_routine(float v_speed, float h_speed);

    eg::Vector3f invert_camera_transform(const eg::Vector2f& position_i, float z); // TODO: add camera profile
    eg::Vector3f invert_world_transform(const eg::Vector3f& position_c); // TODO: add drone rotation/translation

    inline auto current_position() { return this->telemetry->position_velocity_ned().position; }
    inline auto current_velocity() { return this->telemetry->position_velocity_ned().velocity; }
    inline auto current_attitude() { return this->telemetry->attitude_euler_angle(); }

    inline static void check_action_result(mav::Action::Result result)
    {
        if (result != mav::Action::Result::SUCCESS)
            log_red_and_exit(mav::Action::result_str(result));
    }

    inline static void check_offboard_result(mav::Offboard::Result result)
    {
        if (result != mav::Offboard::Result::SUCCESS)
            log_red_and_exit(mav::Offboard::result_str(result));
    }

    inline static void check_connection_result(mav::ConnectionResult result)
    {
        if (result != mav::ConnectionResult::SUCCESS)
            log_red_and_exit(mav::connection_result_str(result));
    };

    inline static void log(const std::string& message) { std::cout << "[Chase2D] " << message << std::endl; }

    inline static void log_green(const std::string& message)
    {
        std::cout << CLI_COLOR_GREEN << "[Chase2D] " << message << CLI_COLOR_NORMAL << std::endl;
    }

    inline static void log_yellow(const std::string& message)
    {
        std::cout << CLI_COLOR_YELLOW << "[Chase2D] " << message << CLI_COLOR_NORMAL << std::endl;
    }

    inline static void log_red_and_exit(const std::string& message)
    {
        std::cout << CLI_COLOR_RED << "[Chase2D] " << message << CLI_COLOR_NORMAL << std::endl;
        exit(EXIT_FAILURE);
    }
};

#endif // PROJECT_CHASE_CHASE2D_H
