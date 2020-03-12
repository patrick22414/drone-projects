#ifndef PROJECT_CHASE_CHASE2D_H
#define PROJECT_CHASE_CHASE2D_H

#define CLI_COLOR_RED "\033[31m"    // Turn text on console red
#define CLI_COLOR_GREEN "\033[32m"  // Turn text on console red
#define CLI_COLOR_YELLOW "\033[33m" // Turn text on console red
#define CLI_COLOR_NORMAL "\033[0m"  // Restore normal console colour

#include <Eigen/Dense>
#include <fmt/format.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include <chrono>
#include <mutex>
#include <thread>

namespace mav = mavsdk;
namespace eg  = Eigen;

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

class Chase2D {
public:
    Chase2D(
        int                       camera_index,
        const std::string&        connection,
        const std::vector<float>& speeds,
        const std::string&        video_output);

    Chase2D(
        const std::string&        video_input,
        const std::string&        connection,
        const std::vector<float>& speeds,
        const std::string&        video_output);

    ~Chase2D() { release(); }

    void start();

    void update();

    void stop();

private:
    explicit Chase2D(const std::string& connection);

    float f_speed = 0;
    float v_speed = 0;
    float y_speed = 0;

    cv::VideoCapture capture;
    cv::VideoWriter  writer;

    double video_fps = -1;

    eg::Vector2i resolution;

    cv::Mat frame;

    cv::Rect2d                tracker_roi;
    cv::Ptr<cv::TrackerMOSSE> tracker = cv::TrackerMOSSE::create();

#ifdef WITH_DRONE
    std::shared_ptr<mav::Action>    action;
    std::shared_ptr<mav::Offboard>  offboard;
    std::shared_ptr<mav::Telemetry> telemetry;
#endif

    void release();

    inline void check_action_result(mav::Action::Result result)
    {
        if (result != mav::Action::Result::SUCCESS)
            log_red_and_exit(mav::Action::result_str(result));
    }

    inline void check_offboard_result(mav::Offboard::Result result)
    {
        if (result != mav::Offboard::Result::SUCCESS)
            log_red_and_exit(mav::Offboard::result_str(result));
    }

    inline void check_connection_result(mav::ConnectionResult result)
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

    inline void log_red_and_exit(const std::string& message)
    {
        std::cout << CLI_COLOR_RED << "[Chase2D] " << message << CLI_COLOR_NORMAL << std::endl;

        release();

        exit(EXIT_FAILURE);
    }
};

#endif // PROJECT_CHASE_CHASE2D_H
