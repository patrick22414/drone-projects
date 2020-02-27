#ifndef PROJECT_CHASE_CHASE2D_H
#define PROJECT_CHASE_CHASE2D_H

#include <Eigen/Dense>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/opencv.hpp>

using namespace Eigen;

class Chase2D {
    // TODO
public:
    Chase2D(const std::string& connection, int camera, cv::VideoCaptureAPIs api, const std::string& video_output = "");

    Chase2D(const std::string& connection, const std::string& video_input);

    std::list<Vector2f> positions_i; // target positions in Image frame
    std::list<Vector3f> positions_c; // target positions in Camera frame, `z` is always 1 since depth is unknown
    std::list<Vector3f> positions_w; // target positions in World frame, using NED coordinates

    cv::VideoCapture capture;
    cv::VideoWriter writer;

    static void log(const std::string& msg) { std::cout << "[Chase2D] " << msg << std::endl; }

private:
    std::shared_ptr<mavsdk::Action> action;
    std::shared_ptr<mavsdk::Offboard> offboard;
    std::shared_ptr<mavsdk::Telemetry> telemetry;

    std::list<cv::Mat> ims;
};

#endif // PROJECT_CHASE_CHASE2D_H
