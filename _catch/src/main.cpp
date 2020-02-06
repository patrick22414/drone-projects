#include <opencv2/opencv.hpp>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace cv;
using namespace mavsdk;
using namespace std;
using namespace std::this_thread;
using namespace std::chrono;

void fitParabola(const vector<Vec4d>& points)
{
    // TODO: find the parameters of a parabola from a series of points in World frame
}

void worldTransform(const Vec3d& p, Telemetry::PositionNED t, Telemetry::EulerAngle r)
{
    // TODO: given a point in the Camera frame, transform the position into the World frame
}

Vec3d cameraTransform(const Vec2d& ball_position_px, double ball_radius_px)
{
    // TODO: given a point in the Image frame in pixels, and the distance from the camera to the
    // ball, find the position of the ball in the Camera frame
    return {0, 0, 0};
}

Vec4d getBallPositionNEDTime(VideoCapture v, Telemetry telemetry)
{
    Mat im;
    Mat im_grey;

    if (!v.read(im)) {
        cerr << "Cannot read frame" << endl;
        exit(-1);
    }

    cvtColor(im, im_grey, COLOR_BGR2GRAY);

    auto drone_position = telemetry.position_velocity_ned().position;
    auto drone_rotation = telemetry.attitude_euler_angle();

    auto timestamp = steady_clock::now();

    return {0, 0, 0, 0};
}

int main(int argc, char* argv[])
{
    const string keys =
        "{h help ?   |    | Print this message}"
        "{n n-frames | 10 | Maximum number of frames used to calculate the trajectory of the ball}"
        "{a altitude | -1 | Altitude used by the drone to catch the ball. Default -1 will remain at current altitude}";

    CommandLineParser parser(argc, argv, keys);
    parser.about("The main program that will catch the ball");

    if (argc == 1) {
        parser.printMessage();
        return 0;
    }

    auto n_frames = parser.get<unsigned>("n-frames");
    auto altitude = parser.get<int>("altitude");

    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    cout << "Get command line arg `n-frames`: " << n_frames << endl;
    cout << "Get command line arg `altitude`: " << altitude << endl;

    return 0;
}
