#include "cxxopts.hpp"
#include "Chase2D.h"

#include <X11/Xlib.h>

int main(int argc, char* argv[])
{
    const CameraProfile pi_camera_v1(0, 3.60, {3.67, 2.74});
    const CameraProfile pi_camera_v2(0, 3.04, {3.674, 2.760});

    XInitThreads();

    cxxopts::Options options("chase", "Project Chase");

    // clang-format off
    options.add_options()
        ("connection", "Connection URL",
            cxxopts::value<std::string>()->default_value("serial:///dev/serial0:921600"))
        ("h,help", "Print this help message",
            cxxopts::value<bool>())
        ("s,speeds","Forward, vertical, and yaw chasing speed (m/s, m/s, deg/s)",
            cxxopts::value<std::vector<float>>()->default_value("2,1,15"))
        ("i,input", "Input video file",
             cxxopts::value<std::string>())
        ("o,output", "Output video file",
             cxxopts::value<std::string>());
    // clang-format on

    options.parse_positional({"connection"});

    auto args = options.parse(argc, argv);

    if (args.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    auto arg_speeds     = args["speeds"].as<std::vector<float>>();
    auto arg_connection = args["connection"].as<std::string>();

    std::cout << "Command-line arguments:" << std::endl;
    std::cout << "  forward speed  : " << arg_speeds[0] << " m/s" << std::endl;
    std::cout << "  vertical speed : " << arg_speeds[1] << " m/s" << std::endl;
    std::cout << "  yaw speed      : " << arg_speeds[2] << " deg/s" << std::endl;
    std::cout << "  connection     : " << arg_connection << std::endl;

    Chase2D chase(arg_connection, pi_camera_v2);

    chase.start(arg_speeds);

    while (true) {
        if (cv::waitKey(1) == 27)
            break;
    }

    chase.stop();

    return 0;
}
