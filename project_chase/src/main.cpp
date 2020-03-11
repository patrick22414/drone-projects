#include "cxxopts.hpp"
#include "Chase2D.h"

int main(int argc, char* argv[])
{
    const CameraProfile pi_camera_v1(0, 3.60, {3.67, 2.74});
    const CameraProfile pi_camera_v2(0, 3.04, {3.674, 2.760});

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
             cxxopts::value<std::string>()->default_value(""))
        ("o,output", "Output video file",
             cxxopts::value<std::string>()->default_value("")->implicit_value("chase-v1.mp4"));
    // clang-format on

    options.parse_positional({"connection"});

    auto args = options.parse(argc, argv);

    if (args.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    auto arg_speeds     = args["speeds"].as<std::vector<float>>();
    auto arg_input      = args["input"].as<std::string>();
    auto arg_output     = args["output"].as<std::string>();
    auto arg_connection = args["connection"].as<std::string>();

    std::cout << "Command-line arguments:" << std::endl
              << "  forward speed  : " << arg_speeds[0] << " m/s" << std::endl
              << "  vertical speed : " << arg_speeds[1] << " m/s" << std::endl
              << "  yaw speed      : " << arg_speeds[2] << " deg/s" << std::endl
              << "  input file     : " << arg_input << std::endl
              << "  output file    : " << arg_output << std::endl
              << "  connection     : " << arg_connection << std::endl;

    std::shared_ptr<Chase2D> chase;

    if (arg_input.empty())
        chase = std::make_shared<Chase2D>(arg_connection, pi_camera_v2, arg_speeds, arg_output);
    else
        chase = std::make_shared<Chase2D>(arg_connection, arg_input, arg_speeds, arg_output);

    chase->start();

    while (true) {
        chase->update();

        auto key = cv::waitKey(1);
        if (key == ' ') {
            key = cv::waitKey(-1);
            if (key == ' ')
                continue;
            else if (key == '\x1b')
                break;
        } else if (key == '\x1b')
            break;

        sleep_for(milliseconds(50));
    }

    chase->stop();

    return 0;
}
