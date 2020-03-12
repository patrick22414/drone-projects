#include "chase2d.h"

#include <cxxopts.hpp>
#include <filesystem>

namespace fs = std::filesystem;

std::string generate_video_filename(const std::string& prefix = "chase")
{
    fs::path    folder = fs::path(getenv("HOME")) / "Videos";
    std::string filename;
    for (int i = 1;; ++i) {
        filename = fmt::format(FMT_STRING("{}/{}-v{}.mp4"), folder.string(), prefix, i);

        if (!fs::exists(filename)) {
            break;
        }
    }

    std::cout << "Generated video filename: " << filename << std::endl;

    return filename;
}

int main(int argc, char* argv[])
{
    cxxopts::Options options("chase", "Project Chase");

    // clang-format off
    options.add_options()
        ("connection", "Connection URL",
            cxxopts::value<std::string>()->default_value("serial:///dev/serial0:921600"))
        ("h,help", "Print this help message",
            cxxopts::value<bool>())
        ("s,speeds","Forward, vertical, and yaw chasing speed (m/s, m/s, deg/s)",
            cxxopts::value<std::vector<float>>()->default_value("2,1,15"))
        ("m,camera", "Camera index",
            cxxopts::value<int>()->default_value("0"))
        ("i,input", "Input video file",
             cxxopts::value<std::string>()->default_value(""))
        ("o,output", "Output video file",
             cxxopts::value<std::string>()->default_value("")->implicit_value(generate_video_filename()));
    // clang-format on

    options.parse_positional({"connection"});

    auto args = options.parse(argc, argv);

    if (args.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    auto arg_speeds     = args["speeds"].as<std::vector<float>>();
    auto arg_camera     = args["camera"].as<int>();
    auto arg_input      = args["input"].as<std::string>();
    auto arg_output     = args["output"].as<std::string>();
    auto arg_connection = args["connection"].as<std::string>();

    std::cout << "Command-line arguments:" << std::endl
              << "  forward speed  : " << arg_speeds[0] << " m/s" << std::endl
              << "  vertical speed : " << arg_speeds[1] << " m/s" << std::endl
              << "  yaw speed      : " << arg_speeds[2] << " deg/s" << std::endl
              << "  camera index   : " << arg_camera << std::endl
              << "  input file     : " << arg_input << std::endl
              << "  output file    : " << arg_output << std::endl
              << "  connection     : " << arg_connection << std::endl;

    std::shared_ptr<Chase2D> chase;

    if (arg_input.empty())
        chase = std::make_shared<Chase2D>(arg_camera, arg_connection, arg_speeds, arg_output);
    else
        chase = std::make_shared<Chase2D>(arg_input, arg_connection, arg_speeds, arg_output);

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
