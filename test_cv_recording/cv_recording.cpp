#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

using namespace cv;

namespace fs = std::filesystem;

std::string generate_video_filename(const std::string& prefix = "test-recording")
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

int main(int argc, char* argv[])
{
    Mat src;
    // use default camera as video source
    VideoCapture cap(0, CAP_V4L);
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_AUTO_WB, 0);
    cap.set(CAP_PROP_WB_TEMPERATURE, 6000);

    // check if we succeeded
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return EXIT_FAILURE;
    }

    // get one frame from camera to know frame size and type
    cap >> src;
    // check if we succeeded
    if (src.empty()) {
        std::cerr << "ERROR! blank frame grabbed\n";
        return EXIT_FAILURE;
    }

    bool isColor = (src.type() == CV_8UC3);

    //--- INITIALIZE VIDEOWRITER
    VideoWriter writer;
    auto codec    = VideoWriter::fourcc('m', 'p', '4', 'v');
    auto fps      = 30.0;
    auto filename = argc > 1 ? generate_video_filename(argv[1]) : generate_video_filename();

    writer.open(filename, codec, fps, src.size(), isColor);

    // check if we succeeded
    if (!writer.isOpened()) {
        std::cerr << "Could not open the output video file for write\n";
        return EXIT_FAILURE;
    }

    //--- GRAB AND WRITE LOOP
    auto total_time = 3;
    std::cout << "Writing video file: " << filename << std::endl << "Press any key to terminate" << std::endl;
    for (int i = 0; i < total_time * fps; ++i) {
        // check if we succeeded
        if (!cap.read(src)) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        // encode the frame into the video file stream
        writer.write(src);

        imshow("live", src);
        waitKey(1);
    }

    // the video file will be closed and released automatically in VideoWriter destructor
    return 0;
}