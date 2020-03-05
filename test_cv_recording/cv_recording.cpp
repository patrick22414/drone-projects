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
        filename << prefix << "-v" << i << ".avi";

        if (!fs::exists(fs::path(full_filename) / filename.str())) {
            break;
        }
    }

    std::cout << "Using video filename " << fs::path(full_filename) / filename.str() << std::endl;

    return fs::path(full_filename) / filename.str();
}

int main(int argc, char* argv[])
{
    // use default camera as video source
    VideoCapture capture(0, CAP_V4L);
    capture.set(CAP_PROP_FRAME_WIDTH, 1640);
    capture.set(CAP_PROP_FRAME_HEIGHT, 1232);

    // check if we succeeded
    if (!capture.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return EXIT_FAILURE;
    }

    // get one frame from camera to know frame size and type
    Mat frame;
    capture >> frame;
    // check if we succeeded
    if (!capture.read(frame)) {
        std::cerr << "ERROR! Unable to grab frame\n";
        return EXIT_FAILURE;
    }

    bool isColor = (frame.type() == CV_8UC3);

    //--- INITIALIZE VIDEOWRITER
    VideoWriter writer;
    auto codec    = VideoWriter::fourcc('M', 'J', 'P', 'G');
    auto fps      = 25.0;
    auto filename = argc > 1 ? generate_video_filename(argv[1]) : generate_video_filename();

    writer.open(filename, codec, fps, frame.size(), isColor);

    // check if we succeeded
    if (!writer.isOpened()) {
        std::cerr << "Could not open the output video file for write\n";
        return EXIT_FAILURE;
    }

    //--- GRAB AND WRITE LOOP
    auto total_time = 5;
    std::cout << "Writing video file: " << filename << std::endl << "Press any key to terminate" << std::endl;
    for (int i = 0; i < total_time * fps; ++i) {
        // check if we succeeded
        if (!capture.read(frame)) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        // encode the frame into the video file stream
        writer.write(frame);

        imshow("live", frame);
        waitKey(1);
    }

    // the video file will be closed and released automatically in VideoWriter destructor
    return 0;
}