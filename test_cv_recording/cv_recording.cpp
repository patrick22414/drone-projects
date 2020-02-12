#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

using namespace cv;
using namespace std;

namespace fs = std::filesystem;

string generateVideoFilename(const string& suffix = "air")
{
    fs::path full_filename = fs::path(getenv("HOME")) / "Videos";
    stringstream filename;
    for (int i = 1;; ++i) {
        filename.str(string());
        filename << "v" << i << "-" << suffix << ".mp4";

        if (!fs::exists(fs::path(full_filename) / filename.str())) {
            break;
        }
    }

    cout << "Using video filename " << fs::path(full_filename) / filename.str() << endl;

    return fs::path(full_filename) / filename.str();
}

int main(int argc, char* argv[])
{
    Mat src;
    // use default camera as video source
    VideoCapture cap(0, CAP_V4L2);
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    // get one frame from camera to know frame size and type
    cap >> src;
    // check if we succeeded
    if (src.empty()) {
        cerr << "ERROR! blank frame grabbed\n";
        return -1;
    }

    bool isColor = (src.type() == CV_8UC3);

    //--- INITIALIZE VIDEOWRITER
    VideoWriter writer;
    auto codec    = VideoWriter::fourcc('m', 'p', '4', 'v');
    auto fps      = 30.0;
    auto filename = argc > 1 ? generateVideoFilename(argv[1]) : generateVideoFilename();

    writer.open(filename, codec, fps, src.size(), isColor);

    // check if we succeeded
    if (!writer.isOpened()) {
        cerr << "Could not open the output video file for write\n";
        return -1;
    }

    //--- GRAB AND WRITE LOOP
    auto total_time = 5;
    cout << "Writing video file: " << filename << endl << "Press any key to terminate" << endl;
    for (int i = 0; i < total_time * fps; ++i) {
        // check if we succeeded
        if (!cap.read(src)) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        // encode the frame into the video file stream
        writer.write(src);
    }

    // the video file will be closed and released automatically in VideoWriter destructor
    return 0;
}