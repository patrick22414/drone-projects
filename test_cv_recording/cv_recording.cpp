#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

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
    auto filename = "./demo.mp4";
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

        // show live and wait for a key with timeout long enough to show images
        imshow("Live", src);
        if (waitKey(5) >= 0)
            break;
    }

    // the videofile will be closed and released automatically in VideoWriter destructor
    return 0;
}