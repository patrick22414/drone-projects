#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

void test_capture()
{
    VideoCapture v(0);
    if (!v.isOpened()) {
        cerr << "Cannot open camera" << endl;
        exit(-1);
    }

    v.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G') );
    v.set(CAP_PROP_FRAME_WIDTH, 1296);
    v.set(CAP_PROP_FRAME_HEIGHT, 972);

    Mat frame;
    steady_clock::time_point start = steady_clock::now();
    steady_clock::time_point end;
    nanoseconds interval;
    double avg_fps = 0;

    while (true) {
        end = steady_clock::now();

        if (!v.read(frame)) {
            cerr << "Cannot read frame" << endl;
            exit(-1);
        }

        interval = duration_cast<nanoseconds>(end - start);
        double fps = 1e9 / interval.count();
        avg_fps = 0.9 * avg_fps + 0.1 * fps;

        cout << "Frame interval: " << interval.count() * 1e-6 << " ms" << endl;
        cout << "Average FPS: " << avg_fps << endl;

        start = end;

        imshow("frame", frame);

        // Pause/Start/Stop with space/esc
        int key = waitKey(1);
        if (key == '\x1b') {
            break;
        } else if (key == ' ') {
            key = waitKey(0);
            if (key == '\x1b') {
                break;
            } else if (key == ' ') {
                continue;
            }
        }
    }

    v.release();
}

void test_write()
{
    // Prepare VideoCapture
    VideoCapture v = VideoCapture(0);

    v.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G') );
    v.set(CAP_PROP_FRAME_WIDTH, 1296);
    v.set(CAP_PROP_FRAME_HEIGHT, 972);

    if (!v.isOpened()) {
        cerr << "Cannot open camera" << endl;
        exit(-1);
    }

    Mat im;
    if (!v.read(im)) {
        cerr << "Cannot read frame" << endl;
        exit(-1);
    }

    bool isColor = im.type() == CV_8UC3;

    // Prepare VideoWriter
    string filename = "/home/yang/demo.avi";
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    double fps = 30;

    VideoWriter w = VideoWriter();
    w.open(filename, codec, fps, im.size(), isColor);
    if (!w.isOpened()) {
        cerr << "Cannot open output video file" << endl;
        exit(-1);
    }

    // Get all frames
    const int nFrames = 300;
    vector<steady_clock::time_point> timestamps(nFrames);
    for (int i = 0; i < nFrames; i++) {
        timestamps.emplace_back();
    }

    for (int i = 0; i < nFrames; i++) {
        timestamps[i] = steady_clock::now();
        if (!v.read(im)) {
            cerr << "Cannot read frame to write" << endl;
            exit(-1);
        }

        w.write(im);
    }

}

int main()
{
    test_write();
}