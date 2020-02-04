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

int main()
{
    VideoCapture v(0);
    if (!v.isOpened()) {
        cerr << "Cannot open camera" << endl;
        return -1;
    }

    v.set(CAP_PROP_FRAME_WIDTH, 640);
    v.set(CAP_PROP_FRAME_HEIGHT, 480);

    Mat frame;
    steady_clock::time_point start = steady_clock::now();
    steady_clock::time_point end;
    nanoseconds interval;
    double avg_fps = 0;

    while (true) {
        if (!v.read(frame)) {
            cerr << "Cannot read frame" << endl;
            return -1;
        }

        end = steady_clock::now();

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

    return 0;
}
