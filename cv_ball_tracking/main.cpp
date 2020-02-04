#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

int main()
{
    VideoCapture v = VideoCapture("/home/yang/Videos/ball-5.mp4");

    if (!v.isOpened()) {
        cerr << "Cannot open capture" << endl;
        return -1;
    }

    Mat im;
    Mat im_grey;
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    while (true) {
        if (!v.read(im)) {
            cerr << "Cannot read frame" << endl;
            break;
        }

        cvtColor(im, im_grey, COLOR_BGR2GRAY);

        threshold(im_grey, im_grey, 90, 255, THRESH_BINARY_INV);
//        morphologyEx(im_grey, im_grey, MORPH_DILATE, element);

        vector<vector<Point2i>> contours;
        vector<Vec4i> hierarchy;
        findContours(im_grey, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            for (const auto& contour : contours) {

                Moments m = moments(contour);

                double area = m.m00;

                if (area > 10) {
                    auto rotated_rect = fitEllipse(contour);
                    Point2f vertices[4];
                    rotated_rect.points(vertices);
                    for (int i = 0; i < 4; i++)
                        line(im, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 1);

                    double x = m.m10 / area;
                    double y = m.m01 / area;
                    double r = sqrt(area / CV_PI);

                    cout << "Area: " << area << endl;
                    
                    circle(im, Point(x, y), r, Scalar(255, 0, 0), 1);
                }
            }
        }

        imshow("frame", im);

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

        sleep_for(milliseconds(50));
    }

    v.release();

    return 0;
}
