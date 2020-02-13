#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

optional<char*> getArg(const string& name, int argc, char* argv[])
{
    for (int i = 1; i < argc; i++) {
        if (name == argv[i]) {
            return argv[i + 1];
        }
    }

    return {};
}

int main(int argc, char* argv[])
{
    Mat im;
    VideoCapture v;

    if (getArg("-v", argc, argv).has_value()) {
        v = VideoCapture(getArg("-v", argc, argv).value());
    } else {
        v = VideoCapture(0, CAP_V4L2);
        v.set(CAP_PROP_FRAME_WIDTH, 640);
        v.set(CAP_PROP_FRAME_HEIGHT, 480);
    }

    Mat im_neg;
    Mat im_hsv;
    Mat im_bin;

    bool found_ball_last_time = false;

    const Scalar lower_blue = {71, 0, 0};
    const Scalar upper_blue = {120, 255, 255};

    while (true) {
        if (!v.read(im)) {
            cerr << "Cannot read frame or end of video" << endl;
            break;
        }

        bitwise_not(im, im_neg);
        cvtColor(im_neg, im_hsv, COLOR_BGR2HSV);

        inRange(im_hsv, lower_blue, upper_blue, im_bin);

        vector<vector<Point2i>> contours;
        vector<Vec4i> hierarchy;
        findContours(im_bin, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            sort(contours.begin(), contours.end(), [](const vector<Point2i>& c1, const vector<Point2i>& c2) {
                return contourArea(c1, false) > contourArea(c2, false);
            });

            auto largestContour = contours[0];

            auto m = moments(largestContour);

            double area = m.m00;
            double xi   = 0;
            double yi   = 0;
            double ri   = 0;
            if (area > 100) {
                if (found_ball_last_time) {
                    Point2f center;
                    float radius;
                    minEnclosingCircle(largestContour, center, radius);

                    circle(im, {(int)center.x, (int)center.y}, (int)radius, {0, 0, 255}, 1);

                    xi = m.m10 / area;
                    yi = m.m01 / area;
                    ri = sqrt(area / CV_PI);

                    cout << "find ball at (" << xi << ", " << yi << ")" << endl;

                    circle(im, {(int)xi, (int)yi}, (int)ri, {255, 0, 0}, 1);
                } else {
                    found_ball_last_time = true;
                }
            } else {
                found_ball_last_time = false;
            }
        } else {
            found_ball_last_time = false;
        }

        imshow("image", im);
        imshow("image binary", im_bin);

        if (waitKey(33) >= 0) {
            break;
        }
    }

    destroyAllWindows();

    return 0;
}
