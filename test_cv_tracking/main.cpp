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

    bool found_ball_last_time = false;

    const Scalar lower_orange_1 = {0, 28, 0};
    const Scalar upper_orange_1 = {60, 255, 255};

    const Scalar lower_orange_2 = {108, 28, 0};
    const Scalar upper_orange_2 = {180, 255, 255};

    Mat im_hsv;
    Mat im_bin_1;
    Mat im_bin_2;

    // Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

    vector<Vec3d> circles(0);

    double xi = -1;
    double yi = -1;
    double ri = -1;

    while (true) {
        if (!v.read(im)) {
            cerr << "Cannot read frame or end of video" << endl;
            break;
        }

        cvtColor(im, im_hsv, COLOR_BGR2HSV);

        inRange(im_hsv, lower_orange_1, upper_orange_1, im_bin_1);
        inRange(im_hsv, lower_orange_2, upper_orange_2, im_bin_2);

        bitwise_or(im_bin_1, im_bin_2, im_bin_1);

        // morphologyEx(im_bin_1, im_bin_1, MORPH_CLOSE, element);

        vector<vector<Point2i>> contours;
        vector<Vec4i> hierarchy;
        findContours(im_bin_1, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            sort(contours.begin(), contours.end(), [](const vector<Point2i>& c1, const vector<Point2i>& c2) {
                return contourArea(c1, false) > contourArea(c2, false);
            });

            auto largestContour = contours[0];

            auto m = moments(largestContour);

            double area = m.m00;

            // Use momentum or not
            if (false) {
                xi = xi < 0 ? (m.m10 / area) : (0.8 * xi + 0.2 * m.m10 / area);
                yi = yi < 0 ? (m.m01 / area) : (0.8 * yi + 0.2 * m.m01 / area);
                ri = ri < 0 ? sqrt(area / CV_PI) : (0.8 * ri + 0.2 * sqrt(area / CV_PI));
            } else {
                xi = m.m10 / area;
                yi = m.m01 / area;
                ri = sqrt(area / CV_PI);
            }

            if (ri > 6) {
                if (found_ball_last_time) {
                    circles.push_back({xi, yi, ri});
                } else {
                    found_ball_last_time = true;
                }
            } else {
                found_ball_last_time = false;
            }
        } else {
            found_ball_last_time = false;
        }

        for (int i = 1; i < (int)circles.size(); i++) {
            int x = (int)circles[i][0];
            int y = (int)circles[i][1];
            int r = (int)circles[i][2];
            circle(im, {x, y}, r, {0, 225, 255}, 1);
        }

        imshow("image", im);
        imshow("image binary", im_bin_1);

        auto key = waitKey(33);
        if (key == '\x1b') {
            break;
        } else if (key == ' ') {
            key = waitKey(-1);
            if (key == '\x1b') {
                break;
            } else if (key == ' ') {
                continue;
            }
        }
    }

    destroyAllWindows();

    return 0;
}