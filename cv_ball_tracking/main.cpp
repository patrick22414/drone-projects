#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

double calcDistance(double diameter_px)
{
    const double focal_len = 3.6e-3; // focal length 3.6mm
    const double pix_size = 1.4e-6 * 2; // pixel size 1.4um, binning 2x2
    const double tennis_size = 6.6e-2; // tennis ball size 6.6cm
    double d_m = tennis_size * focal_len / (diameter_px * pix_size);

    return d_m;
}

vector<double> solveXY(double u_px, double v_px)
{
    const double focal_len = 3.6e-3; // focal length 3.6mm
    const double pix_size = 1.4e-6 * 2; // pixel size 1.4um, binning 2x2

    double intrinsic[3][3] = {
        {focal_len, 0, 0.5 * 640 * pix_size},
        {0, focal_len, 0.5 * 480 * pix_size},
        {0, 0, 1},
    };

    double object_i_homo[3] = {u_px * pix_size, v_px * pix_size, 1};

    Mat A = Mat(3, 3, CV_64FC1, intrinsic);
    Mat b = Mat(3, 1, CV_64FC1, object_i_homo);
    Mat x = Mat(3, 1, CV_64FC1);
    solve(A, b, x, DECOMP_SVD);

    vector<double> object_c_homo;
    x.copyTo(object_c_homo);

    return object_c_homo;
}

int main()
{
    VideoCapture v = VideoCapture("/home/yang/Videos/ball-5.mp4");

    if (!v.isOpened()) {
        cerr << "Cannot open capture" << endl;
        return -1;
    }

    Mat im;
    Mat imGrey;
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    while (true) {
        if (!v.read(im)) {
            cerr << "Cannot read frame" << endl;
            break;
        }

        cvtColor(im, imGrey, COLOR_BGR2GRAY);

        threshold(imGrey, imGrey, 90, 255, THRESH_BINARY_INV);
        //        morphologyEx(im_grey, im_grey, MORPH_DILATE, element);

        vector<vector<Point2i>> contours;
        vector<Vec4i> hierarchy;
        findContours(imGrey, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            for (const auto& contour : contours) {
                Moments m = moments(contour);

                double area = m.m00;

                if (area > 100) {
                    auto rotatedRect = fitEllipse(contour);
                    Point2f vertices[4];
                    rotatedRect.points(vertices);
                    for (int i = 0; i < 4; i++)
                        line(im, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 1);

                    double xi = m.m10 / area;
                    double yi = m.m01 / area;
                    double ri = sqrt(area / CV_PI);

                    auto objectCameraHomo = solveXY(xi, yi);
                    double xc = objectCameraHomo[0];
                    double yc = objectCameraHomo[1];

                    double distance = calcDistance(ri * 2);
                    double zc = sqrt(distance * distance - xc * xc - yc * yc);

                    printf("(%f, %f, %f)\n", xc, yc, zc);

                    circle(im, Point(xi, yi), ri, Scalar(255, 0, 0), 1);
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

        //        sleep_for(milliseconds(50));
    }

    v.release();

    return 0;
}
