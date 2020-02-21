#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#define CLI_COLOR_RED "\033[31m"   // Turn text on console red
#define CLI_COLOR_GREEN "\033[32m" // Turn text on console red
#define CLI_COLOR_NORMAL "\033[0m" // Restore normal console colour

namespace eg = Eigen;

eg::Vector3f
invert_camera_transform_eg(float ball_x_px, float ball_y_px, float ball_radius_px, const eg::Vector2i& resolution);

int main()
{
    const eg::Vector2i resolution(640, 480);

    cv::VideoCapture capture("/home/yang/Videos/v4-air.mp4");
    //    cv::VideoCapture capture(0, cv::CAP_V4L2);
    //    capture.set(cv::CAP_PROP_FRAME_WIDTH, resolution[0]);
    //    capture.set(cv::CAP_PROP_FRAME_HEIGHT, resolution[1]);

    if (!capture.isOpened()) {
        std::cerr << CLI_COLOR_RED << "Cannot open video capture" << CLI_COLOR_NORMAL << std::endl;
        exit(-1);
    }

    const cv::Scalar lower_orange = {0, 60, 0};
    const cv::Scalar upper_orange = {45, 255, 100};

    cv::Mat im_old;
    cv::Mat im;
    cv::Mat im_diff;
    cv::Mat im_diff_bin;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    if (!capture.read(im_old)) {
        std::cerr << "Cannot read frame or end of video" << std::endl;
    }

    while (true) {
        if (!capture.read(im)) {
            std::cerr << "Cannot read frame or end of video" << std::endl;
            break;
        }

        cv::absdiff(im, im_old, im_diff);
        im.copyTo(im_old);

        cv::threshold(im_diff, im_diff, 31, 255, cv::THRESH_BINARY);

        cv::cvtColor(im_diff, im_diff_bin, cv::COLOR_BGR2GRAY);
        cv::threshold(im_diff_bin, im_diff_bin, 0, 255, cv::THRESH_BINARY);

        cv::morphologyEx(im_diff_bin, im_diff_bin, cv::MORPH_OPEN, element);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(im_diff_bin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            std::vector<cv::Point> all_points;
            for (auto& contour : contours) {
                all_points.insert(all_points.end(), contour.begin(), contour.end());
            }

            auto rect = cv::minAreaRect(all_points);

            cv::Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; ++i) {
                cv::line(im, vertices[i], vertices[(i + 1) % 4], {255, 0, 0}, 1);
            }

            auto center = static_cast<cv::Point>(rect.center);
            auto radius = static_cast<int>(std::min(rect.size.width, rect.size.height) * 0.5 + 1);

            cv::circle(im, center, radius, {255, 0, 0}, 1);
        }

        cv::imshow("im_diff_bin", im_diff_bin);
        cv::imshow("im", im);

        // Wait for half a second
        auto key = cv::waitKey(33);

        if (key == '\x1b') {
            break;
        } else if (key == ' ') {
            key = cv::waitKey(-1);
            if (key == '\x1b') {
                break;
            } else if (key == ' ') {
                continue;
            }
        }
    }

    return 0;
}

eg::Vector3f
invert_camera_transform_eg(float ball_x_px, float ball_y_px, float ball_radius_px, const eg::Vector2i& resolution)
{
    const float ball_radius = 1e-2 * 2.5 * 2.54; // tiny basketball radius 2.5 inches

    // Pi Camera V1 parameters
    const float sensor_w_px   = 2592;   // sensor width in pixels
    const float sensor_h_px   = 1944;   // sensor height in pixels
    const float focal_length  = 3.6e-3; // focal length 3.6mm
    const float real_pix_size = 1.4e-6; // real pixel size 1.4um

    auto res_w_px = static_cast<float>(resolution(0));
    auto res_h_px = static_cast<float>(resolution(1));

    float pix_size = real_pix_size * (sensor_w_px / res_w_px); // relative pixel size with binning

    float ball_i_x = ball_x_px * pix_size; // ball position x, Image frame
    float ball_i_y = ball_y_px * pix_size; // ball position y, Image frame

    float z_c = ball_radius / (ball_radius_px * pix_size) * focal_length;

    eg::Matrix3f intrinsics;
    // clang-format off
    intrinsics << focal_length,            0, 0.5f * (res_w_px - 1) * pix_size,
        0, focal_length, 0.5f * (res_h_px - 1) * pix_size,
        0,            0,                                1;
    // clang-format on

    eg::Vector3f target_i;
    target_i << ball_i_x, ball_i_y, z_c;

    eg::Vector3f target_c = intrinsics.jacobiSvd(eg::ComputeFullU | eg::ComputeFullV).solve(target_i);

    return target_c;
}
