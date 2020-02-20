#include <opencv2/opencv.hpp>

#define CLI_COLOR_RED "\033[31m"   // Turn text on console red
#define CLI_COLOR_GREEN "\033[32m" // Turn text on console red
#define CLI_COLOR_NORMAL "\033[0m" // Restore normal console colour

cv::Vec3f invert_camera_transform(float ball_x_px, float ball_y_px, float ball_radius_px, const cv::Vec2i& resolution);

int main()
{
    const cv::Vec2i resolution = {640, 480};

    cv::VideoCapture capture(0, cv::CAP_V4L2);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, resolution[0]);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, resolution[1]);

    if (!capture.isOpened()) {
        std::cerr << CLI_COLOR_RED << "Cannot open video capture" << CLI_COLOR_NORMAL << std::endl;
        exit(-1);
    }

    const cv::Scalar lower_orange = {0, 28, 0};
    const cv::Scalar upper_orange = {60, 255, 100};

    cv::Mat im;
    cv::Mat im_hsv;
    cv::Mat im_bin;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    while (true) {
        if (!capture.read(im)) {
            std::cerr << "Cannot read frame or end of video" << std::endl;
            break;
        }

        // Convert to HSV color space
        cv::cvtColor(im, im_hsv, cv::COLOR_BGR2HSV);

        // Filter color within range
        cv::inRange(im_hsv, lower_orange, upper_orange, im_bin);

        cv::morphologyEx(im_bin, im_bin, cv::MORPH_OPEN, element);
        cv::morphologyEx(im_bin, im_bin, cv::MORPH_DILATE, element);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(im_bin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find largest contour by area
            std::sort(
                contours.begin(),
                contours.end(),
                [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                    return cv::contourArea(c1, false) > cv::contourArea(c2, false);
                });

            auto largest_contour = contours[0];

            auto center = cv::Point2f();
            auto radius = 0.0f;
            cv::minEnclosingCircle(largest_contour, center, radius);

            // Count as a ball if the blob has a large enough radius
            if (radius > 10) {
                auto c = invert_camera_transform(center.x, center.y, radius, resolution) * 100;

                char text[99];
                std::sprintf(text, "[% d,% d,% d]", (int) c[0], (int) c[1], (int) c[2]);

                cv::circle(im, cv::Point(center), radius, {255, 255, 0}, 2);

                cv::Point text_center(center.x - 60, center.y);
                cv::putText(im, std::string(text), text_center, cv::FONT_HERSHEY_COMPLEX, 0.6, {0, 0, 0}, 3);
                cv::putText(im, std::string(text), text_center, cv::FONT_HERSHEY_COMPLEX, 0.6, {255, 255, 0});

                cv::imshow("im_bin", im_bin);
                cv::imshow("im", im);
            }

            // Wait for half a second
            auto key = cv::waitKey(100);

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
    }

    return 0;
}

cv::Vec3f invert_camera_transform(float ball_x_px, float ball_y_px, float ball_radius_px, const cv::Vec2i& resolution)
{
    const float ball_radius = 1e-2 * 2.5 * 2.54; // tiny basketball radius 2.5 inches

    // Pi Camera V1 parameters
    const float sensor_w_px     = 2592;   // sensor width in pixels
    const float sensor_h_px     = 1944;   // sensor height in pixels
    const float focal_length    = 3.6e-3; // focal length 3.6mm
    const float real_pixel_size = 1.4e-6; // real pixel size 1.4um

    float res_w_px = resolution[0];
    float res_h_px = resolution[1];

    float pix_size = real_pixel_size * (sensor_w_px / res_w_px); // relative pixel size with binning

    float ball_i_x = ball_x_px * pix_size; // ball position x, Image frame
    float ball_i_y = ball_y_px * pix_size; // ball position y, Image frame

    // camera intrinsic matrix
    // clang-format off
    float intrinsics[9] = {
        focal_length,            0, 0.5f * (res_w_px - 1) * pix_size,
        0, focal_length, 0.5f * (res_h_px - 1) * pix_size,
        0,            0,                                1,
    };
    // clang-format on

    // object homogeneous coordinates, Image frame (z = 1)
    float object_i_homo[3] = {ball_i_x, ball_i_y, 1};

    cv::Mat A(3, 3, CV_32F, intrinsics);
    cv::Mat b(3, 1, CV_32F, object_i_homo);
    cv::Mat x(3, 1, CV_32F);
    solve(A, b, x, cv::DECOMP_SVD);

    // ray direction, expressed by (0, 0, 0) -> (xc1, yc1, 1)
    float xc1 = x.at<float>(0);
    float yc1 = x.at<float>(1);

    // distance from camera to ball
    float distance = ball_radius * sqrt(focal_length * focal_length + ball_i_x * ball_i_x + ball_i_y * ball_i_y) /
                     (ball_radius_px * pix_size);

    // distance from camera to (xc1, yc1, 1)
    float distance1 = sqrt(xc1 * xc1 + yc1 * yc1 + 1);

    // object coordinates, Camera frame
    float xc = xc1 * distance / distance1;
    float yc = yc1 * distance / distance1;
    float zc = sqrt(distance * distance - xc * xc - yc * yc);

    return {xc, yc, zc};
}
