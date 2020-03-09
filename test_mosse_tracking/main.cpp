#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main(int argc, char** argv)
{
    // show help
    if (argc < 2) {
        std::cout << " Usage: tracker <video_name>\n"
                     " examples:\n"
                     " example_tracking_kcf Bolt/img/%04d.jpg\n"
                     " example_tracking_kcf faceocc2.webm\n"
                  << std::endl;
        return 0;
    }

    // declares all required variables
    cv::Rect2d roi;
    cv::Mat frame;

    // create a tracker object
    cv::Ptr<cv::Tracker> tracker = cv::TrackerMOSSE::create();

    // set input video
    std::string video = argv[1];
    cv::VideoCapture capture(video);

    // get bounding box
    capture.read(frame);
    cv::flip(frame, frame, -1);
    roi = cv::selectROI("tracker", frame);

    std::cout << "Selected ROI 1: " << roi << std::endl;

    if (roi.width == 0 || roi.height == 0) {
        roi.x -= 8;
        roi.y -= 8;
        roi.width  = 17;
        roi.height = 17;
    }

    std::cout << "Selected ROI 2: " << roi << std::endl;

    // initialize the tracker
    tracker->init(frame, roi);

    // perform the tracking process
    printf("Start the tracking process, press ESC to quit.\n");
    while (true) {
        // get frame from the video
        capture.read(frame);
        cv::flip(frame, frame, -1);

        // stop the program if no more images
        if (frame.rows == 0 || frame.cols == 0)
            break;

        // update the tracking result
        tracker->update(frame, roi);

        // draw the tracked object
        cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2, 1);

        // show image with the tracked object
        cv::imshow("tracker", frame);

        // quit on ESC button
        if (cv::waitKey(33) == 27)
            break;
    }

    return 0;
}
