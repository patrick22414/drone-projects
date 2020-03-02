#include "Chase2D.h"

Chase2D::Chase2D(
    const std::string& connection, int camera, const eg::Vector2i& resolution, const std::string& video_output) :
    Chase2D(connection)
{
    capture = cv::VideoCapture(camera, cv::CAP_V4L);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, resolution[0]);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, resolution[1]);

    if (!capture.isOpened())
        log_red_and_exit("Cannot open camera");

    if (!capture.read(im))
        log_red_and_exit("Cannot read frame");

    if (!video_output.empty()) {
        auto codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        auto fps   = 30.0; // TODO

        writer = cv::VideoWriter(video_output, codec, fps, im.size());

        if (!writer.isOpened())
            log_red_and_exit("Cannot open video output to write");
    }

    this->resolution = resolution;
}

Chase2D::Chase2D(const std::string& connection, const std::string& video_input) : Chase2D(connection)
{
    capture = cv::VideoCapture(video_input);

    if (!capture.isOpened())
        log_red_and_exit("Cannot open video input");

    if (!capture.read(im))
        log_red_and_exit("Cannot read frame");

    this->resolution = {im.size().width, im.size().height};
}

// Private constructor to init MAVSDK connection
Chase2D::Chase2D(const std::string& connection)
{
    mav::Mavsdk mavsdk;

    auto connection_result = mavsdk.add_any_connection(connection);
    check_connection_result(connection_result);

    while (!mavsdk.is_connected()) {
        log_yellow("Waiting for vehicle to connect...");
        sleep_for(seconds(1));
    }

    mav::System& system = mavsdk.system();
    log_green("Vehicle connected");

    action    = std::make_shared<mav::Action>(system);
    offboard  = std::make_shared<mav::Offboard>(system);
    telemetry = std::make_shared<mav::Telemetry>(system);

    while (!telemetry->health_all_ok()) {
        log_yellow("Waiting for vehicle to be ready...");
        sleep_for(seconds(1));
    }

    log_green("Vehicle is ready to be armed");

    mav::Action::Result arm_result = action->arm();
    check_action_result(arm_result);
    log_green("Vehicle is Armed");

    log_green("Connection complete");
}

void Chase2D::start(float v_speed, float h_speed, bool show_live)
{
    is_recording     = true;
    recording_thread = std::thread(&Chase2D::recording_routine, this, show_live);

    is_tracking     = true;
    tracking_thread = std::thread(&Chase2D::tracking_routine, this, show_live);

    is_chasing     = true;
    chasing_thread = std::thread(&Chase2D::chasing_routine, this, v_speed, h_speed);
}

void Chase2D::recording_routine(bool show_live)
{
    // Recording routine records the raw camera frame to `im`, and optionally displays a live feed of the raw frame.
    log("Start recording routine");

    bool has_writer = writer.isOpened();

    while (is_recording) {
        new_timestamp = Clock::now();

        if (!capture.read(im))
            log_red_and_exit("Cannot read frame or end of video");

        if (has_writer)
            writer.write(im);

        if (show_live) {
            cv::imshow("Chase2D Recording Live", im);
            cv::waitKey(1);
        }
    }

    log("End recording routine");
}

void Chase2D::tracking_routine(bool show_live)
{
    // Tracking routine processes the raw image from `im`, finds the target in each coordinate system, in turn populates
    // `positions_i`, `positions_c`, and `positions_w`, and optionally displays a live feed of the processed image
    log("Start tracking routine");

    while (is_tracking) {
        if (new_timestamp > old_timestamp) {
            old_timestamp = new_timestamp;

            // TODO: Locate target in Image and put into positions_i
            positions_i.emplace_back();

            // TODO: Transform target to Camera and put into positions_c (fake homogeneous coordinates)
            positions_c.emplace_back();

            // TODO: Transform target to World and put into positions_w (fake homogeneous coordinates)
            positions_w.emplace_back();
        }
    }

    log("End tracking routine");
}

void Chase2D::chasing_routine(float v_speed, float h_speed)
{
    // Chasing routine takes the last known position of the target (`positions_w[-1]`) and current telemetry status of
    // the drone, determines the next velocity and gives it to the drone.
    log("Start chasing routine");

    while (is_chasing) {
        // TODO
    }

    log("End chasing routine");
}

void Chase2D::stop()
{
    is_recording = false;
    is_chasing   = true;

    recording_thread.join();
    chasing_thread.join();
}