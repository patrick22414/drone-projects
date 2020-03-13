#include "chase2d.h"

Chase2D::Chase2D(
    int                       camera_index,
    const std::string&        connection,
    const std::vector<float>& speeds,
    const std::string&        video_output) :
    Chase2D(connection)
{
#ifdef __linux__
    capture.open(camera_index, cv::CAP_V4L);
#else // macOS or Win
    capture.open(camera_index);
#endif

    if (!capture.isOpened())
        log_red_and_exit("Cannot open camera");

    if (!capture.read(frame))
        log_red_and_exit("Cannot read frame");

    log_green("Video capture opened");

    if (!video_output.empty()) {
        auto codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        auto fps   = 30.0;

        writer.open(video_output, codec, fps, frame.size());

        if (!writer.isOpened())
            log_red_and_exit("Cannot open video output to write");

        log_green(fmt::format(FMT_STRING("Video writer opened at {}"), video_output));
    }

    f_speed = speeds[0];
    v_speed = speeds[1];
    y_speed = speeds[2];

    this->resolution = {frame.size().width, frame.size().height};
}

Chase2D::Chase2D(
    const std::string&        video_input,
    const std::string&        connection,
    const std::vector<float>& speeds,
    const std::string&        video_output) :
    Chase2D(connection)
{
#ifdef __linux__
    capture.open(video_input, cv::CAP_V4L);
#else // macOS or Win
    capture.open(video_input);
#endif

    if (!capture.isOpened())
        log_red_and_exit("Cannot open video input");

    if (!capture.read(frame))
        log_red_and_exit("Cannot read frame");

    log_green("Video capture opened");

    video_fps = capture.get(cv::CAP_PROP_FPS);

    if (!video_output.empty()) {
        auto codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        auto fps   = video_fps;

        writer.open(video_output, codec, fps, frame.size());

        if (!writer.isOpened())
            log_red_and_exit("Cannot open video output to write");

        log_green(fmt::format(FMT_STRING("Video writer opened at {}"), video_output));
    }

    f_speed = speeds[0];
    v_speed = speeds[1];
    y_speed = speeds[2];

    this->resolution = {frame.size().width, frame.size().height};
}

Chase2D::Chase2D(const std::string& connection)
{
#ifdef WITH_DRONE
    // Private constructor to init MAVSDK connection
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
#else
    log_yellow("Connection not available");
#endif
}

void Chase2D::start()
{
    log_green("Started");

    while (true) {
        if (!capture.read(frame))
            log_red_and_exit("Cannot read frame or end of video");

        cv::flip(frame, frame, -1);
        cv::imshow("Press Space to select ROI", frame);

        if (cv::waitKey(1) == ' ') {
            tracker_roi = cv::selectROI("Press Space to select ROI", frame);

            if (tracker_roi.x == 0 && tracker_roi.y == 0)
                continue;

            if (tracker_roi.width == 0 || tracker_roi.height == 0) {
                tracker_roi.x -= 10;
                tracker_roi.y -= 10;
                tracker_roi.width  = 20;
                tracker_roi.height = 20;
            }

            tracker->init(frame, tracker_roi);
            break;
        }

        if (video_fps > 0)
            sleep_for(milliseconds(static_cast<int>(1000 / video_fps)));
    }

    cv::destroyAllWindows();

#ifdef WITH_DRONE
    // Must set one offboard command before start offboard
    offboard->set_velocity_body({0, 0, 0, 0});

    auto start_result = offboard->start();
    check_offboard_result(start_result);

    log("Start: offboard started");
#endif
}

void Chase2D::update()
{
    // Recording section
    if (!capture.read(frame))
        log_red_and_exit("Cannot read frame or end of video");

    cv::flip(frame, frame, -1);

    // Tracking section
    bool target_acquired = tracker->update(frame, tracker_roi);

    if (target_acquired) {
        cv::rectangle(frame, tracker_roi, {255, 0, 0}, 2);
        log(fmt::format(
            FMT_STRING("Tracker ROI: [{:.0f}x{:.0f} from {:.0f},{:.0f}]"),
            tracker_roi.width,
            tracker_roi.height,
            tracker_roi.x,
            tracker_roi.y));

        if (writer.isOpened())
            writer.write(frame);

        eg::Vector2f position_i(tracker_roi.x + tracker_roi.width / 2, tracker_roi.y + tracker_roi.height / 2);

        // Chasing section
        float xc = position_i[0] - resolution[0] / 2.0f;
        float yc = position_i[1] - resolution[1] / 2.0f;

        float safe_area = resolution[1] / 10.0f;

        float chase_forward = f_speed;
        float chase_down    = abs(yc) < safe_area ? 0 : (yc > 0 ? v_speed : -v_speed);
        float chase_yaw     = abs(xc) < safe_area ? 0 : (xc > 0 ? y_speed : -y_speed);

        log_green(fmt::format(
            FMT_STRING("Chasing: FORWARD: {: .1f} m/s, DOWN: {: .1f} m/s, RIGHT: {: .1f} deg/s"),
            chase_forward,
            chase_down,
            chase_yaw));

#ifdef WITH_DRONE
        offboard->set_velocity_body({chase_forward, 0, chase_down, chase_yaw});
#endif
    } else {
        cv::rectangle(frame, tracker_roi, {0, 255, 255}, 2);
        log_yellow("Tracker target lost");
    }

    cv::imshow("Tracking", frame);
}

void Chase2D::stop()
{
#ifdef WITH_DRONE
    auto stop_result = offboard->stop();
    check_offboard_result(stop_result);

    log("Stop: offboard stopped");

    auto rtl_result = action->return_to_launch();
    check_action_result(rtl_result);

    while (telemetry->in_air()) {
        log("Vehicle is landing...");
        sleep_for(seconds(2));
    }
#endif

    release();
    log_green("Stopped");
}

void Chase2D::release()
{
    frame.release();
    tracker.release();
    capture.release();
    writer.release();
}
