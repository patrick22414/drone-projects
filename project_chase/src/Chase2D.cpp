#include "Chase2D.h"

Chase2D::Chase2D(const std::string& connection, const CameraProfile& camera, const std::string& video_output) :
    Chase2D(connection)
{
    this->resolution = camera.resolution;
    this->intrinsics = camera.intrinsics;

    capture = cv::VideoCapture(camera.index, cv::CAP_V4L);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, camera.resolution[0]);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, camera.resolution[1]);

    if (!capture.isOpened())
        log_red_and_exit("Cannot open camera");

    if (!capture.read(frame))
        log_red_and_exit("Cannot read frame");

    if (!video_output.empty()) {
        auto codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        auto fps   = 30.0; // TODO

        writer = cv::VideoWriter(video_output, codec, fps, frame.size());

        if (!writer.isOpened())
            log_red_and_exit("Cannot open video output to write");
    }
}

Chase2D::Chase2D(const std::string& connection, const std::string& video_input) : Chase2D(connection)
{
    capture = cv::VideoCapture(video_input);

    if (!capture.isOpened())
        log_red_and_exit("Cannot open video input");

    if (!capture.read(frame))
        log_red_and_exit("Cannot read frame");

    this->resolution = {frame.size().width, frame.size().height};
}

Chase2D::Chase2D(const std::string& connection)
{
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
    // Recording routine records the raw camera frame to `frame`, and optionally displays a live feed of the raw frame.
    log("Start recording routine");

    bool has_writer = writer.isOpened();

    while (is_recording) {
        mutex.lock(); // CRITICAL SECTION

        frame_timestamp = Clock::now();
        frame_position  = current_position();
        frame_attitude  = current_attitude();

        if (!capture.read(frame))
            log_red_and_exit("Cannot read frame or end of video");

        mutex.unlock(); // END CRITICAL SECTION

        if (has_writer)
            writer.write(frame);

        if (show_live) {
            cv::imshow("Chase2D Recording Live", frame);
            cv::waitKey(1);
        }
    }

    log("End recording routine");
}

void Chase2D::tracking_routine(bool show_live)
{
    // Tracking routine processes the raw image from `frame`, finds the target in each coordinate system, in turn
    // populates `positions_i`, `positions_c`, and `positions_w`, and optionally displays a live feed of the processed
    // image
    log("Start tracking routine");

    cv::Mat im;
    cv::Mat im_grey;

    while (is_tracking) {
        if (frame_timestamp > tracked_timestamp) {
            mutex.lock(); // CRITICAL SECTION

            tracked_timestamp = frame_timestamp;

            frame.copyTo(im);

            mutex.unlock(); // END CRITICAL SECTION

            // TODO: Locate target in Image and put into positions_i
            eg::Vector2f position_i = {};
            positions_i.push_back(position_i);

            // TODO: Transform target to Camera and put into positions_c (fake homogeneous coordinates)
            eg::Vector3f position_c = invert_camera_transform(position_i);
            positions_c.push_back(position_c);

            // TODO: Transform target to World and put into positions_w (fake homogeneous coordinates)
            eg::Vector3f position_w = invert_world_transform(position_c);
            positions_w.push_back(position_w);
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
        mutex.lock(); // CRITICAL SECTION

        auto target_last_known_position = positions_w[positions_w.size() - 1];

        mutex.unlock(); // END CRITICAL SECTION
    }

    log("End chasing routine");
}

void Chase2D::stop()
{
    is_recording = false;
    is_tracking  = false;
    is_chasing   = false;

    recording_thread.join();
    tracking_thread.join();
    chasing_thread.join();
}

eg::Vector3f Chase2D::invert_camera_transform(const eg::Vector2f& position_i)
{
    return eg::Vector3f();
}

eg::Vector3f Chase2D::invert_world_transform(const eg::Vector3f& position_c)
{
    return eg::Vector3f();
}

eg::Matrix4f Chase2D::build_extrinsics(mav::Telemetry::PositionNED translation, mav::Telemetry::EulerAngle rotation)
{
    // TODO
    eg::Matrix4f extrinsics = eg::Matrix4f::Zero();

    auto drone_rotation  = euler_angle_to_rotation_matrix(rotation);
    auto camera_mounting = euler_angle_to_rotation_matrix({});

    extrinsics.block<3, 3>(0, 0) = drone_rotation * camera_mounting;

    extrinsics.block<3, 1>(0, 3) << translation.north_m, translation.east_m, translation.down_m;

    return extrinsics;
}

eg::Matrix3f Chase2D::euler_angle_to_rotation_matrix(mav::Telemetry::EulerAngle ea)
{
    auto a = (ea.roll_deg / 180) * M_PI;
    auto b = (ea.pitch_deg / 180) * M_PI;
    auto c = (ea.yaw_deg / 180) * M_PI;

    eg::Matrix3f mx;
    mx << 1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a);

    eg::Matrix3f my;
    my << cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b);

    eg::Matrix3f mz;
    mz << cos(c), -sin(c), 0, sin(c), cos(c), 0, 0, 0, 1;

    return mz * my * mx;
}
