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

void Chase2D::start(const std::vector<float>& speeds)
{
    while (true) {
        if (!capture.read(frame))
            log_red_and_exit("Cannot read frame or end of video");

        cv::flip(frame, frame, -1);
        cv::imshow("Press Space to select ROI", frame);

        if (cv::waitKey(1) == ' ') {
            cv::destroyAllWindows();
            tracker_roi = cv::selectROI("Press Space to confirm ROI", frame);
            break;
        }
    }

    cv::destroyAllWindows();

    is_recording     = true;
    recording_thread = std::thread(&Chase2D::recording_routine, this);

    is_tracking     = true;
    tracking_thread = std::thread(&Chase2D::tracking_routine, this);

    is_chasing     = true;
    chasing_thread = std::thread(&Chase2D::chasing_routine, this, speeds[0], speeds[1], speeds[2]);
}

void Chase2D::recording_routine()
{
    // Recording routine records the raw camera frame to `frame`, and optionally displays a live feed of the raw frame.
    log("Start recording routine");

    bool has_writer = writer.isOpened();

    while (is_recording) {
        mutex.lock(); // CRITICAL SECTION

        frame_timestamp = Clock::now();
#ifdef WITH_DRONE
        frame_position = current_position();
        frame_attitude = current_attitude();
#endif

        if (!capture.read(frame))
            log_red_and_exit("Cannot read frame or end of video");

        cv::flip(frame, frame, -1);

        mutex.unlock(); // END CRITICAL SECTION

        if (has_writer)
            writer.write(frame);
    }

    log("End recording routine");
}

void Chase2D::tracking_routine()
{
    // Tracking routine processes the raw image from `frame`, finds the target in each coordinate system, in turn
    // populates `positions_i`, `positions_c`, and `positions_w`, and optionally displays a live feed of the processed
    // image
    log("Start tracking routine");

    tracker->init(frame, tracker_roi);

    cv::Mat im;
    while (is_tracking) {
        if (frame_timestamp > tracked_timestamp) {
            mutex.lock(); // CRITICAL SECTION

            tracked_timestamp = frame_timestamp;
            frame.copyTo(im);

#ifdef WITH_DRONE
            extrinsics = build_extrinsics(frame_position, frame_attitude);
#endif

            mutex.unlock(); // END CRITICAL SECTION

            tracker->update(im, tracker_roi);
            std::cout << "Tracker ROI: " << tracker_roi << std::endl;

            // Locate target in Image and put into positions_i
            eg::Vector2f position_i(tracker_roi.x + tracker_roi.width / 2, tracker_roi.y + tracker_roi.height / 2);


            // Transform target to Camera and put into positions_c (fake homogeneous coordinates)
            // auto position_c = invert_camera_transform(position_i);

            // Transform target to World and put into positions_w (fake homogeneous coordinates)
            // auto position_w = invert_world_transform(position_c);

            mutex.lock(); // CRITICAL SECTION

            positions_i.push_back(position_i);
            // positions_c.push_back(position_c);
            // positions_w.push_back(position_w);

            mutex.unlock(); // END CRITICAL SECTION

            cv::rectangle(im, tracker_roi, {255, 0, 0});
            cv::drawMarker(im, {static_cast<int>(position_i[0]), static_cast<int>(position_i[1])}, {0, 255, 255});
        }

        cv::imshow("Tracking", im);
        cv::waitKey(1);
    }

    cv::destroyAllWindows();

    log("End tracking routine");
}

void Chase2D::chasing_routine(float f_speed, float v_speed, float y_speed)
{
    // Chasing routine takes the last known position of the target (`positions_w[-1]`) and current telemetry status of
    // the drone, determines the next velocity and gives it to the drone.
    log("Start chasing routine");

#ifdef WITH_DRONE
    // Must set one offboard command before start offboard
    offboard->set_velocity_body({0, 0, 0, 0});
    auto start_result = offboard->start();
    check_offboard_result(start_result);
#endif

    while (is_chasing) {
        sleep_for(milliseconds(1000));

        if (positions_i.empty())
            continue;

        mutex.lock(); // CRITICAL SECTION

        auto target_last_known_position = positions_i[positions_i.size() - 1];

        mutex.unlock(); // END CRITICAL SECTION

        float xc = target_last_known_position[0] - resolution[0] / 2.0f;
        float yc = target_last_known_position[1] - resolution[1] / 2.0f;

        float safe_area = resolution[1] / 10.0f;

        float chase_forward = f_speed;
        float chase_down    = abs(yc) < safe_area ? 0 : (yc > 0 ? v_speed : -v_speed);
        float chase_yaw     = abs(xc) < safe_area ? 0 : (xc > 0 ? y_speed : -y_speed);

        std::cout << "[Chase2D] Chasing at: FORWARD: " << v_speed << "m/s " << (chase_down > 0 ? "DOWN: " : "UP  : ")
                  << f_speed << "m/s " << (chase_yaw > 0 ? "RIGHT: " : "LEFT : ") << y_speed << "deg/s" << std::endl;

#ifdef WITH_DRONE
        offboard->set_velocity_body({chase_forward, 0, chase_down, chase_yaw});
#endif
    }

#ifdef WITH_DRONE
    auto stop_result = offboard->stop();
    check_offboard_result(stop_result);
#endif

    log("End chasing routine");
}

eg::Vector3f Chase2D::invert_camera_transform(const eg::Vector2f& position_i)
{
    eg::Vector3f position_i_homo(position_i[0], position_i[1], 1);
    eg::Vector3f position_c = intrinsics.jacobiSvd(eg::ComputeThinU | eg::ComputeThinV).solve(position_i_homo);
    return position_c;
}

eg::Vector3f Chase2D::invert_world_transform(const eg::Vector3f& position_c)
{
    eg::Vector4f position_c_homo(position_c[0], position_c[1], position_c[2], 1);
    eg::Vector4f position_w_homo = extrinsics.jacobiSvd(eg::ComputeThinU | eg::ComputeThinV).solve(position_c_homo);
    return position_w_homo.head(3);
}

eg::Matrix4f Chase2D::build_extrinsics(mav::Telemetry::PositionNED translation, mav::Telemetry::EulerAngle rotation)
{
    eg::Matrix4f extrinsics = eg::Matrix4f::Zero();

    static const auto camera_mounting_rotation = euler_angle_to_rotation_matrix({180, 90, 0});

    auto drone_attitude_rotation = euler_angle_to_rotation_matrix(rotation);

    extrinsics.block<3, 3>(0, 0) = drone_attitude_rotation * camera_mounting_rotation;
    extrinsics.block<3, 1>(0, 3) << translation.north_m, translation.east_m, translation.down_m;
    extrinsics(3, 3) = 1;

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

void Chase2D::stop()
{
    is_recording = false;
    is_tracking  = false;
    is_chasing   = false;

    recording_thread.join();
    tracking_thread.join();
    chasing_thread.join();
}