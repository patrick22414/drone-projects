#include "common.h"
#include "transforms.h"

// Given a point in the Image frame, transform the position into the Camera frame
Vec3d invert_camera_transform(double ball_x_px, double ball_y_px, double ball_radius_px, const Vec2i& resolution)
{
    const double ball_radius = 1e-2 * 2.5 * 2.54; // tiny basketball radius 2.5 inches

    // Pi Camera V1 parameters
    const double sensor_w_px     = 2592;   // sensor width in pixels
    const double sensor_h_px     = 1944;   // sensor height in pixels
    const double focal_length    = 3.6e-3; // focal length 3.6mm
    const double real_pixel_size = 1.4e-6; // real pixel size 1.4um

    double res_w_px = resolution[0];
    double res_h_px = resolution[1];

    double pix_size = real_pixel_size * (sensor_w_px / res_w_px); // relative pixel size with binning

    double ball_i_x = ball_x_px * pix_size; // ball position x, Image frame
    double ball_i_y = ball_y_px * pix_size; // ball position y, Image frame

    // camera intrinsic matrix
    double intrinsics[3][3] = {
        {focal_length, 0, 0.5 * (res_w_px - 1) * pix_size},
        {0, focal_length, 0.5 * (res_h_px - 1) * pix_size},
        {0, 0, 1},
    };

    // object homogeneous coordinates, Image frame (z = 1)
    double object_i_homo[3] = {ball_i_x, ball_i_y, 1};

    Mat A(3, 3, CV_64F, intrinsics);
    Mat b(3, 1, CV_64F, object_i_homo);
    Mat x(3, 1, CV_64F);
    solve(A, b, x, DECOMP_SVD);

    // ray direction, expressed by (0, 0, 0) -> (xc1, yc1, 1)
    double xc1 = x.at<double>(0);
    double yc1 = x.at<double>(1);

    // distance from camera to ball
    double distance = ball_radius * sqrt(focal_length * focal_length + ball_i_x * ball_i_x + ball_i_y * ball_i_y) /
                      (ball_radius_px * pix_size);

    // distance from camera to (xc1, yc1, 1)
    double distance1 = sqrt(xc1 * xc1 + yc1 * yc1 + 1);

    // object coordinates, Camera frame
    double xc = xc1 * distance / distance1;
    double yc = yc1 * distance / distance1;
    double zc = sqrt(distance * distance - xc * xc - yc * yc);

    return {xc, yc, zc};
}

// Given a point in the Camera frame, transform the position into the World frame
Vec3d invert_world_transform(const Vec3d& p_c, const Telemetry::PositionNED& t, const Telemetry::EulerAngle& r)
{

    Matx33d drone_attitude_rotation  = euler_angle_to_rotation_matrix(r);
    Matx33d camera_mounting_rotation = euler_angle_to_rotation_matrix({0, 180, 90 + 20});

    // Total extrinsic rotation
    Matx33d A = drone_attitude_rotation * camera_mounting_rotation;

    // Camera frame coordinates minus transformation T
    Matx31d b(p_c[0] - t.north_m, p_c[1] - t.east_m, p_c[2] - t.down_m);

    Matx31d x = A.solve(b, DECOMP_SVD);

    return Vec3d(x.val);
}

Vec6d calculate_parabola(const vector<Vec3d>& points, const vector<nanoseconds>& timestamps)
{
    // Find the parameters of a parabola from a series of points in World frame
    assert(points.size() == timestamps.size());

    int n = points.size();

    vector<double> A_data(n * 3 * 6, 0.0);
    vector<double> b_data(n * 3);
    for (int i = 0; i < n; ++i) {
        double s = 1e-9 * timestamps[i].count(); // time in second

        int A_start         = i * 3 * 6;
        A_data[A_start + 0] = A_data[A_start + 7] = A_data[A_start + 14] = s;
        A_data[A_start + 3] = A_data[A_start + 10] = A_data[A_start + 17] = 1;

        int b_start         = i * 3;
        b_data[b_start + 0] = points[i][0];
        b_data[b_start + 1] = points[i][1];
        b_data[b_start + 2] = points[i][2] - G_HALF * s * s;
    }

    Mat A = Mat(A_data).reshape(1, n * 3);
    Mat b = Mat(b_data);
    Mat x = Mat(6, 1, CV_64F);

    solve(A, b, x, DECOMP_SVD);

    Vec6d parabola((double*)x.data);

    return parabola;
}

Offboard::PositionNEDYaw calculate_destination(const Vec6d& parabola, double catch_alt)
{
    // catch altitude should be negated in NED frame
    catch_alt = -catch_alt;

    double vx = parabola[0]; // x speed, assuming unchanged
    double vy = parabola[1]; // y speed, assuming unchanged
    double vz = parabola[2]; // initial z speed
    double x0 = parabola[3]; // initial x position
    double y0 = parabola[4]; // initial y position
    double z0 = parabola[5]; // initial z position

    // Solve a quadratic equation
    double a = G_HALF;
    double b = vz;
    double c = z0 - catch_alt;

    double delta = b * b - 4 * a * c;
    if (delta <= 0) {
        throw runtime_error("How could the delta be <= 0???");
    }

    double root1 = (-b + sqrt(delta)) / (2 * a);
    double root2 = (-b - sqrt(delta)) / (2 * a);

    double t = root1 > root2 ? root1 : root2;

    double catch_x = vx * t + x0;
    double catch_y = vy * t + y0;

    return {(float)catch_x, (float)catch_y, (float)catch_alt, 0};
}
