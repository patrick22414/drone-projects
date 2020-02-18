#include "common.h"
#include "transforms.h"

// Given a point in the Image frame, transform the position into the Camera frame
Vec3f invert_camera_transform(float ball_x_px, float ball_y_px, float ball_radius_px, const Vec2i& resolution)
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
    float intrinsics[9] = {
        focal_length,
        0,
        0.5f * (res_w_px - 1) * pix_size,
        0,
        focal_length,
        0.5f * (res_h_px - 1) * pix_size,
        0,
        0,
        1,
    };

    // object homogeneous coordinates, Image frame (z = 1)
    float object_i_homo[3] = {ball_i_x, ball_i_y, 1};

    Mat A(3, 3, CV_32F, intrinsics);
    Mat b(3, 1, CV_32F, object_i_homo);
    Mat x(3, 1, CV_32F);
    solve(A, b, x, DECOMP_SVD);

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

// Given a point in the Camera frame, transform the position into the World frame
Vec3f invert_world_transform(const Vec3f& p_c, const Telemetry::PositionNED& t, const Telemetry::EulerAngle& r)
{
    Matx33f drone_attitude_rotation  = euler_angle_to_rotation_matrix(r);
    Matx33f camera_mounting_rotation = euler_angle_to_rotation_matrix({0, 180, 90 + 20});

    // Total extrinsic rotation
    Matx33f A = drone_attitude_rotation * camera_mounting_rotation;

    // Camera frame coordinates minus transformation T
    Matx31f b(p_c[0] - t.north_m, p_c[1] - t.east_m, p_c[2] - t.down_m);

    Matx31f x = A.solve(b, DECOMP_SVD);

    return Vec3f(x.val);
}

Offboard::PositionNEDYaw calculate_destination_2(const vector<Vec3f>& points_w, float catch_alt)
{
    // Find a parabola
    // The parabola is defined as:
    // --- [u; v; w] = R * [x; y; z] + T
    // --- v = p0 + p1 * u + p2 * u^2
    // --- w = 0
    //
    // R and T transforms XYZ into UVW
    //     where UV is the plane y = ax + b, V axis is the Z axis and W is the normalized normal vector of the plane

    int N = points_w.size();

    Mat XYZ(3, N, CV_32F);
    for (int i = 0; i < N; ++i) {
        Mat(points_w[i], false).copyTo(XYZ.col(i));
    }

    // Fit to a plane y = ax + b
    Mat A1 = Mat::ones(N, 2, CV_32F);

    A1.col(0) = XYZ.row(0).t();

    Mat B1 = XYZ.row(1).t();

    Vec2f X1;
    solve(A1, B1, X1, DECOMP_QR);

    cout << "X1 = \n" << X1 << endl;

    // Now X1 = [a, b] for the plane y = ax + b
    float a = X1[0];
    float b = X1[1];

    // Choose an origin on this plane. Go with [0, b] for simplicity
    float x0 = 0;
    float y0 = b;

    // Rotation and Transformation matrices
    float tmp = sqrt(a * a + 1);
    Matx33f R(1 / tmp, 0, a / tmp, a / tmp, 0, -1 / tmp, 0, 1, 0);
    Vec3f T(x0, y0, 0);

    Mat UVW = R * XYZ;
    UVW.row(0) += T[0];
    UVW.row(1) += T[1];

    cout << "UVW = \n" << format(UVW.t(), Formatter::FMT_NUMPY) << endl;

    Mat row_u = UVW.row(0);
    Mat row_v = UVW.row(1);

    // Fit to a parabola on the u-v plane: v = p0 + p1 * u + p2 * u^2
    Mat A2(N, 3, CV_32F);
    A2.col(0) = 1;
    A2.col(1) = row_u.t();
    A2.col(2) = row_u.mul(row_u).t();

    Vec3f X2;
    solve(A2, row_v.t(), X2, DECOMP_QR);

    float p0 = X2[0];
    float p1 = X2[1];
    float p2 = X2[2];

    cout << CLI_COLOR_GREEN << "Got parabola parameters:" << CLI_COLOR_NORMAL << " [" << a << ", " << b << ", " << p0
         << ", " << p1 << ", " << p2 << "]" << endl;

    // This is the catch altitude in V coordinates
    float v = -catch_alt;

    // Solve v = p0 + p1 * u + p2 * u^2
    auto roots = solve_quadratic(p2, p1, p0 - v);

    if (!roots.first.has_value())
        throw runtime_error("Solve quadratic equation failed");
    else {
        float u1 = roots.first.value();

        // xyz = R^-1 * (uvw - T)
        Vec3f uvw1 = Vec3f(u1, v, 0) - T;
        Vec3f xyz1;
        solve(R, uvw1, xyz1);

        if (!roots.second.has_value())
            return {xyz1[0], xyz1[1], xyz1[2], 0};

        float u2 = roots.second.value();

        Vec3f uvw2 = Vec3f(u2, v, 0) - T;
        Vec3f xyz2;
        solve(R, uvw1, xyz2);

        // Choose a result along the flying direction
        Vec3f flying_direction = points_w[N - 1] - points_w[0];

        if (flying_direction.dot(xyz1) > flying_direction.dot(xyz2))
            return {xyz1[0], xyz1[1], xyz1[2], 0};
        else
            return {xyz2[0], xyz2[1], xyz2[2], 0};
    }
}
