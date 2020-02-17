#include "common.h"

// Handle Action results
void check_action_result(Action::Result result, const string& fail_message)
{
    if (result != Action::Result::SUCCESS) {
        cerr << CLI_COLOR_RED << fail_message << Action::result_str(result) << CLI_COLOR_NORMAL << endl;
        exit(-1);
    }
}

// Handle Offboard results
void check_offboard_result(Offboard::Result result, const string& fail_message)
{
    if (result != Offboard::Result::SUCCESS) {
        cerr << CLI_COLOR_RED << fail_message << Offboard::result_str(result) << CLI_COLOR_NORMAL << endl;
        exit(-1);
    }
}

// Handle connection results
void check_connection_result(ConnectionResult result, const string& fail_message)
{
    if (result != ConnectionResult::SUCCESS) {
        cerr << CLI_COLOR_RED << fail_message << connection_result_str(result) << CLI_COLOR_NORMAL << endl;
        exit(-1);
    }
}

void exit_and_land(const Action& action, const Telemetry& telemetry)
{
    const Action::Result land_result = action.land();
    check_action_result(land_result, "Landing failed: ");

    while (telemetry.in_air()) {
        cout << "Exit and land..." << endl;
        sleep_for(seconds(1));
    }

    cout << "Landed!" << endl;
}

Matx33f euler_angle_to_rotation_matrix(const Telemetry::EulerAngle& ea)
{
    float a = ea.roll_deg / 180 * (float) CV_PI;
    float b = ea.pitch_deg / 180 * (float) CV_PI;
    float c = ea.yaw_deg / 180 * (float) CV_PI;

    Matx33f mx(1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a));

    Matx33f my(cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b));

    Matx33f mz(cos(c), -sin(c), 0, sin(c), cos(c), 0, 0, 0, 1);

    return mz * my * mx;
}

pair<optional<float>, optional<float>> solve_quadratic(float a, float b, float c)
{
    float delta = b * b - 4 * a * c;

    if (delta > 0) {
        return {(-b + sqrt(delta)) / a / 2, (-b - sqrt(delta)) / a / 2};
    } else if (delta == 0) {
        return {-b / a / 2, {}};
    }

    return {{}, {}};
}
