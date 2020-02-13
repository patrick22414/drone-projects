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

Matx33d euler_angle_to_rotation_matrix(const Telemetry::EulerAngle& ea)
{
    double a = ea.roll_deg / 180 * CV_PI;
    double b = ea.pitch_deg / 180 * CV_PI;
    double c = ea.yaw_deg / 180 * CV_PI;

    Matx33d mx(1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a));

    Matx33d my(cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b));

    Matx33d mz(cos(c), -sin(c), 0, sin(c), cos(c), 0, 0, 0, 1);

    return mz * my * mx;
}
