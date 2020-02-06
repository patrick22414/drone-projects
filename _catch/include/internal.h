//
// Created by yang on 2/6/20.
//

#ifndef _CATCH_INTERNAL_H
#define _CATCH_INTERNAL_H

#define G 9.81

#define CLI_COLOR_RED "\033[31m" // Turn text on console red
#define CLI_COLOR_GREEN "\033[32m" // Turn text on console red
#define CLI_COLOR_YELLOW "\033[33m" // Turn text on console red
#define CLI_COLOR_BLUE "\033[34m" // Turn text on console blue
#define CLI_COLOR_NORMAL "\033[0m" // Restore normal console colour

#include <opencv2/opencv.hpp>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>

using namespace cv;
using namespace std;
using namespace mavsdk;

// Sort contours by descending area
inline void sortContours(vector<vector<Point2i>>& contours)
{
    sort(
        contours.begin(), contours.end(), [](const vector<Point2i>& c1, const vector<Point2i>& c2) {
            return contourArea(c1, false) > contourArea(c2, false);
        });
}

// Handle Action results
inline void check_action_result(Action::Result result, const string& fail_message)
{
    if (result != Action::Result::SUCCESS) {
        cerr << CLI_COLOR_RED << fail_message << Action::result_str(result) << CLI_COLOR_NORMAL
             << endl;
        exit(-1);
    }
}

// Handle Offboard results
inline void check_offboard_result(Offboard::Result result, const string& fail_message)
{
    if (result != Offboard::Result::SUCCESS) {
        cerr << CLI_COLOR_RED << fail_message << Offboard::result_str(result) << CLI_COLOR_NORMAL
             << endl;
        exit(-1);
    }
}

// Handle connection results
inline void check_connection_result(ConnectionResult result, const string& fail_message)
{
    if (result != ConnectionResult::SUCCESS) {
        cerr << CLI_COLOR_RED << fail_message << connection_result_str(result) << CLI_COLOR_NORMAL
             << endl;
        exit(-1);
    }
}

#endif //_CATCH_INTERNAL_H
