#include "Chase2D.h"

int main()
{
    mav::Telemetry::PositionNED position_ned{1, 2, 3};

    eg::Vector3f vector_3_f(position_ned.north_m, position_ned.east_m, position_ned.down_m);

    std::cout << vector_3_f << std::endl;

    return 0;
}
