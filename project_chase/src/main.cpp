#include "Chase2D.h"

int main()
{
    // TODO
    Chase2D chase("udp://:14540", "test-v1.mp4");

    chase.start(1, 1);

    chase.stop();

    return 0;
}
