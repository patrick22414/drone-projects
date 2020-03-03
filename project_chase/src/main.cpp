#include "Chase2D.h"

eg::Vector2f test()
{
    eg::Vector3f vec(1, 2, 3);
    return vec.head(2);
}

int main()
{
    auto x = test();
    std::cout << x << std::endl;

    x[0] = 0;

    std::cout << x << std::endl;

    return 0;
}
