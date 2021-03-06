import numpy as np
from matplotlib import pyplot as plt

if __name__ == "__main__":
    fig, ax = plt.subplots()

    uvw1 = np.array(
        [
            [1.0804645, -4.9401126, 0.010080513, 1],
            [0.79167128, -4.8477669, 0.0018966272, 1],
            [0.56873643, -5.0289402, -0.010560282, 1],
            [0.26764062, -4.8107386, -0.0078939945, 1],
            [0.0012339205, -4.6640644, -0.0012544841, 1],
            [-0.25259039, -4.5534649, -0.002084516, 1],
            [-0.45031476, -4.2029209, 0.0098161325, 1],
        ]
    )

    us = uvw1[:, 0]
    vs = uvw1[:, 1]

    ax.plot(us, -vs)

    # parabola
    p0, p1, p2 = -4.6839, -0.732423, 0.492358
    u1 = np.linspace(min(us), max(us))
    v1 = p0 + p1 * u1 + p2 * u1 * u1

    ax.plot(u1, -v1)

    ax.axis("equal")
    plt.show()
