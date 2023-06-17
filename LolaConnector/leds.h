#pragma once

#include <array>

struct RGB {
    float r, g, b;
    static const RGB RED;
    static const RGB GREEN;
    static const RGB BLUE;
    static const RGB ORANGE;
    static const RGB YELLOW;
};

using Eye = std::array<RGB, 8>;

struct Eyes {
    Eye left;
    Eye right;
};

struct Ears {
    std::array<float, 10> left;
    std::array<float, 10> right;
};

struct Leds {
    Ears ears;
    Eyes eyes;
};
