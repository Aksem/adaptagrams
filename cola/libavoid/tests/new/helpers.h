#ifndef ADAPTAGRAMS_HELPERS_H
#define ADAPTAGRAMS_HELPERS_H

#include <cmath>
#include <gmock/gmock.h>
#include "libavoid/libavoid.h"


bool compare_float(double x, double y, double epsilon = 0.001f) {
    if (fabs(x - y) < epsilon)
        return true;
    return false;
}

MATCHER_P(IsEqualToRoute, value, "equal routes") {
    std::vector<Avoid::Point> actualRoute = arg->displayRoute().ps;
    if (actualRoute.size() != value.size()) {
        *result_listener << "Length of routes is not equal " << actualRoute.size() << " != " << value.size() << " (expected)";
        return false;
    }

    bool result = true;
    *result_listener << "\n";
    for (int i = 0; i < actualRoute.size(); i++) {
        if (!(compare_float(actualRoute.at(i).x, value.at(i).x) &&
              compare_float(actualRoute.at(i).y, value.at(i).y))) {
            *result_listener << "Point " << (i + 1) << ": actual (" << actualRoute.at(i).x << ", "
                             << actualRoute.at(i).y << "), expected (" << value.at(i).x << ", " << value.at(i).y
                             << ")\n";
            result = false;
        }
    }
    return result;
}

#endif //ADAPTAGRAMS_HELPERS_H
