#include "libavoid/libavoid.h"
#include "gmock/gmock.h"

namespace Avoid {
    void PrintTo(ConnRef *conn, std::ostream *os) {
        std::vector<Point> route = conn->displayRoute().ps;
        *os << "[";
        for (int i = 0; i < route.size(); i++) {
            *os << "(" << route.at(i).x << ", " << route.at(i).y << ")";
            if (i != route.size() - 1) {
                *os << ", ";
            }
        }
        *os << "]";
    }
}
