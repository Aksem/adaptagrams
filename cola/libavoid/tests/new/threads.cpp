// Independent Routers on different threads must not affect each other.
//
// getNewUniqueId() hands out the ids that order a shape's connection pins (and
// vertices, edges and obstacles). When it was one process-wide counter,
// concurrent routes raced on its non-atomic increment, so two objects could get
// the same id and a shape's pin set silently dropped one of them ("can't
// connect to shape ... no pins with class id"), changing the drawing.
//
// The race needs real parallelism, so the test is skipped on a single core and
// may not catch a regression on a machine with few cores.

#include <atomic>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "libavoid/libavoid.h"

using namespace Avoid;

namespace {

// One shape with forty pins along its top edge, each the end of a connector.
std::string routeScene()
{
    Router *router = new Router(OrthogonalRouting);
    Rectangle rect(Point(0, 100), Point(400, 200));
    ShapeRef *shape = new ShapeRef(router, rect);
    for (unsigned i = 1; i <= 40; ++i) {
        new ShapeConnectionPin(shape, i, i / 41.0, 0, true, 0, ConnDirUp);
    }
    for (unsigned i = 1; i <= 40; ++i) {
        new ConnRef(router, ConnEnd(Point(i * 10.0, 0), ConnDirAll), ConnEnd(shape, i));
    }
    router->processTransaction();

    std::string out;
    char buf[64];
    for (ConnRef *conn : router->connRefs) {
        for (const Point &p : conn->displayRoute().ps) {
            std::snprintf(buf, sizeof buf, " %g,%g", p.x, p.y);
            out += buf;
        }
        out += '\n';
    }
    delete router;
    return out;
}

}  // namespace

TEST(Threads, ConcurrentRoutersDrawWhatOneThreadDraws) {
    const unsigned cores = std::thread::hardware_concurrency();
    if (cores < 2) {
        GTEST_SKIP() << "needs at least two cores to race";
    }
    const std::string expected = routeScene();

    const unsigned threadCount = cores < 16 ? cores : 16;
    const unsigned routesPerThread = 500;
    std::atomic<unsigned> differing{0};
    std::vector<std::thread> threads;
    for (unsigned t = 0; t < threadCount; ++t) {
        threads.emplace_back([&] {
            for (unsigned i = 0; i < routesPerThread; ++i) {
                if (routeScene() != expected) {
                    ++differing;
                }
            }
        });
    }
    for (std::thread &thread : threads) {
        thread.join();
    }
    EXPECT_EQ(differing.load(), 0u)
            << "of " << threadCount * routesPerThread << " routes on "
            << threadCount << " threads";
}
