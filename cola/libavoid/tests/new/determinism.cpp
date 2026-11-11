// Routing must not depend on where the allocator places libavoid's objects.
//
// Some ordered containers broke ties by comparing POINTERS, so the same scene,
// routed twice in one process, could be drawn differently depending on heap
// layout. Each case below is a scene reduced from a real diagram where that
// happened. It is routed twice, once with objects at ascending addresses and
// once at descending addresses, and both drawings must be identical.

#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>

#include "gtest/gtest.h"
#include "libavoid/libavoid.h"

using namespace Avoid;

namespace {

// A TEST-ONLY ALLOCATOR THAT DECIDES ADDRESS ORDER. While a scene is routing,
// every allocation in this executable comes from one arena, handed out either
// in ASCENDING or in DESCENDING addresses. Creation order is the same both
// times; address order is reversed. Any comparison that breaks a tie by
// pointer value therefore flips between the two passes and the drawing
// changes -- deterministically, on any platform, whatever the real allocator
// would have done.
namespace arena {
    const std::size_t kSize = std::size_t(64) << 20;
    char *base = nullptr;
    std::size_t up = 0, down = kSize;
    bool active = false, descending = false;

    void begin(bool desc) {
        if (!base) base = static_cast<char *>(std::malloc(kSize));
        up = 0; down = kSize; descending = desc; active = true;
    }
    void end() { active = false; }
    bool owns(void *p) { return base && p >= base && p < base + kSize; }
    void *take(std::size_t n) {
        n = (n + 15) & ~std::size_t(15);
        if (up + n > down) throw std::bad_alloc();
        if (descending) { down -= n; return base + down; }
        void *p = base + up; up += n; return p;
    }
}

}  // namespace

void *operator new(std::size_t n) {
    if (arena::active) return arena::take(n);
    void *p = std::malloc(n ? n : 1);
    if (!p) throw std::bad_alloc();
    return p;
}
void operator delete(void *p) noexcept { if (!arena::owns(p)) std::free(p); }
void operator delete(void *p, std::size_t) noexcept { if (!arena::owns(p)) std::free(p); }

namespace {

ShapeRef *box(Router *router, unsigned id, double x1, double y1, double x2, double y2)
{
    Rectangle rect(Point(x1, y1), Point(x2, y2));
    return new ShapeRef(router, rect, id);
}

void exclusivePin(ShapeRef *shape, unsigned classId, double xProp, double yProp, ConnDirFlags dir)
{
    ShapeConnectionPin *pin = new ShapeConnectionPin(shape, classId, xProp, yProp, true, 0.0, dir);
    pin->setExclusive(true);
}

std::string drawing(Router *router)
{
    std::string out;
    char buf[64];
    for (ConnRef *conn : router->connRefs) {
        out += std::to_string(conn->id()) + ':';
        for (const Point &p : conn->displayRoute().ps) {
            std::snprintf(buf, sizeof buf, " %g,%g", p.x, p.y);
            out += buf;
        }
        out += '\n';
    }
    return out;
}

// Routes `scene` with objects at ascending and then descending addresses and
// returns both drawings.
template <typename Scene>
std::pair<std::string, std::string> routeBothWays(Scene scene)
{
    std::string drawn[2];
    for (int pass = 0; pass < 2; ++pass) {
        arena::begin(pass == 1);
        Router *router = new Router(OrthogonalRouting);
        scene(router);
        router->processTransaction();
        drawn[pass] = drawing(router);
        arena::end();
        // The router lives in the arena, which is simply reset for the next
        // pass; it is not destroyed.
    }
    return { drawn[0], drawn[1] };
}

}  // namespace

// Reduced from a 30-box diagram. Shape 6 has several exclusive pins of the
// same class, two of them at exactly the same point. The orthogonal visibility
// sweep (orthogonal.cpp, CmpVertInf) meets those pins' vertices at equal
// positions and used to order them by address, so the line left from a
// different pin depending on heap layout.
TEST(Determinism, CoincidentExclusivePinsOnOneSide) {
    auto drawn = routeBothWays([](Router *router) {
        ShapeRef *s6 = box(router, 6, 434, 180, 566, 261);
        exclusivePin(s6, 1, 0, 0.999, ConnDirLeft);
        exclusivePin(s6, 1, 0, 0.873016, ConnDirLeft);
        exclusivePin(s6, 1, 0, 0.999, ConnDirLeft);
        ShapeRef *s9 = box(router, 9, -200, 0, 0, 600);
        exclusivePin(s9, 110, 1, 0.775, ConnDirRight);
        new ConnRef(router, ConnEnd(s6, 1), ConnEnd(s9, 110), 55);
    });
    EXPECT_EQ(drawn.first, drawn.second);
}

// Reduced from a 50-line diagram. Two lines run in opposite directions between
// the same pair of boxes and, with a shared-path penalty, cross. Crossing
// removal (router.cpp, CrossingConnectorsMap) chooses which to reroute first
// and used to order the candidates by address, so either line could be the one
// that moved.
TEST(Determinism, OpposedLinesBetweenOnePairOfBoxes) {
    auto drawn = routeBothWays([](Router *router) {
        router->setRoutingParameter(fixedSharedPathPenalty, 106.02);
        router->setRoutingParameter(shapeBufferDistance, 22);
        ShapeRef *s1 = box(router, 1, 635, 330, 776, 411);
        exclusivePin(s1, 101, 0, 0.555556, ConnDirLeft);
        exclusivePin(s1, 1, 0, 0.396825, ConnDirLeft);
        ShapeRef *s4 = box(router, 4, 434, 330, 575, 411);
        exclusivePin(s4, 111, 1, 0.555556, ConnDirRight);
        exclusivePin(s4, 1, 1, 0.238095, ConnDirRight);
        exclusivePin(s4, 1, 1, 0.396825, ConnDirRight);
        new ConnRef(router, ConnEnd(s4, 1), ConnEnd(s1, 101), 38);
        new ConnRef(router, ConnEnd(s1, 1), ConnEnd(s4, 111), 61);
    });
    EXPECT_EQ(drawn.first, drawn.second);
}
