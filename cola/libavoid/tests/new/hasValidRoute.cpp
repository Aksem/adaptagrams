#include "libavoid/libavoid.h"
#include "gtest/gtest.h"
#include "helpers.h"
/*
 * Test ConnRef::hasValidRoute() and ConnRef::hasCrossingObstacles().
 *
 * hasValidRoute() — false when the router fell back to a direct straight
 *                    line (A* found no path).
 * hasCrossingObstacles() — true when the route crosses any obstacle the
 *                           connector is not attached to.
 */

using namespace Avoid;

class HasValidRoute : public ::testing::Test {
protected:
    void SetUp() override {
        router = new Router(OrthogonalRouting);
        router->setRoutingParameter(RoutingParameter::shapeBufferDistance, 16);
        router->setRoutingParameter(RoutingParameter::segmentPenalty, 50);
        router->setRoutingParameter(RoutingParameter::idealNudgingDistance, 16);
    }

    void TearDown() override {
        delete router;
    }

    ShapeRef* addShape(Point topLeft, Point bottomRight, unsigned int shapeId) {
        Rectangle shapeRectangle(topLeft, bottomRight);
        ShapeRef *shape = new ShapeRef(router, shapeRectangle, shapeId);
        auto pin = new ShapeConnectionPin(shape, 100,
                                          ATTACH_POS_CENTRE, ATTACH_POS_CENTRE, true, 0.0, ConnDirNone);
        pin->setExclusive(false);
        return shape;
    }

    ConnRef* connectShapes(ShapeRef *shape1, ShapeRef *shape2) {
        ConnEnd srcEnd(shape1, 100);
        ConnEnd dstEnd(shape2, 100);
        return new ConnRef(router, srcEnd, dstEnd);
    }

    Router *router;
};

// --- hasValidRoute tests ---

TEST_F(HasValidRoute, FalseBeforeProcessTransaction) {
    ShapeRef *left = addShape({ 100, 100 }, { 200, 200 }, 1);
    ShapeRef *right = addShape({ 400, 100 }, { 500, 200 }, 2);
    ConnRef *conn = connectShapes(left, right);

    EXPECT_FALSE(conn->hasValidRoute());
}

TEST_F(HasValidRoute, TrueForUnobstructedPath) {
    ShapeRef *left = addShape({ 100, 100 }, { 200, 200 }, 1);
    ShapeRef *right = addShape({ 400, 100 }, { 500, 200 }, 2);
    ConnRef *conn = connectShapes(left, right);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_TrueForUnobstructedPath");

    EXPECT_TRUE(conn->hasValidRoute());
}

TEST_F(HasValidRoute, TrueWhenObstacleAvoidable) {
    ShapeRef *left = addShape({ 100, 100 }, { 200, 200 }, 1);
    ShapeRef *right = addShape({ 500, 100 }, { 600, 200 }, 2);
    // Obstacle in the middle, but leaves room above/below.
    addShape({ 300, 120 }, { 400, 180 }, 3);

    ConnRef *conn = connectShapes(left, right);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_TrueWhenObstacleAvoidable");

    EXPECT_TRUE(conn->hasValidRoute());
}

TEST_F(HasValidRoute, FalseWhenPinDirectionsPreventRouting) {
    // Pins point away from each other, and blocking shapes prevent
    // the router from going around, so no valid path exists.
    ShapeRef *left = addShape({ 100, 100 }, { 200, 200 }, 1);
    ShapeRef *right = addShape({ 400, 100 }, { 500, 200 }, 2);

    // Left pin only allows ConnDirLeft (away from right shape).
    auto leftPin = new ShapeConnectionPin(left, 200,
            ATTACH_POS_CENTRE, ATTACH_POS_CENTRE, true, 0.0, ConnDirLeft);
    leftPin->setExclusive(false);
    // Right pin only allows ConnDirRight (away from left shape).
    auto rightPin = new ShapeConnectionPin(right, 200,
            ATTACH_POS_CENTRE, ATTACH_POS_CENTRE, true, 0.0, ConnDirRight);
    rightPin->setExclusive(false);

    // Block the left pin's outward direction with a wall to its left.
    addShape({ 0, 50 }, { 84, 250 }, 10);
    // Block the right pin's outward direction with a wall to its right.
    addShape({ 516, 50 }, { 600, 250 }, 11);
    // Block above and below to close off all escape routes.
    addShape({ 0, 0 }, { 600, 84 }, 12);
    addShape({ 0, 216 }, { 600, 300 }, 13);

    ConnEnd srcEnd(left, 200);
    ConnEnd dstEnd(right, 200);
    ConnRef *conn = new ConnRef(router, srcEnd, dstEnd);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_FalseWhenPinDirectionsPreventRouting");

    EXPECT_FALSE(conn->hasValidRoute());
}

// --- hasCrossingObstacles tests ---

TEST_F(HasValidRoute, NoCrossingObstaclesForUnobstructedPath) {
    ShapeRef *left = addShape({ 100, 100 }, { 200, 200 }, 1);
    ShapeRef *right = addShape({ 400, 100 }, { 500, 200 }, 2);
    ConnRef *conn = connectShapes(left, right);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_NoCrossingObstaclesForUnobstructedPath");

    EXPECT_FALSE(conn->hasCrossingObstacles());
}

TEST_F(HasValidRoute, NoCrossingObstaclesWhenAvoidable) {
    ShapeRef *left = addShape({ 100, 100 }, { 200, 200 }, 1);
    ShapeRef *right = addShape({ 500, 100 }, { 600, 200 }, 2);
    addShape({ 300, 120 }, { 400, 180 }, 3);

    ConnRef *conn = connectShapes(left, right);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_NoCrossingObstaclesWhenAvoidable");

    EXPECT_FALSE(conn->hasCrossingObstacles());
}

TEST_F(HasValidRoute, CrossingObstaclesWhenShapeEnclosed) {
    // A small shape fully inside a large obstacle — the route must
    // cross the enclosing obstacle to reach the inner shape's pin.
    ShapeRef *inner = addShape({ 20, 20 }, { 30, 30 }, 1);
    addShape({ 0, 0 }, { 50, 50 }, 2);
    ShapeRef *outside = addShape({ 100, 100 }, { 200, 200 }, 3);

    ConnRef *conn = connectShapes(inner, outside);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_CrossingObstaclesWhenShapeEnclosed");

    EXPECT_TRUE(conn->hasCrossingObstacles());
}

TEST_F(HasValidRoute, CrossingObstaclesWhenDestinationEnclosed) {
    ShapeRef *outside = addShape({ 100, 100 }, { 200, 200 }, 1);
    addShape({ 300, 0 }, { 500, 300 }, 2);
    ShapeRef *inner = addShape({ 380, 120 }, { 420, 180 }, 3);

    ConnRef *conn = connectShapes(outside, inner);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_CrossingObstaclesWhenDestinationEnclosed");

    EXPECT_TRUE(conn->hasCrossingObstacles());
}

TEST_F(HasValidRoute, NoCrossingObstaclesAfterEnclosingRemoved) {
    ShapeRef *inner = addShape({ 20, 20 }, { 30, 30 }, 1);
    ShapeRef *enclosing = addShape({ 0, 0 }, { 50, 50 }, 2);
    ShapeRef *outside = addShape({ 100, 100 }, { 200, 200 }, 3);

    ConnRef *conn = connectShapes(inner, outside);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_NoCrossingObstaclesAfterEnclosingRemoved-1");
    EXPECT_TRUE(conn->hasCrossingObstacles());

    router->deleteShape(enclosing);
    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_NoCrossingObstaclesAfterEnclosingRemoved-2");

    EXPECT_FALSE(conn->hasCrossingObstacles());
}

TEST_F(HasValidRoute, MultipleConnectorsCrossingIndependent) {
    // Enclosed shape — route to it must cross the enclosing obstacle.
    ShapeRef *inner = addShape({ 20, 20 }, { 30, 30 }, 1);
    addShape({ 0, 0 }, { 50, 50 }, 2);

    // Free shapes — route between them is obstacle-free.
    ShapeRef *freeLeft = addShape({ 100, 100 }, { 200, 200 }, 3);
    ShapeRef *freeRight = addShape({ 400, 100 }, { 500, 200 }, 4);

    ConnRef *crossingConn = connectShapes(inner, freeRight);
    ConnRef *freeConn = connectShapes(freeLeft, freeRight);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HasValidRoute_MultipleConnectorsCrossingIndependent");

    EXPECT_TRUE(crossingConn->hasCrossingObstacles());
    EXPECT_FALSE(freeConn->hasCrossingObstacles());
}
