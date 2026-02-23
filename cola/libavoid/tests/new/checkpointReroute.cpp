#include "libavoid/libavoid.h"
#include "gtest/gtest.h"
#include "helpers.h"
/*
 * Test that calling setRoutingCheckpoints() marks the connector as needing
 * rerouting so that the next processTransaction() recalculates its route.
 */

using namespace Avoid;

class CheckpointRerouteRouter : public ::testing::Test {
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

    Router *router;
};

TEST_F(CheckpointRerouteRouter, SetRoutingCheckpointsCausesReroute) {
    // Create two shapes with a connector between them.
    ShapeRef *leftShape = addShape({ 100, 100 }, { 200, 200 }, 1);
    ShapeRef *rightShape = addShape({ 400, 100 }, { 500, 200 }, 2);

    ConnEnd srcEnd(leftShape, 100);
    ConnEnd dstEnd(rightShape, 100);
    ConnRef *connRef = new ConnRef(router, srcEnd, dstEnd);

    // Initial route calculation.
    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/CheckpointReroute_SetRoutingCheckpointsCausesReroute-1");

    // Record the route before adding checkpoints.
    Polygon routeBefore = connRef->displayRoute();

    // Now set checkpoints on the connector — this should trigger rerouting.
    std::vector<Checkpoint> checkpoints;
    checkpoints.push_back(Checkpoint(Point(300, 300)));
    connRef->setRoutingCheckpoints(checkpoints);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/CheckpointReroute_SetRoutingCheckpointsCausesReroute-2");

    // Record the route after adding checkpoints.
    Polygon routeAfter = connRef->displayRoute();

    // The route must have changed because the checkpoint pulls the path
    // through (300, 300) which is below the direct horizontal path.
    EXPECT_NE(routeBefore.size(), routeAfter.size())
        << "Route should change after setRoutingCheckpoints + processTransaction";
}

TEST_F(CheckpointRerouteRouter, UpdatedCheckpointsCauseReroute) {
    // Create two shapes with a connector between them.
    ShapeRef *leftShape = addShape({ 100, 100 }, { 200, 200 }, 1);
    ShapeRef *rightShape = addShape({ 400, 100 }, { 500, 200 }, 2);

    ConnEnd srcEnd(leftShape, 100);
    ConnEnd dstEnd(rightShape, 100);
    ConnRef *connRef = new ConnRef(router, srcEnd, dstEnd);

    // Set initial checkpoint and route.
    std::vector<Checkpoint> checkpoints1;
    checkpoints1.push_back(Checkpoint(Point(300, 300)));
    connRef->setRoutingCheckpoints(checkpoints1);
    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/CheckpointReroute_UpdatedCheckpointsCauseReroute-1");

    Polygon routeWithCheckpoint1 = connRef->displayRoute();

    // Change the checkpoint to a different location.
    std::vector<Checkpoint> checkpoints2;
    checkpoints2.push_back(Checkpoint(Point(300, 50)));
    connRef->setRoutingCheckpoints(checkpoints2);
    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/CheckpointReroute_UpdatedCheckpointsCauseReroute-2");

    Polygon routeWithCheckpoint2 = connRef->displayRoute();

    // The two routes should differ because the checkpoint moved.
    bool routesAreIdentical = true;
    if (routeWithCheckpoint1.size() != routeWithCheckpoint2.size()) {
        routesAreIdentical = false;
    } else {
        for (size_t i = 0; i < routeWithCheckpoint1.size(); ++i) {
            if (!compare_float(routeWithCheckpoint1.ps[i].x, routeWithCheckpoint2.ps[i].x) ||
                !compare_float(routeWithCheckpoint1.ps[i].y, routeWithCheckpoint2.ps[i].y)) {
                routesAreIdentical = false;
                break;
            }
        }
    }

    EXPECT_FALSE(routesAreIdentical)
        << "Route should change when checkpoints are updated via setRoutingCheckpoints";
}
