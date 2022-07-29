#include "libavoid/libavoid.h"
#include "gtest/gtest.h"
#include "helpers.h"
/*
 * Test routing after changing connectors(add new one, delete or modify existing) in router.
 * */

using namespace Avoid;

class ConnectorChangesRouter : public ::testing::Test {
protected:
    void SetUp() override {
        router = new Router(OrthogonalRouting);
        router->setRoutingParameter(RoutingParameter::shapeBufferDistance, 16);
        router->setRoutingParameter(RoutingParameter::segmentPenalty, 50);
        router->setRoutingParameter(RoutingParameter::idealNudgingDistance, 16);
        router->setRoutingOption(RoutingOption::nudgeOrthogonalSegmentsConnectedToShapes, true);
        router->setRoutingOption(RoutingOption::nudgeOrthogonalTouchingColinearSegments, false);
    }

    void TearDown() override {
        delete router;
    }

    ShapeRef* addShape(Point topLeft, Point bottomRight, unsigned int shapeId, unsigned int connectionId, unsigned int connectionId2 = 0) {
        Rectangle shapeRectangle(topLeft, bottomRight);
        ShapeRef *shape = new ShapeRef(router, shapeRectangle, shapeId);
        auto pin = new ShapeConnectionPin(shape, 100,
                                          ATTACH_POS_CENTRE, ATTACH_POS_CENTRE, true, 0.0, ConnDirNone);
        pin->setExclusive(false);
        return shape;
    }

    ConnRef*  connectShapes(ShapeRef *shape1, unsigned int shape1ConnId, ShapeRef *shape2) {
        ConnEnd srcPtEnd(shape1, shape1ConnId);
        ConnEnd dstPtEnd(shape2, 100);
        ConnRef *connection = new ConnRef(router, srcPtEnd, dstPtEnd);
        return connection;
    }

    Router *router;
};

TEST_F(ConnectorChangesRouter, RoutesAreUpdatedAfterDeletingConnector) {
    // add two edges between shapes, remove one and another one should be exactly at the center between shapes
    ShapeRef *leftShape = addShape({ 100, 100 }, { 300, 300 }, 2, 5, 7);
    ShapeRef *rightShape = addShape({ 400, 100 }, { 600, 300 }, 9, 10, 11);

    ConnRef *leftToMiddleConn = connectShapes(leftShape, 100, rightShape);
    ConnRef *rightToLeftConn = connectShapes(rightShape, 100, leftShape);

    router->processTransaction();
    router->deleteConnector(rightToLeftConn);
    router->processTransaction();

    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/ConnectorChangesRouter_RoutesAreUpdatedAfterDeletingConnector");

    std::vector<Point> expectedleftToRight = { {200, 200}, {500, 200} };
    EXPECT_THAT(leftToMiddleConn, IsEqualToRoute(expectedleftToRight));
}

