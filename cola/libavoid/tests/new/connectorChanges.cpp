#include "libavoid/libavoid.h"
#include "gtest/gtest.h"
#include "helpers.h"
/*
 * Test routing after changing connectors(add new one, delete or modify existing) in router.
 * */

class ConnectorChangesRouter : public ::testing::Test {
protected:
    void SetUp() override {
        router = new Avoid::Router(Avoid::OrthogonalRouting);
        router->setRoutingParameter(Avoid::RoutingParameter::shapeBufferDistance, 16);
        router->setRoutingParameter(Avoid::RoutingParameter::segmentPenalty, 50);
        router->setRoutingParameter(Avoid::RoutingParameter::idealNudgingDistance, 16);
        router->setRoutingOption(Avoid::RoutingOption::nudgeOrthogonalSegmentsConnectedToShapes, true);
        router->setRoutingOption(Avoid::RoutingOption::nudgeOrthogonalTouchingColinearSegments, false);
    }

    void TearDown() override {
        delete router;
    }

    Avoid::ShapeRef* addShape(Avoid::Point topLeft, Avoid::Point bottomRight, unsigned int shapeId, unsigned int connectionId, unsigned int connectionId2 = 0) {
        Avoid::Rectangle shapeRectangle(topLeft, bottomRight);
        Avoid::ShapeRef *shape = new Avoid::ShapeRef(router, shapeRectangle, shapeId);
        auto pin = new Avoid::ShapeConnectionPin(shape, 100,
                                                 Avoid::ATTACH_POS_CENTRE, Avoid::ATTACH_POS_CENTRE, true, 0.0, Avoid::ConnDirNone);
        pin->setExclusive(false);
        return shape;
    }

    Avoid::ConnRef*  connectShapes(Avoid::ShapeRef *shape1, unsigned int shape1ConnId, Avoid::ShapeRef *shape2) {
        Avoid::ConnEnd srcPtEnd(shape1, shape1ConnId);
        Avoid::ConnEnd dstPtEnd(shape2, 100);
        Avoid::ConnRef *connection = new Avoid::ConnRef(router, srcPtEnd, dstPtEnd);
        return connection;
    }

    Avoid::Router *router;
};

TEST_F(ConnectorChangesRouter, RoutesAreUpdatedAfterDeletingConnector) {
    // add two edges between shapes, remove one and another one should be exactly at the center between shapes
    Avoid::ShapeRef *leftShape = addShape({ 100, 100 }, { 300, 300 }, 2, 5, 7);
    Avoid::ShapeRef *rightShape = addShape({ 400, 100 }, { 600, 300 }, 9, 10, 11);

    Avoid::ConnRef *leftToMiddleConn = connectShapes(leftShape, 100, rightShape);
    Avoid::ConnRef *rightToLeftConn = connectShapes(rightShape, 100, leftShape);

    router->processTransaction();
    router->deleteConnector(rightToLeftConn);
    router->processTransaction();

    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/ConnectorChangesRouter_RoutesAreUpdatedAfterDeletingConnector");

    std::vector<Avoid::Point> expectedleftToRight = { {200, 200}, {500, 200} };
    EXPECT_THAT(leftToMiddleConn, IsEqualToRoute(expectedleftToRight));
}

