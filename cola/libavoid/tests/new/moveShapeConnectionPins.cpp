#include "libavoid/libavoid.h"
#include "gtest/gtest.h"
#include "helpers.h"
/*
 * Test routing after moving existing shape connection pins that have routes.
 * */

typedef std::pair<Avoid::ShapeRef*, Avoid::ShapeConnectionPin*> ShapeWithPin;

class MoveShapeConnectionPins : public ::testing::Test {
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

    ShapeWithPin addShape(Avoid::Point topLeft, Avoid::Point bottomRight, unsigned int shapeId, unsigned int connectionId, unsigned int connectionId2 = 0) {
        Avoid::Rectangle shapeRectangle(topLeft, bottomRight);
        Avoid::ShapeRef *shape = new Avoid::ShapeRef(router, shapeRectangle, shapeId);
        auto pin = new Avoid::ShapeConnectionPin(shape, 100,
                                                 Avoid::ATTACH_POS_CENTRE, Avoid::ATTACH_POS_CENTRE, true, 0.0, Avoid::ConnDirNone);
        pin->setExclusive(false);
        ShapeWithPin result(shape, pin);
        return result;
    }

    Avoid::ConnRef*  connectShapes(Avoid::ShapeRef *shape1, unsigned int shape1ConnId, Avoid::ShapeRef *shape2) {
        Avoid::ConnEnd srcPtEnd(shape1, shape1ConnId);
        Avoid::ConnEnd dstPtEnd(shape2, 100);
        Avoid::ConnRef *connection = new Avoid::ConnRef(router, srcPtEnd, dstPtEnd);
        return connection;
    }

    Avoid::Router *router;
};

TEST_F(MoveShapeConnectionPins, RoutesAreUpdatedAfterMovingShapeConnectionPins) {
    // add two edges between shapes, remove one and another one should be exactly at the center between shapes
    ShapeWithPin leftShapeWithPin = addShape({ 100, 100 }, { 300, 300 }, 2, 5, 7);
    ShapeWithPin rightShapeWithPin = addShape({ 400, 100 }, { 600, 300 }, 9, 10, 11);

    Avoid::ConnRef *leftToRightConn = connectShapes(leftShapeWithPin.first, 100, rightShapeWithPin.first);
    Avoid::ConnRef *rightToLeftConn = connectShapes(rightShapeWithPin.first, 100, leftShapeWithPin.first);

    router->processTransaction();
    Avoid::Point newPosition(1, 1);
    leftShapeWithPin.second->updatePosition(newPosition);
    router->processTransaction();

    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/MoveShapeConnectionPins_RoutesAreUpdatedAfterMovingShapeConnectionPins");

    std::vector<Avoid::Point> expectedleftToRight = { {300, 300}, {358, 300}, {358, 208}, { 500, 208 } };
    std::vector<Avoid::Point> expectedRightToLeft = { {500, 192}, {342, 192}, {342, 284}, { 300, 284 } };
    EXPECT_THAT(leftToRightConn, IsEqualToRoute(expectedleftToRight));
    EXPECT_THAT(rightToLeftConn, IsEqualToRoute(expectedRightToLeft));
}

// TODO: test moving shape connection pins with absolute coordinates

TEST_F(MoveShapeConnectionPins, RoutesAreUpdatedAfterMovingShapeAndShapeConnectionPins) {
    // add two edges between shapes, remove one and another one should be exactly at the center between shapes
    ShapeWithPin leftShapeWithPin = addShape({ 100, 100 }, { 300, 300 }, 2, 5, 7);
    ShapeWithPin rightShapeWithPin = addShape({ 400, 100 }, { 600, 300 }, 9, 10, 11);

    Avoid::ConnRef *leftToRightConn = connectShapes(leftShapeWithPin.first, 100, rightShapeWithPin.first);
    Avoid::ConnRef *rightToLeftConn = connectShapes(rightShapeWithPin.first, 100, leftShapeWithPin.first);

    router->processTransaction();
    router->moveShape(rightShapeWithPin.first, 100, 100);
    Avoid::Point newPosition(1, 1);
    rightShapeWithPin.second->updatePosition(newPosition);
    router->processTransaction();

    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/MoveShapeConnectionPins_RoutesAreUpdatedAfterMovingShapeAndShapeConnectionPins");

    std::vector<Avoid::Point> expectedleftToRight = { {192, 200}, { 192, 400}, {700, 400} };
    std::vector<Avoid::Point> expectedRightToLeft = { {700, 384}, {208, 384}, {208, 200} };
    EXPECT_THAT(leftToRightConn, IsEqualToRoute(expectedleftToRight));
    EXPECT_THAT(rightToLeftConn, IsEqualToRoute(expectedRightToLeft));
}
