#include "libavoid/libavoid.h"
#include "gtest/gtest.h"
#include "helpers.h"
/*
 * Test routing of mixed connections: router is configured as orthogonal and there are both orthogonal and polyline
 * connections.
 * */

typedef std::pair<Avoid::ShapeRef*, Avoid::ShapeConnectionPin*> ShapeWithPin;

class MixedConnTypes : public ::testing::Test {
protected:
    void SetUp() override {
        router = new Avoid::Router(Avoid::OrthogonalRouting | Avoid::PolyLineRouting);
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

    Avoid::ConnRef*  connectShapes(Avoid::ShapeRef *shape1, unsigned int shape1ConnId, Avoid::ShapeRef *shape2, Avoid::ConnType connType) {
        Avoid::ConnEnd srcPtEnd(shape1, shape1ConnId);
        Avoid::ConnEnd dstPtEnd(shape2, 100);
        Avoid::ConnRef *connection = new Avoid::ConnRef(router, srcPtEnd, dstPtEnd);
        connection->setRoutingType(connType);
        return connection;
    }

    Avoid::Router *router;
};

TEST_F(MixedConnTypes, RoutesAreCorrectForPolylineInOrthogonal) {
    // add two edges between shapes, remove one and another one should be exactly at the center between shapes
    ShapeWithPin leftShapeWithPin = addShape({ 100, 100 }, { 300, 300 }, 2, 5, 7);
    ShapeWithPin rightShapeWithPin = addShape({ 400, 400 }, { 600, 600 }, 9, 10, 11);

    Avoid::ConnRef *leftToRightConn = connectShapes(leftShapeWithPin.first, 100, rightShapeWithPin.first, Avoid::ConnType::ConnType_Orthogonal);
    Avoid::ConnRef *rightToLeftConn = connectShapes(rightShapeWithPin.first, 100, leftShapeWithPin.first, Avoid::ConnType::ConnType_PolyLine);

    router->processTransaction();
    router->deleteConnector(rightToLeftConn);
    router->processTransaction();

    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/MixedConnTypes_RoutesAreCorrectForPolylineInOrthogonal");

    std::vector<Avoid::Point> expectedleftToRight = { {200, 200}, {200, 500}, {500, 500} };
    EXPECT_THAT(leftToRightConn, IsEqualToRoute(expectedleftToRight));
}
