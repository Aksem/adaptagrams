#include "libavoid/libavoid.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "helpers.h"

/*
 * Test routing between child shapes in a parent shape.
 * All child shapes have two or four shape connection pins(with the same class id per pair) on shape edges for
 * outgoing connections.
 * */

using namespace Avoid;


const int CONNECTIONPIN_INCOMING = 111;

class HierarchicalOrthogonalRouter : public ::testing::Test {
protected:
    void SetUp() override {
        router = new Router(OrthogonalRouting);
        router->setRoutingParameter(RoutingParameter::shapeBufferDistance, 4);
        router->setRoutingParameter(RoutingParameter::segmentPenalty, 50);
        router->setRoutingParameter(RoutingParameter::idealNudgingDistance, 4);
        router->setRoutingOption(RoutingOption::nudgeOrthogonalSegmentsConnectedToShapes, true);
        router->setRoutingOption(RoutingOption::nudgeOrthogonalTouchingColinearSegments, true);

        Rectangle parent({ -100, -400 }, { 900, 900 });
        new ShapeRef(router, parent, 1);
    }

    void TearDown() override {
        delete router;
    }

    ShapeRef* addChild(Point topLeft, Point bottomRight, unsigned int shapeId, unsigned int connectionId, unsigned int connectionId2 = 0) {
        Rectangle childRectangle(topLeft, bottomRight);
        ShapeRef *childShape = new ShapeRef(router, childRectangle, shapeId);
        new ShapeConnectionPin(childShape, connectionId, 0, 14, false, 0, ConnDirLeft);
        new ShapeConnectionPin(childShape, connectionId, 200, 14, false, 0, ConnDirRight);
        if (connectionId2 != 0) {
            new ShapeConnectionPin(childShape, connectionId2, 0, 56, false, 0, ConnDirLeft);
            new ShapeConnectionPin(childShape, connectionId2, 200, 56, false, 0, ConnDirRight);
        }
        new ShapeConnectionPin(childShape, CONNECTIONPIN_CENTRE,
                               ATTACH_POS_CENTRE, ATTACH_POS_CENTRE, true, 0.0, ConnDirAll);
        return childShape;
    }

    ShapeRef* addChildWithIncomingPins(Point topLeft, Point bottomRight, unsigned int shapeId, unsigned int connectionId, unsigned int connectionId2 = 0) {
        Rectangle childRectangle(topLeft, bottomRight);
        ShapeRef *childShape = new ShapeRef(router, childRectangle, shapeId);
        new ShapeConnectionPin(childShape, connectionId, 0, 14, false, 0, ConnDirLeft);
        new ShapeConnectionPin(childShape, connectionId, 200, 14, false, 0, ConnDirRight);
        if (connectionId2 != 0) {
            new ShapeConnectionPin(childShape, connectionId2, 0, 56, false, 0, ConnDirLeft);
            new ShapeConnectionPin(childShape, connectionId2, 200, 56, false, 0, ConnDirRight);
        }
        new ShapeConnectionPin(childShape, CONNECTIONPIN_INCOMING,
                               0, 150, false, 0.0, ConnDirLeft);
        new ShapeConnectionPin(childShape, CONNECTIONPIN_INCOMING,
                               200, 150, false, 0.0, ConnDirRight);
        return childShape;
    }

    ConnRef*  connectShapes(ShapeRef *shape1, unsigned int shape1ConnId, ShapeRef *shape2, unsigned int shape2ConnId) {
        ConnEnd srcPtEnd(shape1, shape1ConnId);
        ConnEnd dstPtEnd(shape2, shape2ConnId);
        ConnRef *connection = new ConnRef(router, srcPtEnd, dstPtEnd);
        return connection;
    }

    Router *router;
};

/* Checks: https://github.com/Aksem/adaptagrams/issues/3 */
TEST_F(HierarchicalOrthogonalRouter, TwoChildrenVertically) {
    // two children connected from pins on border to center
    ShapeRef *topChildShape = addChild({ 600, 500 }, { 800, 700 }, 2, 5);
    ShapeRef *bottomChildShape = addChild({ 600, 700 }, { 800, 900 }, 3, 6);

    ConnRef *bottomToTopConn = connectShapes(bottomChildShape, 6, topChildShape, CONNECTIONPIN_CENTRE);
    ConnRef *topToBottomConn = connectShapes(topChildShape, 5, bottomChildShape, CONNECTIONPIN_CENTRE);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HierarchicalOrthogonalRouter_TwoChildrenVertically");

    std::vector<Point> expectedBottomToTop = { {600, 714}, {596, 714}, {596, 600}, {700, 600} };
    EXPECT_THAT(bottomToTopConn, IsEqualToRoute(expectedBottomToTop));
    std::vector<Point> expectedTopToBottom = { {600, 514}, {592, 514}, {592, 800}, {700, 800} };
    EXPECT_THAT(topToBottomConn, IsEqualToRoute(expectedTopToBottom));
}

TEST_F(HierarchicalOrthogonalRouter, TwoChildrenVerticallyAllWithPins) {
    // two vertically positioned shapes connected from pins on borders to pins
    // on borders there are two pairs of connections to test both sides,
    // issues like https://github.com/Aksem/adaptagrams/issues/9 can
    // appear also on one side(e.g. closer to parent side)
    ShapeRef *topChildShape = addChildWithIncomingPins({ 650, 200 }, { 850, 400 }, 2, 5, 6);
    ShapeRef *bottomChildShape = addChildWithIncomingPins({ 650, 500 }, { 850, 700 }, 3, 7, 8);

    ConnRef *bottomToTopConn = connectShapes(bottomChildShape, 7, topChildShape, CONNECTIONPIN_INCOMING);
    ConnRef *bottomToTopConn2 = connectShapes(bottomChildShape, 8, topChildShape, CONNECTIONPIN_INCOMING);
    ConnRef *topToBottomConn = connectShapes(topChildShape, 5, bottomChildShape, CONNECTIONPIN_INCOMING);
    ConnRef *topToBottomConn2 = connectShapes(topChildShape, 6, bottomChildShape, CONNECTIONPIN_INCOMING);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HierarchicalOrthogonalRouter_TwoChildrenVerticallyAllWithPins");

    std::vector<Point> expectedBottomToTop = { {850, 514}, {854, 514}, {854, 350}, {850, 350} };
    EXPECT_THAT(bottomToTopConn, IsEqualToRoute(expectedBottomToTop));
    std::vector<Point> expectedBottomToTop2 = { {650, 556}, {646, 556}, {646, 350}, {650, 350} };
    EXPECT_THAT(bottomToTopConn2, IsEqualToRoute(expectedBottomToTop2));
    std::vector<Point> expectedTopToBottom = { {850, 214}, {858, 214}, {858, 650}, {850, 650} };
    EXPECT_THAT(topToBottomConn, IsEqualToRoute(expectedTopToBottom));
    std::vector<Point> expectedTopToBottom2 = { {650, 256}, {642, 256}, {642, 650}, {650, 650} };
    EXPECT_THAT(topToBottomConn2, IsEqualToRoute(expectedTopToBottom2));
}


TEST_F(HierarchicalOrthogonalRouter, ThreeChildrenVertically) {
    ShapeRef *topChildShape = addChild({ 600, 300 }, { 800, 500 }, 2, 5);
    ShapeRef *bottomChildShape = addChild({ 600, 600 }, { 800, 800 }, 3, 6);
    ShapeRef *leftChildShape = addChild({100, 400}, {300, 600}, 4, 7, 8);

    ConnRef *bottomToTopConn = connectShapes(bottomChildShape, 6, topChildShape, CONNECTIONPIN_CENTRE);
    ConnRef *topToBottomConn = connectShapes(topChildShape, 5, bottomChildShape, CONNECTIONPIN_CENTRE);
    ConnRef *leftToTopConn = connectShapes(leftChildShape, 7, topChildShape, CONNECTIONPIN_CENTRE);
    ConnRef *leftToBottomConn = connectShapes(leftChildShape, 8, bottomChildShape, CONNECTIONPIN_CENTRE);

    router->processTransaction();
    router->outputDiagramSVG(IMAGE_OUTPUT_PATH "output/HierarchicalOrthogonalRouter_ThreeChildrenVertically");

    std::vector<Point> expectedBottomToTop = { {600, 614}, {596, 614}, {596, 400}, {700, 400} };
    EXPECT_THAT(bottomToTopConn, IsEqualToRoute(expectedBottomToTop));
    std::vector<Point> expectedTopToBottom = { {600, 314}, {592, 314}, {592, 700}, {700, 700} };
    EXPECT_THAT(topToBottomConn, IsEqualToRoute(expectedTopToBottom));
    std::vector<Point> expectedLeftToTop = { {300, 407}, {700, 407} };
    EXPECT_THAT(leftToTopConn, IsEqualToRoute(expectedLeftToTop));
    std::vector<Point> expectedLeftToBottom = { {300, 600}, {700, 600} };
    EXPECT_THAT(leftToBottomConn, IsEqualToRoute(expectedLeftToBottom));
}
